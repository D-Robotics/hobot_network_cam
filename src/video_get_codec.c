/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2024 D-Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"

// #include "common_utils.h"
#include "hobot_network_cam/video_get_codec.h"

Video_Decode_Data_Callback video_decode_callback;
int running = 1;
static int verbose = 0;
static int decode_output_exit = 0;

static void print_decode_params(DecodeParams *params)
{
    printf("Decode params...\n codec_type: %d, width: %d, height: %d, input_file: %s, output_file: %s\n", params->codec_type, params->width, params->height, params->input, params->output);
}
// Define JPEG start and end markers
#define JPEG_START_MARKER 0xFFD8
#define JPEG_END_MARKER 0xFFD9

// Macro to combine two bytes into a 16-bit value
#define MAKEWORD(a, b) ((uint16_t)(((a) << 8) | (b)))

// Function to read the next JPEG image from the file and return its data and size
int extract_jpeg(FILE *file, uint8_t **image_data, size_t *image_size)
{
    uint8_t buffer[1024];
    size_t bytes_read;
    int in_jpeg = 0;
    size_t jpeg_size = 0;
    size_t jpeg_capacity = 1024;
    uint8_t *jpeg_data = (uint8_t *)malloc(jpeg_capacity);
    if (!jpeg_data)
    {
        perror("Unable to allocate memory");
        return -1;
    }

    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0 || !feof(file))
    {
        for (size_t i = 0; i < bytes_read; i++)
        {
            // Check for JPEG start marker
            if (i < bytes_read - 1 && MAKEWORD(buffer[i], buffer[i + 1]) == JPEG_START_MARKER)
            {
                in_jpeg = 1;
                jpeg_size = 0; // Reset JPEG size for new image
            }

            // Write data to the current JPEG buffer
            if (in_jpeg)
            {
                if (jpeg_size >= jpeg_capacity)
                {
                    jpeg_capacity *= 2;
                    jpeg_data = (uint8_t *)realloc(jpeg_data, jpeg_capacity);
                    if (!jpeg_data)
                    {
                        perror("Unable to reallocate memory");
                        return -1;
                    }
                }
                jpeg_data[jpeg_size++] = buffer[i];
            }

            // Check for JPEG end marker
            if (i > 0 && MAKEWORD(buffer[i - 1], buffer[i]) == JPEG_END_MARKER && in_jpeg)
            {
                *image_data = jpeg_data;
                *image_size = jpeg_size;

                // Reset file position to start of the next JPEG
                fseek(file, -(long)(bytes_read - i - 1), SEEK_CUR);
                return 0;
            }
        }
        if (feof(file))
        {
            if (jpeg_data)
                free(jpeg_data);
            return -1;
        }
    }

    free(jpeg_data);
    return -1;
}

// av_open_stream: 打开视频流并找到最佳视频流索引
// p_param: 视频解码工作函数参数
// p_avContext: 保存 AVFormatContext 指针的指针
// p_avpacket: AVPacket 结构指针
// 返回值: 返回找到的最佳视频流索引，如果失败返回 -1
int32_t av_open_stream(DecodeParams *p_param, AVFormatContext **p_avContext, AVPacket *p_avpacket)
{
    int32_t ret = 0;
    uint8_t retry = 10;
    int32_t video_idx = -1;

    if (!p_param || !p_avContext || !p_avpacket)
        return -1;

    AVDictionary *option = NULL;

    // 设置 AVOption 字典
    av_dict_set(&option, "stimeout", "3000000", 0);
    av_dict_set(&option, "bufsize", "1024000", 0);
    av_dict_set(&option, "rtsp_transport", "tcp", 0);
    av_dict_set(&option, "probesize", "10000000", 0); // 设置 probesize 为 10M

    // 循环尝试打开视频流，最多重试 10 次
    do
    {
        ret = avformat_open_input(p_avContext, p_param->input, 0, &option);
        if (ret != 0)
        {
            printf("avformat_open_input: %d, retry\n", ret);
        }
    } while (--retry && ret != 0);

    if (!retry)
    {
        printf("Failed to avformat open %s\n", p_param->input);
        goto exit;
    }

    // 查找视频流的信息
    ret = avformat_find_stream_info(*p_avContext, 0);
    if (ret < 0)
    {
        printf("avformat_find_stream_info failed\n");
        return -1;
    }

    // 查找最佳视频流索引
    video_idx = av_find_best_stream(*p_avContext, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (video_idx < 0)
    {
        printf("av_find_best_stream failed, ret: %d\n", video_idx);
        return -1;
    }

    // 获取视频流的帧数
    p_param->frame_num = (*p_avContext)->streams[video_idx]->codec_info_nb_frames;
    printf("p_param->frame_num: %d\n", p_param->frame_num);

exit:
    return video_idx;
}

#define SET_BYTE(_p, _b) *_p++ = (unsigned char)_b;

#define SET_BUFFER(_p, _buf, _len)                                                                                                                                                                     \
    memcpy(_p, _buf, _len);                                                                                                                                                                            \
    (_p) += (_len);

int32_t av_build_dec_seq_header(uint8_t *pbHeader, const media_codec_id_t codec_id, const AVStream *st, int32_t *sizelength)
{
    AVCodecParameters *avc = st->codecpar;

    uint8_t *pbMetaData = avc->extradata;
    int32_t nMetaData = avc->extradata_size;
    uint8_t *p = pbMetaData;
    uint8_t *a = p + 4 - ((long)p & 3);
    uint8_t *t = pbHeader;
    int32_t size;
    int32_t sps, pps, i, nal;

    size = 0;
    *sizelength = 4; // default size length(in bytes) = 4
    if (codec_id == MEDIA_CODEC_ID_H264)
    {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01)
        {
            // check mov/mo4 file format stream
            p += 4;
            *sizelength = (*p++ & 0x3) + 1;
            sps = (*p & 0x1f); // Number of sps
            p++;
            for (i = 0; i < sps; i++)
            {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p + 2, nal - 2);
                p += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }

            pps = *(p++); // number of pps
            for (i = 0; i < pps; i++)
            {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p + 2, nal - 2);
                p += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }
        }
        else if (nMetaData > 3)
        {
            size = -1; // return to meaning of invalid stream data;
            for (; p < a; p++)
            {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1)
                {
                    // find startcode
                    size = avc->extradata_size;
                    if (pbMetaData && 0x00 == pbMetaData[size - 1])
                    {
                        size -= 1;
                    }
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    }
    else if (codec_id == MEDIA_CODEC_ID_H265)
    {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01)
        {
            static const int8_t nalu_header[4] = {0, 0, 0, 1};
            int32_t numOfArrays = 0;
            uint16_t numNalus = 0;
            uint16_t nalUnitLength = 0;
            uint32_t offset = 0;

            p += 21;
            *sizelength = (*p++ & 0x3) + 1;
            numOfArrays = *p++;

            while (numOfArrays--)
            {
                p++; // NAL type
                numNalus = (*p << 8) + *(p + 1);
                p += 2;
                for (i = 0; i < numNalus; i++)
                {
                    nalUnitLength = (*p << 8) + *(p + 1);
                    p += 2;
                    // if(i == 0)
                    {
                        memcpy(pbHeader + offset, nalu_header, 4);
                        offset += 4;
                        memcpy(pbHeader + offset, p, nalUnitLength);
                        offset += nalUnitLength;
                    }
                    p += nalUnitLength;
                }
            }

            size = offset;
        }
        else if (nMetaData > 3)
        {
            size = -1; // return to meaning of invalid stream data;

            for (; p < a; p++)
            {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) // find startcode
                {
                    size = avc->extradata_size;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    }
    else
    {
        SET_BUFFER(pbHeader, pbMetaData, nMetaData);
        size = nMetaData;
    }

    return size;
}

int32_t vp_decode_config_param(media_codec_context_t *context, media_codec_id_t codec_type, int32_t width, int32_t height)
{
    mc_video_codec_dec_params_t *params;
    context->encoder = false; // decoder output
    params = &context->video_dec_params;
    params->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    params->pix_fmt = MC_PIXEL_FORMAT_NV12;
    params->bitstream_buf_size = (width * height * 3 / 2 + 0x3ff) & ~0x3ff;
    params->bitstream_buf_count = 3;
    params->frame_buf_count = 3;

    switch (codec_type)
    {
    case MEDIA_CODEC_ID_H264:
        context->codec_id = MEDIA_CODEC_ID_H264;
        params->h264_dec_config.bandwidth_Opt = true;
        params->h264_dec_config.reorder_enable = true;
        params->h264_dec_config.skip_mode = 0;
        break;
    case MEDIA_CODEC_ID_H265:
        context->codec_id = MEDIA_CODEC_ID_H265;
        params->h265_dec_config.bandwidth_Opt = true;
        params->h265_dec_config.reorder_enable = true;
        params->h265_dec_config.skip_mode = 0;
        params->h265_dec_config.cra_as_bla = false;
        params->h265_dec_config.dec_temporal_id_mode = 0;
        params->h265_dec_config.target_dec_temporal_id_plus1 = 0;
        break;
    case MEDIA_CODEC_ID_MJPEG:
        context->codec_id = MEDIA_CODEC_ID_MJPEG;
        params->mjpeg_dec_config.rot_degree = MC_CCW_0;
        params->mjpeg_dec_config.mir_direction = MC_DIRECTION_NONE;
        params->mjpeg_dec_config.frame_crop_enable = false;
        break;
    case MEDIA_CODEC_ID_JPEG:
        context->codec_id = MEDIA_CODEC_ID_JPEG;
        params->jpeg_dec_config.frame_crop_enable = 0;
        params->jpeg_dec_config.rot_degree = MC_CCW_0;
        params->jpeg_dec_config.mir_direction = MC_DIRECTION_NONE;
        params->mjpeg_dec_config.frame_crop_enable = false;
        break;
    default:
        printf("Not Support decoding type: %d!\n", codec_type);
        return -1;
    }

    return 0;
}

// 解析配置文件
int parse_config(const char *filename, DecodeParams decode_params[], int *decode_streams)
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        printf("Failed to open config file\n");
        return 0;
    }

    char line[MAX_LINE_LENGTH];
    char section[MAX_LINE_LENGTH] = "";
    int vdec_stream_index = -1;

    while (fgets(line, sizeof(line), file))
    {
        // printf("line:%s\n",line);
        // 忽略空行和注释行
        if (line[0] == '#' || line[0] == ';' || line[0] == '\n' || line[0] == '\r')
            continue;

        // 去除行尾的换行符
        line[strcspn(line, "\r\n")] = 0;

        // 检测是否为新的节
        if (line[0] == '[')
        {
            sscanf(line, "[%[^]]", section);
            if (strncmp(section, "vdec_stream", strlen("vdec_stream")) == 0)
            {
                vdec_stream_index++;
            }
            continue;
        }

        char key[256], value[256];
        if (sscanf(line, "%[^=]=%[^\n]", key, value) != 2)
            continue;

        // 去除首尾空格
        char *trimmed_key = strtok(key, " \t");
        char *trimmed_value = strtok(value, " \t");

        if (strcmp(section, "decode") == 0)
        {
            if (strstr(line, "decode_streams") != NULL)
            {
                sscanf(line, "decode_streams = 0x%x", decode_streams);
            }
        }
        // 解析参数并填充结构体
        if (strncmp(section, "vdec_stream", strlen("vdec_stream")) == 0)
        {
            DecodeParams *params = &decode_params[vdec_stream_index];
            if (strcmp(trimmed_key, "codec_type") == 0)
                params->codec_type = atoi(trimmed_value);
            else if (strcmp(trimmed_key, "width") == 0)
                params->width = atoi(trimmed_value);
            else if (strcmp(trimmed_key, "height") == 0)
                params->height = atoi(trimmed_value);
            else if (strcmp(trimmed_key, "input") == 0)
                strcpy(params->input, trimmed_value);
            else if (strcmp(trimmed_key, "output") == 0)
                strcpy(params->output, trimmed_value);
        }
    }

    fclose(file);
    return 1;
}

static bool file_exists(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (file)
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}

int32_t vp_codec_init(media_codec_context_t *context)
{
    int32_t ret = 0;

    ret = hb_mm_mc_initialize(context);
    if (0 != ret)
    {
        printf("hb_mm_mc_initialize failed.\n");
        return -1;
    }

    ret = hb_mm_mc_configure(context);
    if (0 != ret)
    {
        printf("hb_mm_mc_configure failed.\n");
        hb_mm_mc_release(context);
        return -1;
    }

    printf("%s idx: %d, init successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);
    return 0;
}

int32_t vp_codec_deinit(media_codec_context_t *context)
{
    int32_t ret = 0;

    ret = hb_mm_mc_release(context);
    if (ret != 0)
    {
        printf("Failed to hb_mm_mc_release ret = %d \n", ret);
        return -1;
    }

    printf("%s idx: %d, deinit successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);
    return 0;
}

int32_t vp_codec_start(media_codec_context_t *context)
{
    int32_t ret = 0;
    mc_av_codec_startup_params_t startup_params = {0};

    ret = hb_mm_mc_start(context, &startup_params);
    if (ret != 0)
    {
        printf("%s:%d hb_mm_mc_start failed.\n", __FUNCTION__, __LINE__);
        return -1;
    }

    printf("%s idx: %d, start successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);
    return ret;
}

int32_t vp_codec_stop(media_codec_context_t *context)
{
    int32_t ret = 0;
    ret = hb_mm_mc_pause(context);
    if (ret != 0)
    {
        printf("Failed to hb_mm_mc_pause ret = %d \n", ret);
        return -1;
    }

    printf("%s idx: %d, stop successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);
    return ret;
}

int32_t vp_codec_set_input(media_codec_context_t *context, media_codec_buffer_t *frame_buffer, uint8_t *data, uint32_t data_size, int32_t eos)
{
    int32_t ret = 0;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL) || (!eos && (data == NULL)))
    {
        printf("codec param is NULL!\n");
        return -1;
    }

    buffer = frame_buffer;

    buffer->type = (context->encoder) ? MC_VIDEO_FRAME_BUFFER : MC_VIDEO_STREAM_BUFFER;
    ret = hb_mm_mc_dequeue_input_buffer(context, buffer, 2000);
    if (ret != 0)
    {
        printf("hb_mm_mc_dequeue_input_buffer failed ret = %d\n", ret);
        return -1;
    }

    if (context->encoder == false)
    {
        if (buffer->vstream_buf.size < data_size)
        {
            printf("The input stream/frame data is larger than the stream buffer size\n");
            hb_mm_mc_queue_input_buffer(context, buffer, 3000);
            return -1;
        }

        buffer->type = MC_VIDEO_STREAM_BUFFER;
        if (eos == 0)
        {
            buffer->vstream_buf.size = data_size;
            buffer->vstream_buf.stream_end = 0;
        }
        else
        {
            buffer->vstream_buf.size = 0;
            buffer->vstream_buf.stream_end = 1;
        }
        if (verbose)
        {
            printf("buffer->vstream_buf.size: %d\n", buffer->vstream_buf.size);
            printf("buffer->vstream_buf.vir_ptr: %p\n", buffer->vstream_buf.vir_ptr);
        }

        memcpy(buffer->vstream_buf.vir_ptr, data, data_size);
    }

    ret = hb_mm_mc_queue_input_buffer(context, buffer, 2000);
    if (ret != 0)
    {
        printf("hb_mm_mc_queue_input_buffer failed, ret = 0x%x\n", ret);
        return -1;
    }

    if (verbose)
        printf("%s idx: %d, set input successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);
    return ret;
}

int32_t vp_codec_get_output(media_codec_context_t *context, media_codec_buffer_t *frame_buffer, media_codec_output_buffer_info_t *buffer_info, int32_t timeout)
{
    int32_t ret = 0;
    media_codec_output_buffer_info_t *info = NULL;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL) || (buffer_info == NULL))
    {
        printf("codec param is NULL\n");
        return -1;
    }
    buffer = frame_buffer;
    info = buffer_info;

    ret = hb_mm_mc_dequeue_output_buffer(context, buffer, info, timeout);
    if (ret != 0 && ret != -268435443) // Check for timeout error
    {
        printf("%s idx: %d, %s ret = %d\n", context->encoder ? "Encode" : "Decode", context->instance_index,
               ret == -1 ? "hb_mm_mc_dequeue_output_buffer failed" : "hb_mm_mc_dequeue_output_buffer encountered an error", ret);
        return -1;
    }
    else if (ret == -268435443)
    {
        printf("%s idx: %d, %s\n", context->encoder ? "Encode" : "Decode", context->instance_index, "hb_mm_mc_dequeue_output_buffer timed out (possibly normal exit due to lack of data)");
        return -1;
    }
    if ((!context->encoder) && (buffer->type != MC_VIDEO_FRAME_BUFFER))
    {
        if (buffer != NULL)
        {
            ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
            if (ret != 0)
            {
                printf("idx: %d, hb_mm_mc_queue_output_buffer failed ret = %d \n", context->instance_index, ret);
                return -1;
            }
        }
        return -1;
    }

    if (context->codec_id >= MEDIA_CODEC_ID_H264 || context->codec_id <= MEDIA_CODEC_ID_JPEG)
    {
        if (context->encoder == 0) // decoder
        {
            if (info->video_frame_info.decode_result == 0 || buffer->vframe_buf.size == 0)
            {
                if (buffer != NULL)
                {
                    ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
                    if (ret != 0)
                    {
                        printf("idx: %d, hb_mm_mc_queue_output_buffer failed ret = %d\n", context->instance_index, ret);
                        return -1;
                    }
                }
                return -1;
            }

            if (verbose)
            {
                printf("Decodec idx: %d type:%d get frame size:%d\n", context->instance_index, context->codec_id, buffer->vframe_buf.size);
            }
        }
    }

    return ret;
}

int32_t vp_codec_release_output(media_codec_context_t *context, media_codec_buffer_t *frame_buffer)
{
    int32_t ret = 0;
    media_codec_buffer_t *buffer = NULL;

    if ((context == NULL) || (frame_buffer == NULL))
    {
        printf("codec param is NULL!\n");
        return -1;
    }
    buffer = frame_buffer;

    if (verbose)
    {
        printf("%s idx: %d type:%d, buffer:%p\n", context->encoder ? "Encode" : "Decode", context->instance_index, context->codec_id, buffer);
    }

    if (buffer != NULL)
    {
        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0)
        {
            printf("idx: %d, hb_mm_mc_queue_output_buffer failed ret = %d \n", context->instance_index, ret);
            return -1;
        }
    }

    return ret;
}

media_codec_buffer_t ouput_buffer = {0};

int32_t decode_output_video(media_codec_context_t *context, DecodeParams *params)
{
    int32_t ret = 0;
    media_codec_output_buffer_info_t info;

    memset(&ouput_buffer, 0x0, sizeof(media_codec_buffer_t));
    while (decode_output_exit)
    {
        ret = vp_codec_get_output(context, &ouput_buffer, &info, 2000);
        if (ret != 0)
        {
            // wait for each frame for decoding
            // usleep(30 * 1000);
            printf("decode output continue\n");
            continue;
        }

        if (video_decode_callback != NULL)
        {
            video_decode_callback(&ouput_buffer);
        }

        vp_codec_release_output(context, &ouput_buffer);
    }

    return 0;
}

// 视频解码函数
int32_t decode_h264_h265_mjpeg_video(media_codec_context_t *context, DecodeParams *params)
{
    int32_t ret = 0;
    media_codec_buffer_t input_buffer = {0};
    AVFormatContext *avContext = NULL;
    AVPacket avpacket = {0};
    int32_t video_idx = -1;
    int32_t firstPacket = 1;
    bool eos = false;
    int32_t seqHeaderSize = 0;
    uint8_t *seqHeader = NULL;

    video_idx = av_open_stream(params, &avContext, &avpacket);
    if (video_idx < 0)
    {
        printf("failed to av_open_stream\n");
        goto err_av_open;
    }

    while (1)
    {
        mc_inter_status_t pstStatus;
        hb_mm_mc_get_status(context, &pstStatus);

        // wait for each frame for decoding
        usleep(30 * 1000);

        if (!avpacket.size)
        {
            ret = av_read_frame(avContext, &avpacket);
        }

        if (ret < 0)
        {
            if (ret == AVERROR_EOF || avContext->pb->eof_reached == true)
            {
                printf("No more valid data available for decoding, "
                       "avpacket.size: %d."
                       " Decoder will exit due to timeout after fetching"
                       " decoded output.\n",
                       avpacket.size);

                eos = true;
            }
            else
            {
                printf("Failed to av_read_frame error(0x%08x)\n", ret);
            }
            break;
        }
        else
        {
            seqHeaderSize = 0;
            if (firstPacket)
            {
                AVCodecParameters *codec;
                int32_t retSize = 0;
                codec = avContext->streams[video_idx]->codecpar;
                seqHeader = (uint8_t *)calloc(1U, codec->extradata_size + 1024);
                if (seqHeader == NULL)
                {
                    printf("Failed to mallock seqHeader\n");
                    eos = true;
                    break;
                }

                seqHeaderSize = av_build_dec_seq_header(seqHeader, context->codec_id, avContext->streams[video_idx], &retSize);
                if (seqHeaderSize < 0)
                {
                    printf("Failed to build seqHeader\n");
                    eos = true;
                    break;
                }
                firstPacket = 0;
            }
            if (avpacket.size <= context->video_dec_params.bitstream_buf_size)
            {
                if (seqHeaderSize)
                {
                    vp_codec_set_input(context, &input_buffer, seqHeader, seqHeaderSize, eos);
                }
                else
                {
                    vp_codec_set_input(context, &input_buffer, avpacket.data, avpacket.size, eos);
                    // if (log_ctrl_level_get(NULL) == LOG_TRACE) {
                    // 	print_avpacket_info(&avpacket);
                    // 	vp_codec_print_media_codec_output_buffer_info(&frame);
                    // }
                    av_packet_unref(&avpacket);
                    avpacket.size = 0;
                }
            }
            else
            {
                printf("The external stream buffer is too small!\n"
                       "avpacket.size:%d, buffer size:%d\n",
                       avpacket.size, context->video_dec_params.bitstream_buf_size);
                eos = true;
                break;
            }

            if (seqHeader)
            {
                free(seqHeader);
                seqHeader = NULL;
            }
        }
    }

    if (eos)
    {
        vp_codec_set_input(context, &input_buffer, seqHeader, seqHeaderSize, eos);
    }

    if (seqHeader)
    {
        free(seqHeader);
        seqHeader = NULL;
    }

err_av_open:
    if (avContext)
        avformat_close_input(&avContext);
    return 0;
}

// 解码一连串的jpeg图像
int32_t decode_jpeg_sequence(media_codec_context_t *context, DecodeParams *params)
{
    media_codec_buffer_t input_buffer = {0};
    uint8_t *image_data;
    size_t image_size;

    printf("%s idx: %d, start successful\n", context->encoder ? "Encode" : "Decode", context->instance_index);

    FILE *fp_input = fopen(params->input, "rb");
    if (NULL == fp_input)
    {
        printf("Failed to open input file: %s\n", params->input);
        return -1;
    }

    FILE *fp_output = fopen(params->output, "w+b");
    if (NULL == fp_output)
    {
        printf("Failed to open output file: %s\n", params->output);
        return -1;
    }

    while (1)
    {
        if (extract_jpeg(fp_input, &image_data, &image_size) == 0)
        {
            vp_codec_set_input(context, &input_buffer, image_data, image_size, 0);
            free(image_data);
        }
        else
        {
            break;
        }
    }

    if (fp_output)
    {
        fclose(fp_output);
    }

    if (fp_input)
    {
        fclose(fp_input);
    }

    return 0;
}

void *decode_output_thread(void *arg)
{
    DecodeParams *params = (DecodeParams *)arg;

    printf("Decoding output video...\n");
    decode_output_video(&params->decode_context, params);
    pthread_exit(NULL);
}

// 解码线程函数
void *decode_thread(void *arg)
{
    DecodeParams *params = (DecodeParams *)arg;

    printf("Decoding video...\n");
    if (params->codec_type == MEDIA_CODEC_ID_JPEG)
    {
        decode_jpeg_sequence(&params->decode_context, params);
    }
    else
    {
        decode_h264_h265_mjpeg_video(&params->decode_context, params);
    }
    pthread_exit(NULL);
}

int video_set_callback(Video_Decode_Data_Callback callback)
{
    video_decode_callback = callback;
    return 0;
}

int decode_streams = 0x0;
DecodeParams decode_params[MAX_STREAMS];
// 创建解码输入线程
pthread_t decode_threads[MAX_STREAMS];
// 创建解码输出线程
pthread_t decode_outout_threads[MAX_STREAMS];

int video_decode_init()
{
    int32_t ret = 0;
    char *config_file = "codec_config.ini";
    // 解析配置文件

    // Do something with the parsed options
    printf("Config file: %s\n", config_file);
    if (!file_exists(config_file))
    {
        fprintf(stderr, "Error: Configuration file '%s' does not exist.\n", config_file);
        exit(EXIT_FAILURE);
    }
    if (!parse_config(config_file, decode_params, &decode_streams))
    {
        printf("Failed to parse config file\n");
        return 1;
    }
    printf("decode_streams: 0x%x\n", decode_streams);
    // 初始化解码器
    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
            print_decode_params(&decode_params[i]);
            ret = vp_decode_config_param(&decode_params[i].decode_context, decode_params[i].codec_type, decode_params[i].width, decode_params[i].height);
            if (ret != 0)
            {
                printf("Decode config param error, type:%d width:%d height:%d\n", decode_params[i].codec_type, decode_params[i].width, decode_params[i].height);
            }
            ret = vp_codec_init(&decode_params[i].decode_context);
            if (ret != 0)
            {
                printf("Decode vp_codec_init error(%d)\n", i);
                return -1;
            }
            printf("Init video decode instance %d successful\n", decode_params[i].decode_context.instance_index);
            ret = vp_codec_start(&decode_params[i].decode_context);
            if (ret != 0)
            {
                printf("Encode vp_codec_start error\n");
                return -1;
            }
        }
    }
    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
#if 0
			if (!file_exists(decode_params[i].input)) {
				fprintf(stderr, "Error: decode input file '%s' does not exist.\n", decode_params[i].input);
				exit(EXIT_FAILURE);
			}
#endif
            pthread_create(&decode_threads[i], NULL, decode_thread, &decode_params[i]);
        }
    }
    decode_output_exit = 1;

    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
            pthread_create(&decode_outout_threads[i], NULL, decode_output_thread, &decode_params[i]);
        }
    }
    return ret;
}

int video_decode_deinit()
{
    int32_t ret = 0;
    // 等待所有解码输入线程结束
    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
            pthread_join(decode_threads[i], NULL);
        }
    }

    // 等待1000豪秒，让解码器完成所有帧的解码并被output
    usleep(1000 * 1000);
    decode_output_exit = 0;

    // 等待所有解码输出线程结束
    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
            pthread_join(decode_outout_threads[i], NULL);
        }
    }

    for (int i = 0; i < MAX_STREAMS; i++)
    {
        if (decode_streams & (1 << i))
        {
            ret = vp_codec_stop(&decode_params[i].decode_context);
            if (ret != 0)
            {
                printf("Encode vp_codec_stop error\n");
                return -1;
            }
            ret = vp_codec_deinit(&decode_params[i].decode_context);
            if (ret != 0)
            {
                printf("Decode vp_codec_deinit error(%d)\n", i);
                return -1;
            }
            printf("deinit video decode instance %d successful\n", decode_params[i].decode_context.instance_index);
        }
    }
    return ret;
}
