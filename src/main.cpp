// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hobot_network_cam/video_get_codec.h"

class NetworkCamNode : public rclcpp::Node
{
public:
    NetworkCamNode() : Node("network_cam_node")
    {
        // ======================================================================================================================================
        RCLCPP_INFO(this->get_logger(), "=> init network_cam_node");

        // ======================================================================================================================================
        // pub & sub
        stereo_msg_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_combine_raw", 10);

        // ======================================================================================================================================
        int ret = 0;
        ret = video_set_callback((Video_Decode_Data_Callback)decode_data_callback);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "=> video_set_callback failed");
            rclcpp::shutdown();
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "=> video_set_callback success");
        }
        ret = video_decode_init();
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "=> video_decode_init failed");
            rclcpp::shutdown();
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "=> video_decode_init success");
        }

        // while (rclcpp::ok())
        // {
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));
        // }
        // ret = video_decode_deinit();
        // if (ret != 0)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "=> video_decode_deinit failed");
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "=> video_decode_deinit success");
        // }

        // rclcpp::on_shutdown(
        //     [this]()
        //     {
        //         int ret = video_decode_deinit();
        //         if (ret != 0)
        //         {
        //             std::cout <<  "=> video_decode_deinit failed" << std::endl;
        //         }
        //         else
        //         {
        //             std::cout <<  "=> video_decode_deinit success" << std::endl;
        //         }
        //     });
    }

    // ~NetworkCamNode()
    // {
    //     int ret = video_decode_deinit();
    //     if (ret != 0)
    //     {
    //         std::cout <<  "=> video_decode_deinit failed" << std::endl;
    //         // RCLCPP_ERROR(this->get_logger(), "=> video_decode_deinit failed");
    //     }
    //     else
    //     {
    //         std::cout <<  "=> video_decode_deinit success" << std::endl;
    //         // RCLCPP_INFO(this->get_logger(), "=> video_decode_deinit success");
    //     }
    // }

private:
    static int decode_data_callback(media_codec_buffer_t *ouput_buffer)
    {
        if (rclcpp::ok())
        {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("network_cam_node"), "=> video_decode_deinit success");
            // std::cout << "=> decode_data_callback" << std::endl;
            // 分离左右拼接的 NV12 数据
            // 获取原始图像数据
            uint8_t *y_data = ouput_buffer->vframe_buf.vir_ptr[0];
            uint8_t *uv_data = ouput_buffer->vframe_buf.vir_ptr[1];
            size_t full_width = ouput_buffer->vframe_buf.width; // 原图宽度（左右拼接后）
            size_t full_height = ouput_buffer->vframe_buf.height;

            size_t out_y_size = (full_width / 2) * full_height * 2;
            size_t out_uv_size = (full_width / 2) * full_height;
            uint8_t *nv12_vstack = new uint8_t[out_y_size + out_uv_size];

            hstack_to_vstack_nv12(nv12_vstack, y_data, uv_data, full_width, full_height);

            // FILE *fp_output = NULL;
            // fp_output = fopen("640x960.nv12", "w+b");
            // if (!fp_output)
            // {
            //     printf("main fp_output open failed\n");
            //     return -1;
            // }
            // if (fp_output)
            // {
            //     fwrite(nv12_vstack, y_size + uv_size, 1, fp_output);
            // }
            // fclose(fp_output);

            // 创建一个 ROS 2 图像消息
            auto msg = std::make_shared<sensor_msgs::msg::Image>();
            // 使用 std::chrono 获取当前时间戳
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            msg->header.stamp.sec = millis / 1000;                 // 秒部分
            msg->header.stamp.nanosec = (millis % 1000) * 1000000; // 毫秒部分转换为纳秒
            msg->header.frame_id = "stereo_camera_frame";
            msg->height = full_height * 2; // 高度变为 960（上下拼接）
            msg->width = full_width / 2;   // 宽度变为 640（上下拼接）
            msg->encoding = "nv12";        // NV12 编码
            msg->is_bigendian = false;
            msg->step = msg->width; // 每行数据的字节数（对于 NV12，每行是宽度字节数）

            // 将拼接后的 Y 和 UV 数据填充到 ROS 图像消息
            msg->data.resize(out_y_size + out_uv_size);
            std::memcpy(msg->data.data(), nv12_vstack, out_y_size + out_uv_size); // 填充数据
            // std::memcpy(msg->data.data(), ouput_buffer->vframe_buf.vir_ptr[0], y_size);           // 填充数据
            // std::memcpy(msg->data.data() + y_size, ouput_buffer->vframe_buf.vir_ptr[1], uv_size); // 填充数据

            // 发布图像消息
            stereo_msg_pub_->publish(*msg);

            // 释放内存
            delete[] nv12_vstack;
            nv12_vstack = nullptr;
        }
        return 0;
    }

    static int hstack_to_vstack_nv12(uint8_t *nv12_vstack,   // 输出数组，外部分配好内存
                                     const uint8_t *y_data,  // 输入 Y 分量指针
                                     const uint8_t *uv_data, // 输入 UV 分量指针
                                     size_t full_width,      // 输入图像宽度（左右拼接后的宽度）
                                     size_t full_height      // 输入图像高度
    )
    {
        size_t single_width = full_width / 2;
        size_t single_height = full_height;

        size_t y_stride = full_width;
        size_t uv_stride = full_width;

        size_t out_y_stride = single_width;
        size_t out_uv_stride = single_width;

        size_t out_y_size = single_width * single_height * 2; // 640*480*2
        // size_t out_uv_size = single_width * single_height; // 实际不需要使用该值

        // === 拷贝 Y 分量 ===
        for (size_t row = 0; row < single_height; row++)
        {
            // 拷贝左图Y（上半部分）
            std::memcpy(nv12_vstack + row * out_y_stride, y_data + row * y_stride, single_width);

            // 拷贝右图Y（下半部分）
            std::memcpy(nv12_vstack + (row + single_height) * out_y_stride, y_data + row * y_stride + single_width, single_width);
        }

        // === 拷贝 UV 分量 ===
        for (size_t row = 0; row < single_height / 2; row++)
        {
            // 拷贝左图UV（上半部分）
            std::memcpy(nv12_vstack + out_y_size + row * out_uv_stride, uv_data + row * uv_stride, single_width);

            // 拷贝右图UV（下半部分）
            std::memcpy(nv12_vstack + out_y_size + (row + single_height / 2) * out_uv_stride, uv_data + row * uv_stride + single_width, single_width);
        }
    }

    // stereo image publisher
    static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_msg_pub_;
};

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr NetworkCamNode::stereo_msg_pub_ = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NetworkCamNode>();
    rclcpp::spin(node);
    // node.reset();
    rclcpp::shutdown();
    return 0;
}