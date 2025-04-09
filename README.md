# hobot_network_cam

## 描述

启动网络相机，并发布双目图像

## 编译

1. 交叉编译，在PC端执行

```shell
bash ./robot_dev_config/build.sh -p X5 -s hobot_network_cam
```

## 运行

(1) 运行s316相机之前，需要通过网线将相机和RDK板端连接，相机12V电源供电

(2) 将配置文件拷贝到运行目录

```bash
cp -rv ./hobot_network_cam/config/* ./
```

(3) 在包含配置文件的目录运行如下命令，发布双目图像

```bash
ros2 launch hobot_network_cam pub_stereo_imgs.launch.py
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看输出的双目图像


(4) 在包含配置文件的目录运行如下命令，发布双目图像+地瓜双目算法

```bash
ros2 launch hobot_network_cam test_stereo_custom_rectify.launch.py \
stereonet_model_file_path:=./x5baseplus_alldata_woIsaac_yuv444.bin postprocess:=v2 \
render_type:=1 depth_need_filter:=False resize_before_rectify:=True
```

在浏览器输入[http://ip:8000](http://ip:8000)即可查看双目算法的结果

