# origin_rm_vision
第一代君瞄注释版

## Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- OpenCV 4.5.3

## Build
```bash
cd ~/rm_vision_ws/src
git clone <this repo>
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Launch
```bash
source ~/rm_vision_ws/install/setup.bash
# launch camera
ros2 launch mindvision_camera mv_launch.py
# launch serial driver
ros2 launch rm_serial_driver serial_driver.launch.py
# launch auto aim
ros2 launch auto_aim_bringup auto_aim.launch.py
```

## Packages

- [rm_auto_aim](rm_auto_aim)

    自瞄模块

    - [armor_detection](rm_auto_aim/armor_detection)

        装甲板识别

    - [armor_tracker](rm_auto_aim/armor_tracker)
    
        装甲板追踪

    - [auto_aim_bringup](rm_auto_aim/auto_aim_bringup)

        自瞄模块启动
    
    - [auto_aim_interface](rm_auto_aim/auto_aim_interface)

        自瞄模块接口

- [rm_serial_driver](rm_serial_driver)

    串口通信库

- [infantry_msgs](infantry_msgs)

    自定义消息

- [mindvision_camera](mindvision_camera)

    迈德威视相机驱动

## Contributors

- [Chen Jun](https://github.com/chenjunnn)