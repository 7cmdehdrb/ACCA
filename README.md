# Autonomous Vehicle for Localization and LiDAR (ROS-Melodic)

> Autonomous Vihicle Project with ERP42

## Downloaded Packages

- [ROS_erp42_driver](https://github.com/kemjensak/erp42_driver)

- [erp42_telop/erp42_vihicle](https://github.com/jdj2261/ERP42-ROS)

- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

- [follow_waypoints](https://github.com/danielsnider/follow_waypoints)

<detail>

#### Save waypoints

```
rostopic pub /path_ready std_msgs/Empty -1
```

#### Start waypoints

```
rostopic pub /start_journey std_msgs/Empty -1
```

</detail>

- [geometry2]

- [hdl_global_localization](https://github.com/koide3/hdl_global_localization)

- [hdl_localization](https://github.com/koide3/hdl_localization)

- [LMS1xx]

- [navigation](https://github.com/ros-planning/navigation/tree/melodic-devel)

- [ndt_omp](https://github.com/koide3/ndt_omp)

- [nmea_msgs](https://github.com/ros-drivers/nmea_msgs)

- [ntrip_ros](https://github.com/ros-agriculture/ntrip_ros)

- [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/melodic-devel)

- [rtcm_msgs](https://github.com/tilk/rtcm_msgs)

- [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)

- [ublox_f9p]

- [xsens_ros_mti_driver](https://github.com/esteve/xsens_ros_mti_driver)

#### Dependency package install

```
rosdep install --from-paths src --ignore-src -r -y
```
