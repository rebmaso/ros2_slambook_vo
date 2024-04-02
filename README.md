# ros_slambook_vo

This is my adaptation of the visual odometry code in [the SlamBook](https://github.com/gaoxiang12/slambook2). I'm using it as a baseline to modify small things and add features, such as relocalization and ros2 support.

Just add it to your ros2_ws and run `colcon build`

Then run with: 

```
# Launch rviz
gnome-terminal -- bash -c "cd ~/ros2_ws; source install/setup.bash; ros2 run rviz2 rviz2 -d src/slambk_vo/config/uhumans/uhumans.rviz; exec bash" &
# Launch node
gnome-terminal -- bash -c "cd ~/ros2_ws; source install/setup.bash; ros2 run slambk_vo main src/slambk_vo/config/uhumans/config.yaml; exec bash" &
# Play ros bag
gnome-terminal -- bash -c "cd ~/Datasets/uhumans; ros2 bag play  uHumans2_apartment_s1_00h_ros2.bag; exec bash" &
```

To debug the ros node: 

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```


```
# Launch node
gnome-terminal -- bash -c "cd ~/ros2_ws; source install/setup.bash; ros2 run --prefix 'gdb -ex run --args' slambk_vo main src/slambk_vo/config/uhumans/config.yaml; exec bash" &
# Play ros bag
gnome-terminal -- bash -c "cd ~/Datasets/uhumans; ros2 bag play  uHumans2_apartment_s1_00h_ros2.bag; exec bash" &
```

## Changelog

- added relocalization

- added ros 2 support

- added opencv4 suppoort


## TODOs

- test on synthetic rosbag with loop closures and gps like tartan air or mid air

- add prior to windowed BA ... this is wrong and leads to hessian singularities if not using LM

- old keyframes are stored w images landmarks ...and keep growing. bad. do not keep old images. just save descriptrors using dbow & use those descriptors to match loop kf

- add covisibility constraints for loop closure

- full loop closure w pose graph optimization (not just reloc)

- loop closure relocalization: do not add observations tying new frames to loop frames. do not modify map

- handle stuff in quques, or else just add to mapping thread, which is also triggered by each new kf

- maybe better to see why loop kf landmarks deleted and fix that.

- handle concurrency etter between reloc and windowed BA. perhaps shouldmake copy of graph.

- Use gloobal pose grraph to integrate GPS meas? Or tght integration (better, sets scale of monoslam)
