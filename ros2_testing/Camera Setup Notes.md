#### Ros Server

## Followed:
https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/blob/rolling/README.md

Some strange issues with this guide. I had to `cd ..` to the _ws folder before running

 `rosdep install --from-paths src --ignore-src -r -y` 

If this happens make sure to cd back into src folder


I also had to install vcs (https://github.com/dirk-thomas/vcstool?tab=readme-ov-file#how-to-install-vcstool). in summary:

`curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash`

`sudo apt-get update`

`sudo apt-get install python3-vcstool`


Fianally make sure to source both the local ros commands with:

`source install/setup.bash` (run from the _ws folder in terminal)

and the global ros commands:

`source /opt/ros/humble/setup.bash`

* Note you might need to change humble to your version



## Notes for future seed teams:
Make sure to edit the name of the ridged body in motive to what is set in:

`mocap4r2_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml`

Without any changes the current name is `car`




## Known issues/quirks
When logging on via SSH two lines are added to bash.rc setting the ros domain to 2 and sourcing the root ros directory


When logging in via SSH issues with not sourcing bashrc
Recomended workaround: (Using MobaExterm for SSH)
Added source bashrc to startup macros in moba xterm




## ROS Mocap General Notes

Mocap to ros:
https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/blob/rolling/README.md

Currently using NatNet multicast
