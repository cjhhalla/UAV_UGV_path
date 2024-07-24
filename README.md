# UAV_UGV_path
## UAV in px4 , UGV in Jackal sim
+ Mavros and requirements
~~~shell
    $ sudo apt-get update && apt-get upgrade -y
    $ sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
    $ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    $ chmod +x install_geographiclib_datasets.sh
    $ sudo ./install_geographiclib_datasets.sh
    
    $ cd ~/ && git clone https://github.com/PX4/PX4-Autopilot.git
    $ cd PX4-Autopilot
    $ git reset --hard 6823cbc
    $ git submodule update --init --recursive
    
    $ source ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
    $ cd ~/PX4-Autopilot
    $ sudo apt install libgstreamer-plugins-base1.0-dev ros-<distro>-gazebo-plugins
    $ DONT_RUN=1 make px4_sitl_default gazebo

    $ source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
    $ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    $ roslaunch px4 mavros_posix_sitl.launch
    # more detail in https://github.com/engcang/mavros-gazebo-application.git
~~~
+ Run code
~~~shell
     # roslaunch px4 mavros_posix_sitl.launch
     $ mkdir -p ~/catkin_ws/src
     $ cd ~/catkin_ws/src
     $ git clone <this-repo>
     $ cd ~/catkin_ws
     $ catkin build url_uavugv
     $ source ~/catkin_ws/devel/setup.bash
     $ roslaunch url_uavugv uav_circle.launch # uav_rect.launch # uav_figure8.launch
~~~
+ Jackal sim and requirements
~~~shell
    $ sudo apt-get install ros-<distro>-jackal-simulator ros-<distro>-jackal-desktop ros-<distro>-jackal-navigation
    $ source /opt/ros/<distro>/setup.bash
    $ roslaunch jackal_gazebo jackal_world.launch
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone <this-repo>
    $ cd ~/catkin_ws
    $ catkin build url_uavugv
    $ source ~/catkin_ws/devel/setup.bash
    $ roslaunch url_uavugv ugv_circle.launch # ugv_rect.launch # ugv_figure8.launch

+ Update
