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
~~~
+ Run code
~~~shell
     $ git clone <this-repo>
     $ cd uav
     $ source /opt/ros/<distro>/setup.bash
     $ python rect.py
~~~
