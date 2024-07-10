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
~~~
