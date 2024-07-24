#include "apd.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"offb_node");
    ApdControl apd;

    ros::spin();
}