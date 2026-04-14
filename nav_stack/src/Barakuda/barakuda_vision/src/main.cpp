#include "barakuda_vision/barakuda_vision.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "barakuda_vision");

    // Node handle
    ros::NodeHandle n;


    BarakudaVision BarakudaVision(n);
    /*
    int rate;
    ros::NodeHandle private_n("~");
    private_n.param<int>("rate", rate, int(10));
    ros::Rate r(rate);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }

    */

    ros::spin();
    return 0;
}
