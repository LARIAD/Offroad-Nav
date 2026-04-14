#include "barakuda_manager.h"
#include "barakuda_logger.h"
#include "barakuda_manager/WaypointAction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint");

    // Node handle
    ros::NodeHandle n;
    
    // Create the WP action server
    //barakuda_manager::WaypointAction WaypointAction();

    // Barakuda manager node
    BarakudaManager BarakudaManager(n);

    // Barakuda Logger node
    BarakudaLogger BarakudaLogger(n);

    // rate in Hz
    int rate = 20;

    int record_count = 0;
    int log_count = 0;
    // Record rate in Hz
    int record_rate = 1;
    // Log rate in Hz
    int log_rate = 5;

    ros::Rate r(rate);

    while (ros::ok())
    {  
        
        // Every reconds, call the record function if the record is on
        if(BarakudaManager.getGnssRecord()){
            if(record_count >= (rate/record_rate)){
            BarakudaManager.gnssRecord();
            record_count = 0;
            }
        record_count++;
        }

        // Call Log function
        if(log_count >= (rate/log_rate)){
            //
            // 
            // BarakudaLogger.sendLog();
            log_count = 0;
        }
        log_count++;

        // Send robot status
        BarakudaManager.miradorStatusPublisher();

        r.sleep();
        ros::spinOnce();
    }
    
    //ros::spin();
    return 0;
}
