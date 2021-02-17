#include "boxbot_hw.h"
#include "boxbot_control_loop.h"


int main(int argc, char** argv) {

    #ifdef BOXBOT_HW_DEBUG_
    // change the logger level. For debugging purpose
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    #endif

    ros::init(argc, argv, "boxbot_hw");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(5); //multiple threads here to prevent blocking the control loop
    spinner.start();

    std::shared_ptr<boxbot_hw::BoxbotHW> hardware_interface(
        new boxbot_hw::BoxbotHW(nh)
    );
    hardware_interface->init();

    ros::ServiceServer home_srv = nh.advertiseService("boxbot_home", &boxbot_hw::BoxbotHW::home, hardware_interface.get());
    ros::ServiceServer servo_srv = nh.advertiseService("boxbot_rotate", &boxbot_hw::BoxbotHW::rotate, hardware_interface.get());
    // start the control loop
    boxbot_hw::BoxbotControlLoop control_loop(nh, hardware_interface);
    control_loop.run();

    return 0;

}
    
    

    
    

