#ifndef BOXBOT_CONTROL_LOOP_H_
#define BOXBOT_CONTROL_LOOP_H_


#include <time.h>
#include <controller_manager/controller_manager.h>

#include "boxbot_hw.h"

namespace boxbot_hw {

static const double BILLION = 1000000000.0;

class BoxbotControlLoop {

public:    

    /// \brief Constructor
    ///
    /// \param nh node handler
    /// \param hardware_interface boxbot's hardware interface
    BoxbotControlLoop(ros::NodeHandle& nh, 
        std::shared_ptr<hardware_interface::RobotHW> hardware_interface);
    
    /// \brief run the control loop
    void run();

private:
   
    // function called the control loop 
    void update();

        
    ros::NodeHandle nh_;

    std::string name_ = "boxbot_control_loop";

    ros::Duration update_period_;

    double cycle_time_error_threshold_;


    // Timing
    ros::Duration elapsed_time_;
    double loop_hz_;
    struct timespec last_time_;
    struct timespec current_time_;
 
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    std::shared_ptr<hardware_interface::RobotHW> hardware_interface_;
}; // class
} // namespace

#endif
