#ifndef _BOXBOT_H
#define _BOXBOT_H

#include <ros/ros.h>
#include <urdf/model.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "boxbot_hardware/Home.h"
#include "boxbot_hardware/Rotate.h"

//#define BOXBOT_HW_DEBUG_


namespace boxbot_hw{

class BoxbotHW : public hardware_interface::RobotHW {
    public:
        /// Constructor
        BoxbotHW(const ros::NodeHandle &nh, urdf::Model* urdf_model=NULL);
        /// \brief intialize the interface
        void init();
        /// \brief write velocity command to actuators
        ///
        /// This method talks to the actuators via serial protocol. 
        void write(const ros::Time&, const ros::Duration &period);
        void read(const ros::Time&, const ros::Duration &period);
        void registerJointLimits(const hardware_interface::JointHandle& joint_handle_effort, std::size_t joint_id);
        void enforceLimits(const ros::Duration& period);
        bool home(boxbot_hardware::Home::Request &req,
            boxbot_hardware::Home::Response &res);
        bool rotate(boxbot_hardware::Rotate::Request &req, 
            boxbot_hardware::Rotate::Response &res);

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;

        // Joint limits interfaces - Saturation
        joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;

        // data structures that hold the command and the reading from 
        // the actuators
        std::vector<double> cmd; ///< command space
        std::vector<double> vel; ///< joint state: velocity
        std::vector<double> pos; ///< joint state: position
        std::vector<double> eff; ///< joint state: effort


        urdf::Model* urdf_model_; ///< Configuration of the robot
        

        ros::NodeHandle nh_; ///< inertial ros node
        ros::Publisher pub_;
        
        /// \brief load the urdf model from the parameter sever
        /// \param nh node handler
        /// \param parameter name for the urdf model
        void loadURDF(const ros::NodeHandle &nh, std::string param_name); 

        std::vector<std::string> joint_names_; ///< loaded joint names
        unsigned char joint_nums_;

        const std::string name_;  ///< name of the config file

        // Copy of limits, in case we need them later in our control stack
        std::vector<double> joint_mechanical_reduction_;
        std::vector<double> joint_diameters_;
        std::vector<double> steps_;
        std::vector<double> home_;

        double servo_pos_;
        double servo_world_;
        double prev_pos_ = 0.0;

        bool homing_ = false;

 

        const double PI =  3.14159265359;
}; // class

} // namespacw

#endif