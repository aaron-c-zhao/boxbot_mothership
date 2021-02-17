#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "boxbot_hw.h"
#include "boxbot_stepper_control/JointControl.h"
#include <numeric>


namespace boxbot_hw{

BoxbotHW::BoxbotHW(const ros::NodeHandle &nh, urdf::Model* urdf_model) 
    :nh_(nh), name_("hardware_interface") {
    if (urdf_model == NULL) 
        loadURDF(nh, "robot_description");
    else
        urdf_model_ = urdf_model;

    // load the rosparameters 
    ros::NodeHandle rpnh (
        nh_, name_); 
    
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_); 
    rosparam_shortcuts::shutdownIfError(name_, error);
    joint_nums_ = joint_names_.size();
    joint_mechanical_reduction_.resize(joint_nums_, 0.0);

    error += !rosparam_shortcuts::get(name_, rpnh, "mechanical_reduction", joint_mechanical_reduction_);
    error += !rosparam_shortcuts::get(name_, rpnh, "joint_diameter", joint_diameters_);
    error += !rosparam_shortcuts::get(name_, rpnh, "steps", steps_);
    error += !rosparam_shortcuts::get(name_, rpnh, "home", home_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    
}

void BoxbotHW::init() {
        pub_ = nh_.advertise<boxbot_stepper_control::JointControl>("/joint_control", 10);

        // initialize the control arraies
        cmd.resize(joint_nums_, 0.0);
        vel.resize(joint_nums_, 0.0);
        pos.resize(joint_nums_, 0.0);
        eff.resize(joint_nums_, 0.0);
        servo_pos_ = PI / 2.0;
        servo_world_ = 0;

        for (std::size_t joint_id = 0; joint_id < joint_nums_; joint_id++) {
            ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);
            // carete joint state interface
            jnt_state_interface.registerHandle(hardware_interface::JointStateHandle (
                joint_names_[joint_id], &pos[joint_id], &vel[joint_id], &eff[joint_id]));
            
            // create velocity command interface
            hardware_interface::JointHandle pos_handle = hardware_interface::JointHandle (
                jnt_state_interface.getHandle(joint_names_[joint_id]), &cmd[joint_id]);
            jnt_pos_interface.registerHandle(pos_handle);
            registerJointLimits(pos_handle, joint_id);
        }

        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);
}


void BoxbotHW::write(const ros::Time&, const ros::Duration& period) {

    enforceLimits(period);

    boxbot_stepper_control::JointControl control;
    for (std::size_t i = 0; i < joint_nums_; i++) {
        boxbot_stepper_control::Joint baseControl;
        baseControl.motor_id = i;
        baseControl.position = (cmd[i] * joint_mechanical_reduction_[i] * steps_[i]/ (joint_diameters_[i] * PI)); // joint->actuator 
        baseControl.disable = false;
        control.ctr.push_back(baseControl);
    }

    boxbot_stepper_control::Joint servoControl;
    servoControl.motor_id = 3;
    servoControl.position = servo_pos_; 
    servoControl.disable = false;
    control.ctr.push_back(servoControl);
    pub_.publish(control);
}

void BoxbotHW::read(const ros::Time&, const ros::Duration &period) {
    // read the running distance from the encoders
    // TODO: replace make up value by actual values from serial
    for (int i = 0; i < joint_nums_; i++) {
        vel[i] = (cmd[i] - pos[i]) / period.toSec();
        pos[i] = cmd[i];
        ROS_DEBUG_STREAM("[HW Read] " << joint_names_[i] << pos[i]);
    }

    servo_world_ += pos[0] - prev_pos_; 
    prev_pos_ = pos[0];

}


void BoxbotHW::loadURDF (const ros::NodeHandle& nh, std::string param_name) { 
    std::string urdf_string;
    urdf_model_ = new urdf::Model();

    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok())
    {
        std::string search_param_name;

        if (nh.searchParam(param_name, search_param_name))
        {
            ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << search_param_name); nh.getParam(search_param_name, urdf_string);
        }
        else
        {
            ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << param_name); nh.getParam(param_name, urdf_string);
        }

        usleep(100000);
    }

    if (!urdf_model_->initString(urdf_string))
        ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    else
        ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
}

 void BoxbotHW::registerJointLimits( const hardware_interface::JointHandle& joint_handle_position,
        std::size_t joint_id) 
{

    // Limits datastructures
    joint_limits_interface::JointLimits joint_limits;     
    bool has_joint_limits = false;

    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
    {
        has_joint_limits = true;
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                << joint_limits.min_position << ", " << joint_limits.max_position << "]");
        if (joint_limits.has_velocity_limits)
            ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                    << joint_limits.max_velocity << "]");
    }
    else
    {
        if (urdf_joint->type != urdf::Joint::CONTINUOUS)
            ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                    << " does not have a URDF "
                                                    "position limit");
    }

    if (!has_joint_limits) {
        return;
    }

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);
}


void BoxbotHW::enforceLimits(const ros::Duration& period) {
    pos_jnt_sat_interface_.enforceLimits(period);
};

bool BoxbotHW::home(boxbot_hardware::Home::Request &req,
    boxbot_hardware::Home::Response &res) {
    // pos[0] = req.pos.x;
    // pos[1] = req.pos.y;
    // pos[2] = req.pos.z;
    cmd[0] = req.pos.x;
    cmd[1] = req.pos.y;
    cmd[2] = req.pos.z;    
    std::cout << req.pos.x << req.pos.y << req.pos.z << std::endl;
    servo_pos_ = PI / 2.0;
    servo_world_ = (-5.0 * PI) / 180.0;
    prev_pos_ = 0.0;
    return true;
}

bool BoxbotHW::rotate(boxbot_hardware::Rotate::Request &req,
        boxbot_hardware::Rotate::Response &res) {
    if (req.querry) {
        res.servoPos = servo_world_;
        return true; 
    }

    servo_world_ = req.position;
    servo_pos_ = PI / 2.0 - req.position + pos[0] - (5.0 * PI) / 180.0;

    if (servo_pos_ < 0) {
        servo_world_ -= 0.0 - servo_pos_;
        servo_pos_ = 0;
    }

    if (servo_pos_ > PI) {
        servo_world_ += servo_pos_ - PI;
        servo_pos_ = PI;
    }

    return true;
}



} // namespace
