#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

#include "boxbot_autonomous/ControlCmd.h"
#include "boxbot_autonomous/boxbot_planner.h"
#include "boxbot_hardware/Home.h"
#include "boxbot_hardware/Rotate.h"
#include "boxbot_autonomous/Odrive.h"
#include "controller_manager_msgs/SwitchController.h"
#include <unistd.h>

typedef enum class error{
   SUCCESS,
   SETTARGET_FAIL,
   PLAN_FAIL,
   EXECUTION_FAIL,
   HOME_SWITCH_CONTROLLER_FAIL,
   FETCH_LOACTION_FAIL,
   UNKNOWN 
} errorCode;


#define CMD_MOVE            0 
#define CMD_STOP            1
#define CMD_HOME            2
#define CMD_GET_LOCATION    3
#define CMD_ROTATE          4
#define CMD_SET_POS         5



ros::ServiceClient client_hardware_;
ros::ServiceClient client_controller_;
ros::ServiceClient client_rotate_;
ros::Publisher odrive_pub_;
std::shared_ptr<BoxbotPlanner> planner_;

const std::string CONTROLLER = "arm_controller";

bool move(boxbot_autonomous::ControlCmd::Request &req, 
    boxbot_autonomous::ControlCmd::Response &res) {

    geometry_msgs::Quaternion pose = req.goal;

    if (!planner_->setTarget(pose.x, pose.y, pose.z)) {
        res.error_code = static_cast<int>(errorCode::SETTARGET_FAIL);
        return true;
    }

    if (!planner_->plan()) {
        res.error_code = static_cast<int>(errorCode::PLAN_FAIL);
        return true;
    }

    if (!planner_->execute()) {
        res.error_code = static_cast<int>(errorCode::EXECUTION_FAIL);
        return true;
    }
    res.error_code = static_cast<int>(errorCode::SUCCESS);

    return true;
}

bool stop(boxbot_autonomous::ControlCmd::Request &req, 
    boxbot_autonomous::ControlCmd::Response &res) {
    planner_->stop();
    res.error_code = static_cast<int>(errorCode::SUCCESS);
    ROS_INFO("BoxbotAutonmous: stopped movement");
    return true;
}


bool home(boxbot_autonomous::ControlCmd::Request &req, 
    boxbot_autonomous::ControlCmd::Response &res) {
    // switch off the controller to prevent it from holding the place
    controller_manager_msgs::SwitchController switchSrv;
    switchSrv.request.stop_controllers.push_back(CONTROLLER);
    switchSrv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    if (!client_controller_.call(switchSrv)) {
        res.error_code = static_cast<int>(errorCode::HOME_SWITCH_CONTROLLER_FAIL);
        return true;
    }

    boxbot_hardware::Home srv;
    if (req.cmd == req.HOME) {
        // thses positions are in joint space!!!
        srv.request.pos.x = 0.0;
        srv.request.pos.y = 0.0;
        srv.request.pos.z = 1.35;
        srv.request.pos.w = 1.57079632679; 
    }
    else {
        srv.request.pos.x = req.goal.x;
        srv.request.pos.y = req.goal.y;
        srv.request.pos.z = req.goal.z;
        srv.request.pos.w = req.goal.w;
    }

    client_hardware_.call(srv);
    sleep(0.5);

    // switch on the controller
    switchSrv.request.stop_controllers.clear();
    switchSrv.request.start_controllers.push_back(CONTROLLER);
    if (!client_controller_.call(switchSrv)) {
        res.error_code = static_cast<int>(errorCode::HOME_SWITCH_CONTROLLER_FAIL) ;
    }

    res.error_code = static_cast<int>(errorCode::SUCCESS);
    ROS_INFO("BoxbotAutonmous: homed");
    return true;
}

bool getLocation(boxbot_autonomous::ControlCmd::Request &req, 
    boxbot_autonomous::ControlCmd::Response &res) {
        geometry_msgs::Quaternion position;
        planner_->getCurrentPose(position);
        res.location.x = position.x;
        res.location.y = position.y;
        res.location.z = position.z;
        boxbot_hardware::Rotate srv;
        srv.request.querry = true;
        if (!client_rotate_.call(srv)) {
            res.error_code = static_cast<int>(errorCode::FETCH_LOACTION_FAIL);
            return true;
        }
        
        res.location.w = srv.response.servoPos;

        res.error_code = static_cast<int>(errorCode::SUCCESS);
        return true;
}

bool rotate(boxbot_autonomous::ControlCmd::Request &req,
    boxbot_autonomous::ControlCmd::Response &res) {
    boxbot_hardware::Rotate srv;
    srv.request.querry = false;
    srv.request.position = req.goal.w;
    client_rotate_.call(srv);

    res.error_code = static_cast<int>(errorCode::SUCCESS);
    ROS_INFO("BoxbotAutonmous: rotation to %lf", req.goal.w);
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "boxbot_control");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    BoxbotPlanner planner("arm");
    std::shared_ptr<BoxbotPlanner> ptr(&planner);
    planner_ = ptr;

    client_hardware_ = nh.serviceClient<boxbot_hardware::Home>("boxbot/boxbot_home");
    client_controller_ = nh.serviceClient<controller_manager_msgs::SwitchController>("boxbot/controller_manager/switch_controller");
    client_rotate_ = nh.serviceClient<boxbot_hardware::Rotate>("boxbot/boxbot_rotate");
    odrive_pub_ = nh.advertise<boxbot_autonomous::Odrive>("boxbot_odrive", 1);
    
    ros::ServiceServer moveServer = nh.advertiseService("rcs_control/move", move);
    ros::ServiceServer stopServer = nh.advertiseService("rcs_control/stop", stop);
    ros::ServiceServer homeServer = nh.advertiseService("rcs_control/home", home);
    ros::ServiceServer getLocationServer = nh.advertiseService("rcs_control/get_location", getLocation);
    ros::ServiceServer rotateServer = nh.advertiseService("rcs_control/rotate", rotate);
    ros::ServiceServer setPosServer = nh.advertiseService("rcs_control/set_pos", home);

    ros::waitForShutdown();
}


