#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "boxbot_autonomous/boxbot_planner.h"


BoxbotPlanner::BoxbotPlanner(const std::string &planGroup) :
    moveGroup(planGroup)
{
   moveGroup.setPlannerId("RRTConnect");
   moveGroup.setGoalTolerance(0.01);
   const moveit::core::JointModelGroup* joint_model_group = 
        moveGroup.getCurrentState()->getJointModelGroup(planGroup);
}

bool BoxbotPlanner::setTarget(double x, double y, double z) {
    targetPosition.x = x;
    targetPosition.y = y;
    targetPosition.z = z;
    ROS_INFO("BoxbotPlanner: target position set: (%lf, %lf, %lf)", x, y, z);
    return true;  
}

bool BoxbotPlanner::plan() {
    moveGroup.setPositionTarget(targetPosition.x, 
                                targetPosition.y, 
                                targetPosition.z);
    return (moveGroup.plan(motionPlan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS); 
    
}

bool BoxbotPlanner::execute() {
    return (moveGroup.execute(motionPlan) == 
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
}


bool BoxbotPlanner::randomValid() {
    geometry_msgs::Pose ranPose = moveGroup.getRandomPose().pose;
    return BoxbotPlanner::setTarget(ranPose.position.x,
                                ranPose.position.y,
                                ranPose.position.z);
}

void BoxbotPlanner::getTargetPosition(geometry_msgs::Point &target) {
    target.x = targetPosition.x;
    target.y = targetPosition.y;
    target.z = targetPosition.z;
}

void BoxbotPlanner::getCurrentPose(geometry_msgs::Quaternion &quaternion) {
    geometry_msgs::Pose pose = moveGroup.getCurrentPose().pose;
    quaternion.x = pose.position.x;
    quaternion.y = pose.position.y;
    quaternion.z = pose.position.z;
}


void BoxbotPlanner::stop() {
    moveGroup.stop();
}