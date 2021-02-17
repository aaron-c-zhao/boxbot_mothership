#ifndef _BOXBOT_PLANNER
#define _BOXBOT_PLANNER 

#include <moveit/move_group_interface/move_group_interface.h>

/// \brief A boxbot wrapper class for the MoveGroupInterface. 
/// 
/// This class hide the unnecessary complexities from the ourside with intention
/// to provide an API to the FSM that controls the robot.
class BoxbotPlanner {

public: 
    /// \brief Constructor of the class. 
    /// \param planGroup The name of the plan group that catering for. 
    BoxbotPlanner(const std::string &planGroup);

    /// \brief Set the target position of the robot. 
    /// 
    /// Set the target position for the motion planner. The orientation is hidded 
    /// from the outside as it doesn't matter for a 3DOF(4DOF) robot. This method 
    /// will check against a flag isTargetSet, and it will only set the new target
    /// when this flag is false which means the previous target has done execution.
    /// 
    /// \param x The x coordinate of the target position. Positive direction is 
    ///         pointing outside the screen.
    /// \param y The y coordiante of the target position. Positive direction is 
    ///         pointing to the right hand side.
    /// \param z The z coordinate of the target position. Positive direction is 
    ///         pointing upwards.
    /// \return A boolean value that indicates whether the target is set. It returns
    /// flase too when the isTargetSet flag is true.
    bool setTarget(double x, double y, double z);

    /// \brief Generate the motion plan. 
    ///
    /// This method calls the corrsponding Moveit function. But before, it will
    /// check the flag isTarget set. So only when there's a new target, a motion
    /// plan could be generated.
    ///
    /// \return A boolean value that indicates whether the motion plan is generated
    ///         successfully.
    bool plan();

    /// \brief Execute the motion plan.
    ///
    /// Execute the motion plan and toggle the isTargetSet flag.
    ///
    /// \return A boolean value that indicates whether the executino is success.
    bool execute();

    /// \brief Set a random target position for the robot. 
    /// 
    /// This method is for debugging purpose.
    /// 
    /// \return A boolean value that indicates whether the target is set. It returns
    /// flase too when the isTargetSet flag is true.
    bool randomValid();

    /// \brief Retrive the current target positoin.
    /// 
    /// \param target a pointer to where the target will be stored.
    void getTargetPosition(geometry_msgs::Point &target);
    void getCurrentPose(geometry_msgs::Quaternion &quaternion);

    void stop();


private:
    moveit::planning_interface::MoveGroupInterface moveGroup; 
    geometry_msgs::Point targetPosition;
    moveit::planning_interface::MoveGroupInterface::Plan motionPlan;




}; // BoxbotPlanner


#endif 


