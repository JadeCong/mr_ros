// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <eigen_conversions/eigen_msg.h>

// Mesh
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#pragma once

namespace mr_tasks_demo {
using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under pnh
void setupDemoScene(ros::NodeHandle& pnh);

class MassageTask
{
public:
    MassageTask(const std::string& task_name, const ros::NodeHandle& pnh);
    ~MassageTask() = default;
    
    bool init();
    
    bool plan();
    
    bool execute();

private:
    void loadParameters();
    
    static constexpr char LOGNAME[]{ "massage_task" };
    
    ros::NodeHandle pnh_;
    
    std::string task_name_;
    moveit::task_constructor::TaskPtr task_;
    
    // Planning group properties
    std::string arm_group_name_;
    std::string arm_root_frame_;
    std::string arm_tip_frame_;
    std::string ee_group_name_;
    std::string ee_root_frame_;
    std::string ee_tip_frame_;
    
    // Scene frame and model properties
    std::string world_frame_;
    std::string human_name_;
    std::string human_mesh_;
    std::string human_reference_frame_;
    std::string human_collision_frame_;
    std::vector<double> human_dimensions_;
    std::vector<double> human_pose_;
    
    // Predefined pose targets for massaging
    std::string arm_home_pose_;
    std::string massage_meta_skill_;
    geometry_msgs::Pose massage_ready_pose_;
    geometry_msgs::Pose massage_contact_pose_;
    geometry_msgs::Pose massage_safe_pose_;
    
    // Massage metrics
    double pose_target_min_dist_;
    double pose_target_max_dist_;
    double contact_human_min_dist_;
    double contact_human_max_dist_;
    geometry_msgs::Pose massage_metric_pose_;
};
}  // namespace mr_tasks_demo
