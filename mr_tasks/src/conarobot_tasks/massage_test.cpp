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

// Configure Parameters
#include <rosparam_shortcuts/rosparam_shortcuts.h>


using namespace moveit::task_constructor;

constexpr char LOGNAME[] = "massage_test";

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
    if (!psi.applyCollisionObject(object))
        throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject loadHuman(ros::NodeHandle& pnh) {
    std::string human_file, human_name, human_reference_frame, human_collision_frame;
    std::vector<double> human_dimensions;
    geometry_msgs::Pose human_pose;
    std::size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_mesh", human_file);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_name", human_name);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_reference_frame", human_reference_frame);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_collision_frame", human_collision_frame);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_dimensions", human_dimensions);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "human_pose", human_pose);
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
	
    moveit_msgs::CollisionObject human;
    human.id = human_name;
    human.header.frame_id = human_reference_frame;
    
    shapes::Mesh* mesh = shapes::createMeshFromResource(human_file);
    shapes::ShapeMsg human_mesh_msg;
    shape_msgs::Mesh human_mesh;
    shapes::constructMsgFromShape(mesh, human_mesh_msg);
    human_mesh = boost::get<shape_msgs::Mesh>(human_mesh_msg);
    
    human.meshes.push_back(human_mesh);
    human.mesh_poses.push_back(human_pose);
    human.operation = moveit_msgs::CollisionObject::ADD;
    ROS_INFO_NAMED(LOGNAME, "Adding a human mesh into world scene");
    
    return human;
}

void setupDemoScene(ros::NodeHandle& pnh) {
    // Load human to planning scene
    ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service to be available
    moveit::planning_interface::PlanningSceneInterface psi;
    if (pnh.param("spawn_human", true))
        spawnObject(psi, loadHuman(pnh));
}

Task createTask() {
	Task t;
	t.stages()->setName("Cartesian Path");
	
	const std::string group = "mr";
	const std::string ee = "palm";
	
	// create Cartesian interpolation "planner" to be used in various stages
	auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
	// create a joint-space interpolation "planner" to be used in various stages
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();
	
	// start from a fixed robot state
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "Ready");
		
		auto fixed = std::make_unique<stages::FixedState>("initial state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z +0.5", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 0.5;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.05", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.05;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z -0.4", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -0.4;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.04", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.04;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z +0.5", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 0.5;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.03", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.03;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z -0.5", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -0.5;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.02", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.02;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z +0.5", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 0.5;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.005", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.005;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::MoveRelative>("z -0.5", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -0.5;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}
	
	// {  // rotate about TCP
	// 	auto stage = std::make_unique<stages::MoveRelative>("ry +45°", cartesian_interpolation);
	// 	stage->setGroup(ee);
	// 	geometry_msgs::TwistStamped twist;
	// 	twist.header.frame_id = "world";
	// 	twist.twist.angular.y = M_PI / 4.;
	// 	stage->setDirection(twist);
	// 	t.add(std::move(stage));
	// }
	
	// {  // perform a Cartesian motion, defined as a relative offset in joint space
	// 	auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian_interpolation);
	// 	stage->setGroup(group);
	// 	std::map<std::string, double> offsets = { { "panda_joint1", M_PI / 6. }, { "panda_joint3", -M_PI / 6 } };
	// 	stage->setDirection(offsets);
	// 	t.add(std::move(stage));
	// }
	
	// {  // move gripper into predefined open state
	// 	auto stage = std::make_unique<stages::MoveTo>("open gripper", joint_interpolation);
	// 	stage->setGroup(ee);
	// 	stage->setGoal("open");
	// 	t.add(std::move(stage));
	// }
	
	{  // move from reached state back to the original state, using joint interpolation
		// specifying two groups (arm and hand) will try to merge both trajectories
		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation }, { ee, joint_interpolation } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		t.add(std::move(connect));
	}
	
	{  // final state is original state again
		auto fixed = std::make_unique<stages::FixedState>("final state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}
	
	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mr_massage_test");
	ros::NodeHandle pnh("~");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// add the human into mr scene
	setupDemoScene(pnh);
	
	auto task = createTask();
	try {
		if (task.plan())
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}
	
	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
