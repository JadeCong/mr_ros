#include <mr_tasks/massage_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mr_tasks_demo {

constexpr char LOGNAME[] = "mr_task";
constexpr char MassageTask::LOGNAME[];

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

MassageTask::MassageTask(const std::string& task_name, const ros::NodeHandle& pnh) : pnh_(pnh), task_name_(task_name) {
    loadParameters();
}

void MassageTask::loadParameters() {
    // Load parameters
    ROS_INFO_NAMED(LOGNAME, "Loading parameters for task %s", task_name_.c_str());
    
    // Planning group properties
    size_t errors = 0;
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name", arm_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_root_frame", arm_root_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_tip_frame", arm_tip_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "ee_group_name", ee_group_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "ee_root_frame", ee_root_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "ee_tip_frame", ee_tip_frame_);
    
    // Scene frame and model properties
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_name", human_name_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_mesh", human_mesh_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_reference_frame", human_reference_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_collision_frame", human_collision_frame_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_dimensions", human_dimensions_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "human_pose", human_pose_);
    
    // Predefined pose targets for massaging
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "massage_meta_skill", massage_meta_skill_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "massage_ready_pose", massage_ready_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "massage_contact_pose", massage_contact_pose_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "massage_safe_pose", massage_safe_pose_);
    
    // Massage metrics
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pose_target_min_dist", pose_target_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "pose_target_max_dist", pose_target_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "contact_human_min_dist", contact_human_min_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "contact_human_max_dist", contact_human_max_dist_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "massage_metric_pose", massage_metric_pose_);
    
    rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool MassageTask::init() {
    ROS_INFO_NAMED(LOGNAME, "Initializing task %s pipeline", task_name_.c_str());
    const std::string object = human_name_;
    
    // Reset ROS introspection before constructing the new object
    task_.reset();
    task_.reset(new moveit::task_constructor::Task());
    
    Task& t = *task_;
    t.stages()->setName(task_name_);
    t.loadRobotModel();
    
    // Sampling planner
    auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    
    // Cartesian planner
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScaling(1.0);
    cartesian_planner->setMaxAccelerationScaling(1.0);
    cartesian_planner->setStepSize(0.01);
    
    // Joint-Space planner
    auto joint_planner = std::make_shared<solvers::JointInterpolationPlanner>();
    
    // Set task properties
    t.setProperty("arm", arm_group_name_);
    t.setProperty("arm_root", arm_root_frame_);
    t.setProperty("arm_tip", arm_tip_frame_);
    t.setProperty("ee", ee_group_name_);
    t.setProperty("ee_root", ee_root_frame_);
    t.setProperty("ee_tip", ee_tip_frame_);
    
    /****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
    auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
    Stage* current_state_ptr = nullptr;
    {  // Forward current_state on to ready pose above acupoint
        auto current_state = std::make_unique<stages::CurrentState>("current state");
        
        // Verify that human is not in collision
        auto applicability_filter = std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
        applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
            if (s.start()->scene()->isStateColliding(object)) {
                comment = "Human with id '" + object + "' is in collision with mr arm";
                return false;
            }
            return true;
        });
        
        current_state_ptr = applicability_filter.get();
        t.add(std::move(applicability_filter));
    }
    
    /****************************************************
     *                                                  *
     *                Move to Ready Pose                *
     *                                                  *
     ***************************************************/
    {  // move to ready pose above acupoint
        auto stage = std::make_unique<stages::Connect>(
            "move to ready pose", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        
        t.add(std::move(stage));
    }
    
    /****************************************************
	 *                                                  *
	 *                 Massage Start                    *
	 *                                                  *
	 ***************************************************/
    {
        auto massage_start = std::make_unique<SerialContainer>("massage start");
        t.properties().exposeTo(massage_start->properties(), { "arm", "arm_tip", "ee", "ee_tip"});
        massage_start->properties().configureInitFrom(Stage::PARENT, { "arm", "arm_tip", "ee", "ee_tip" });
        
        /****************************************************
         *          Allow Collision (Palm/Human)            *
         ***************************************************/
        {  // allow collision between palm and human
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (palm/human)");
            stage->allowCollisions(object, t.getRobotModel()->getJointModelGroup(ee_group_name_)->getLinkModelNamesWithCollisionGeometry(), true);
            
            massage_start->insert(std::move(stage));
        }
        
        /****************************************************
         *                 Move to Acupoint                 *
         ***************************************************/
        {  // move to acupoint
            auto stage = std::make_unique<stages::MoveTo>("move to acupoint", sampling_planner);
            stage->setGroup(ee_group_name_);
            
            // Set acupoint pose
            geometry_msgs::PoseStamped acupoint_pose;
            acupoint_pose.header.frame_id = ee_root_frame_;
            acupoint_pose.pose = massage_contact_pose_;
            stage->setGoal(acupoint_pose);
            
            massage_start->insert(std::move(stage));
        }
        
        // Add massage_start to task
        t.add(std::move(massage_start));
    }
    
    /****************************************************
	 *                                                  *
	 *               Massage On-going                   *
	 *                                                  *
	 ***************************************************/
    {
        auto massage_ongoing = std::make_unique<SerialContainer>("massage on-going");
        t.properties().exposeTo(massage_ongoing->properties(), { "arm", "arm_tip", "ee", "ee_tip" });
        massage_ongoing->properties().configureInitFrom(Stage::PARENT, { "arm", "arm_tip", "ee", "ee_tip" });
        
        /****************************************************
         *           Repeat Massage-Meta-Skill              *
         ***************************************************/
        {  // repeat the massage-meta-skill with specified frequency(0.25Hz) for 16 seconds
            auto stage = std::make_unique<stages::MoveRelative>("repeat massage-meta-skill", cartesian_planner);
            stage->setGroup(ee_group_name_);
            stage->properties().set("marker_ns", "repeat_massage_meta_skill");
            stage->properties().set("link", ee_tip_frame_);
            stage->properties().configureInitFrom(Stage::PARENT);
            
            // TODO: set the massage_twist direction
            geometry_msgs::TwistStamped massage_twist;
            massage_twist.header.frame_id = ee_group_name_;
            massage_twist.twist.angular.z = M_PI; 
            stage->setDirection(massage_twist);
            
            massage_ongoing->insert(std::move(stage));
        }
        
        // Add massage_ongoing to task
        t.add(std::move(massage_ongoing));
    }
    
    /****************************************************
	 *                                                  *
	 *                   Massage End                    *
	 *                                                  *
	 ***************************************************/
    {
        auto massage_end = std::make_unique<SerialContainer>("massage end");
        t.properties().exposeTo(massage_end->properties(), { "arm", "arm_tip", "ee", "ee_tip" });
        massage_end->properties().configureInitFrom(Stage::PARENT, { "arm", "arm_tip", "ee", "ee_tip" });
        
        /****************************************************
         *                 Move to Safe Pose                *
         ***************************************************/
        {  // move to safe pose above acupoint
            auto stage = std::make_unique<stages::MoveRelative>("move to safe pose", cartesian_planner);
            stage->setGroup(ee_group_name_);
            stage->properties().set("marker_ns", "move_to_safe_pose");
            stage->properties().set("link", ee_tip_frame_);
            stage->properties().configureInitFrom(Stage::PARENT, { "arm", "ee" });
            stage->setMinMaxDistance(pose_target_min_dist_, pose_target_max_dist_);
            
            // Set safe-up direction
            geometry_msgs::Vector3Stamped up_vec;
            up_vec.header.frame_id = ee_group_name_;
            up_vec.vector.z = -0.2;
            stage->setDirection(up_vec);
            
            massage_end->insert(std::move(stage));
        }
        
        /****************************************************
         *          Forbid Collision (Palm/Human)           *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (palm/human)");
            stage->allowCollisions(object, t.getRobotModel()->getJointModelGroup(ee_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
            
            massage_end->insert(std::move(stage));
        }
        
        /****************************************************
         *          Retreat Motion for connecting           *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat motion for connecting", cartesian_planner);
            stage->setGroup(ee_group_name_);
            stage->properties().set("marker_ns", "retreat_motion_for_connecting");
            stage->setIKFrame(ee_tip_frame_);
            stage->setMinMaxDistance(pose_target_min_dist_, pose_target_max_dist_);
            stage->properties().configureInitFrom(Stage::PARENT, { "arm", "arm_tip", "ee", "ee_tip" });
            
            // Set retreat direction
            geometry_msgs::Vector3Stamped retreat_vec;
			retreat_vec.header.frame_id = ee_group_name_;
			retreat_vec.vector.z = -0.005;
			stage->setDirection(retreat_vec);
            
            massage_end->insert(std::move(stage));
        }
        
        // Add massage_end to task
        t.add(std::move(massage_end));
    }
    
    /****************************************************
     *                                                  *
     *                     Move to Home                 *
     *                                                  *
     ***************************************************/
    {  // move to home
        auto stage = std::make_unique<stages::MoveTo>("move to home", sampling_planner);
        stage->setGroup(arm_group_name_);
        stage->setGoal(arm_home_pose_);
        stage->properties().configureInitFrom(Stage::PARENT, { "arm", "ee" });
        // stage->restrictDirection(stages::MoveTo::FORWARD);
        
        t.add(std::move(stage));
    }
    
    {
        auto fixed = std::make_unique<stages::FixedState>("final state");
        fixed->setState(scene);
        t.add(std::move(fixed));
    }
    
    // Prepare Task structure for planning
    try {
        t.init();
    } catch (InitStageException& e) {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to initialize task: " << e);
        return false;
    }
    
    return true;
}

bool MassageTask::plan() {
    ROS_INFO_NAMED(LOGNAME, "Start searching solutions for task %s", task_name_.c_str());
    int max_solutions = pnh_.param<int>("max_solutions", 10);
    
    return task_->plan(max_solutions);
}

bool MassageTask::execute() {
    ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory for task %s", task_name_.c_str());
    moveit_msgs::MoveItErrorCodes execute_result;
    
    execute_result = task_->execute(*task_->solutions().front());
    
    if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR_NAMED(LOGNAME, "Failed to execute solution trajectory for task %s", task_name_.c_str());
        return false;
    }
    
    return true;
}

}  // namespace mr_tasks_demo
