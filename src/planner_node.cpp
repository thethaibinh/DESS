#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : to_world_tf2(to_world_buffer),
    to_camera_tf2(to_camera_buffer),
    camera_frame("camera"),
    world_frame("world"),
    steering_value(0.0),
    sampling_mode(0),
    first_time_in_new_state_(true),
    trajectory_queue_(),
    autopilot_state_(States::OFF) {
  // Publishers
  control_command_pub_ =
    n_.advertise<dodgeros_msgs::Command>("/kingfisher/dodgeros_pilot/feedthrough_command", 1);
  trajectoty_pub = n_.advertise<geometry_msgs::Pose>("/trajectory", 1);
  point_cloud_pub = n_.advertise<sm::PointCloud2>("/cloud_out", 10);
  visual_pub = n_.advertise<visualization_msgs::Marker>("/visualization", 10);
  image_sub = n_.subscribe("/kingfisher/dodgeros_pilot/unity/depth", 1,
                           &PlannerNode::msgCallback, this);
  sampling_mode_sub = n_.subscribe("/sampling_mode", 1,
                                   &PlannerNode::sampling_mode_callback, this);
  start_sub = n_.subscribe("/kingfisher/start_navigation", 1,
                                   &PlannerNode::start_callback, this);
  reset_sub = n_.subscribe("/kingfisher/dodgeros_pilot/reset_sim", 1,
                             &PlannerNode::reset_callback, this);
  state_sub = n_.subscribe("/kingfisher/dodgeros_pilot/state", 1,
                             &PlannerNode::state_callback, this);
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

void PlannerNode::sampling_mode_callback(const std_msgs::Int8::ConstPtr& msg) {
  sampling_mode = msg->data;
}

void PlannerNode::start_callback(const std_msgs::Empty::ConstPtr& msg) {
  ROS_WARN("[%s] Planner: Start publishing commands!",
           pnh_.getNamespace().c_str());
  setAutoPilotStateForced(States::START);
}

void PlannerNode::reset_callback(const std_msgs::Empty::ConstPtr& msg) {
  ROS_WARN("[%s] Planner: Reset quadrotor simulator!",
           pnh_.getNamespace().c_str());
  setAutoPilotStateForced(States::OFF);
  steering_value = 0.0f;
}

void PlannerNode::state_callback(const dodgeros_msgs::QuadState& state) {

  const std::lock_guard<std::mutex> lock(state_mutex_);
  // assign new state
  _state = state;

  // checking if new trajectory generated
  // only consider the latest
  while (trajectory_queue_.size() > 1) {
    trajectory_queue_.pop_front();
  }
  ros::Time wall_time_now = ros::Time::now();
  ros::Duration trajectory_point_time =
    wall_time_now - time_start_trajectory_execution;
  double point_time = trajectory_point_time.toSec();
  if (trajectory_queue_.size()) {
    if (check_valid_trajectory(_state, trajectory_queue_.front()) &&
        // autopilot_state_ == States::START) {
        point_time > 1.2f) {
      reference_trajectory_ = trajectory_queue_.front();
      // ROS_WARN("[%s] Duration %f", pnh_.getNamespace().c_str(),
      //          reference_trajectory_.get_duration());
      // marking trajectory execution starting time
      time_start_trajectory_execution = ros::Time::now();
    }
    trajectory_queue_.pop_front();
  }

  // Update autopilot state
  geometry_msgs::Point goal_in_world_frame;
  goal_in_world_frame.x = goal_x_world_coordinate;
  goal_in_world_frame.y = goal_y_world_coordinate;
  goal_in_world_frame.z = goal_z_world_coordinate;

  double distance_to_goal =
    (quadrotor_common::geometryToEigen(_state.pose.position) -
     quadrotor_common::geometryToEigen(goal_in_world_frame))
      .norm();

  if (_state.pose.position.z > 4.8 && autopilot_state_ == States::START) {
    setAutoPilotStateForced(States::TRAJECTORY_CONTROL);
  }
  // Stop planning when the goal is less than 1.5m close.
  else if (autopilot_state_ == States::TRAJECTORY_CONTROL &&
           (distance_to_goal < 3 ||
            _state.pose.position.x > goal_x_world_coordinate)) {
    setAutoPilotStateForced(States::GO_TO_GOAL);
  }

  // tracking trajectory
  track_trajectory();
}

void PlannerNode::track_trajectory() {
  double control_command_delay_ = 0.0;
  ros::Time wall_time_now = ros::Time::now();
  ros::Time command_execution_time =
    wall_time_now + ros::Duration(control_command_delay_);

  quadrotor_common::TrajectoryPoint reference_point;
  quadrotor_common::ControlCommand command;

  if (autopilot_state_ == States::START) {
    last_feasible_planning_time = command_execution_time;
    steering_value = 0.0f;
    reference_point.position = Eigen::Vector3d(0.0, 0.0, 5.0);
  } else if (autopilot_state_ == States::TRAJECTORY_CONTROL) {
    ros::Duration trajectory_point_time =
      command_execution_time - time_start_trajectory_execution;
    double point_time = trajectory_point_time.toSec();
    double trajectory_duration = reference_trajectory_.get_duration();
    point_time =
      (point_time > trajectory_duration) ? trajectory_duration : point_time;

    get_reference_point_at_time(reference_trajectory_, point_time,
                                reference_point);
  } else if (autopilot_state_ == States::GO_TO_GOAL) {
    last_feasible_planning_time = command_execution_time;
    steering_value = 0.0f;
    reference_point.position =
      Eigen::Vector3d(goal_x_world_coordinate, goal_y_world_coordinate,
                      goal_z_world_coordinate);
  }

  command = track_trajectory_point(reference_point);

  if (autopilot_state_ != States::COMMAND_FEEDTHROUGH &&
      autopilot_state_ != States::OFF) {
    command.timestamp = wall_time_now;
    command.expected_execution_time = command_execution_time;
    command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
    publishControlCommand(command);
  }
}

void PlannerNode::get_reference_point_at_time(
  const ruckig::Trajectory<3>& reference_trajectory, const double& point_time,
  quadrotor_common::TrajectoryPoint& reference_point) {

  // Get the corresponding transform attached to the trajectory
  geometry_msgs::TransformStamped transform_to_world =
    reference_trajectory.get_transform_to_world();

  // Get pvaj in the body frame at a given time from the reference trajectory
  std::array<double, 3> position, velocity, acceleration, jerk;
  size_t num_section;
  reference_trajectory.at_time(point_time, position, velocity, acceleration, jerk,
          num_section);

  // Transform pvaj to the world frame
  geometry_msgs::Point position_in_body_frame, position_in_world_frame;
  geometry_msgs::Vector3 velocity_in_body_frame, velocity_in_world_frame,
    acceleration_in_body_frame, acceleration_in_world_frame, jerk_in_body_frame,
    jerk_in_world_frame;
  // Aligning Oxyz axis due to difference in frames
  position_in_body_frame.x = position[2];
  position_in_body_frame.y = -position[0];
  position_in_body_frame.z = -position[1];

  velocity_in_body_frame.x = velocity[2];
  velocity_in_body_frame.y = -velocity[0];
  velocity_in_body_frame.z = -velocity[1];
  acceleration_in_body_frame.x = acceleration[2];
  acceleration_in_body_frame.y = -acceleration[0];
  acceleration_in_body_frame.z = -acceleration[1];
  jerk_in_body_frame.x = jerk[2];
  jerk_in_body_frame.y = -jerk[0];
  jerk_in_body_frame.z = -jerk[1];
  try {
    tf2::doTransform(position_in_body_frame, position_in_world_frame,
                     transform_to_world);
    tf2::doTransform(velocity_in_body_frame, velocity_in_world_frame,
                     transform_to_world);
    tf2::doTransform(acceleration_in_body_frame, acceleration_in_world_frame,
                     transform_to_world);
    tf2::doTransform(jerk_in_body_frame, jerk_in_world_frame,
                     transform_to_world);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Failure %s\n", ex.what());  // Print exception which was
                                          // caught
  }

  // Asigning heading
  Eigen::Vector3d trajectory_vector =
    quadrotor_common::geometryToEigen(
      reference_trajectory.get_terminal_position_in_world_frame()) -
    quadrotor_common::geometryToEigen(
      reference_trajectory.get_initial_position_in_world_frame());
  double terminal_heading = atan2f(trajectory_vector[1], trajectory_vector[0]);
  reference_point.heading = terminal_heading;
  Eigen::Vector3d current_euler_angles =
    quadrotor_common::quaternionToEulerAnglesZYX(
      quadrotor_common::geometryToEigen(_state.pose.orientation));
  if (fabs(steering_value) > 1e-6)
    reference_point.heading = current_euler_angles(2) + steering_value;

  // Asigning to trajectory reference point
  reference_point.position =
    quadrotor_common::geometryToEigen(position_in_world_frame);
  reference_point.velocity =
    quadrotor_common::geometryToEigen(velocity_in_world_frame);
  reference_point.acceleration =
    quadrotor_common::geometryToEigen(acceleration_in_world_frame);
  reference_point.jerk = quadrotor_common::geometryToEigen(jerk_in_world_frame);
}

quadrotor_common::ControlCommand PlannerNode::track_trajectory_point(
  const quadrotor_common::TrajectoryPoint& reference_point) {
  quadrotor_common::QuadStateEstimate quad_state_ =
    quad_common_state_from_dodgedrone_state(_state);
  return base_controller_.run(quad_state_, reference_point,
                              base_controller_params_);
}

void PlannerNode::publishControlCommand(
  const quadrotor_common::ControlCommand& control_cmd) {
  if (control_cmd.control_mode == quadrotor_common::ControlMode::NONE) {
    ROS_ERROR("[%s] Control mode is NONE, will not publish ControlCommand",
              pnh_.getNamespace().c_str());
  } else {
    dodgeros_msgs::Command ros_command;
    ros_command.header.stamp = control_cmd.timestamp;
    ros_command.t = _state.t;
    ros_command.is_single_rotor_thrust = false;
    ros_command.collective_thrust = control_cmd.collective_thrust;
    ros_command.bodyrates.x = control_cmd.bodyrates.x();
    ros_command.bodyrates.y = control_cmd.bodyrates.y();
    ros_command.bodyrates.z = control_cmd.bodyrates.z();

    control_command_pub_.publish(ros_command);
  }
}

void PlannerNode::setAutoPilotStateForced(
    const States& new_state) {
  const ros::Time time_now = ros::Time::now();

  if (new_state != States::TRAJECTORY_CONTROL && !trajectory_queue_.empty()) {
    trajectory_queue_.clear();
  }
  time_of_switch_to_current_state_ = time_now;
  first_time_in_new_state_ = true;
  autopilot_state_ = new_state;

  std::string state_name;
  switch (autopilot_state_) {
    case States::OFF:
      state_name = "OFF";
      break;
    case States::START:
      state_name = "START";
      break;
    case States::HOVER:
      state_name = "HOVER";
      break;
    case States::LAND:
      state_name = "LAND";
      break;
    case States::EMERGENCY_LAND:
      state_name = "EMERGENCY_LAND";
      break;
    case States::BREAKING:
      state_name = "BREAKING";
      break;
    case States::GO_TO_POSE:
      state_name = "GO_TO_POSE";
      break;
    case States::VELOCITY_CONTROL:
      state_name = "VELOCITY_CONTROL";
      break;
    case States::REFERENCE_CONTROL:
      state_name = "REFERENCE_CONTROL";
      break;
    case States::TRAJECTORY_CONTROL:
      state_name = "TRAJECTORY_CONTROL";
      break;
    case States::COMMAND_FEEDTHROUGH:
      state_name = "COMMAND_FEEDTHROUGH";
      break;
    case States::RC_MANUAL:
      state_name = "RC_MANUAL";
      break;
  }
  ROS_INFO("[%s] Switched to %s state", pnh_.getNamespace().c_str(),
           state_name.c_str());
}

bool PlannerNode::check_valid_trajectory(
  const dodgeros_msgs::QuadState& _state,
  const ruckig::Trajectory<3>& trajectory) {
  double pos_diff = (quadrotor_common::geometryToEigen(_state.pose.position) -
                     quadrotor_common::geometryToEigen(
                       trajectory.get_initial_position_in_world_frame()))
                      .norm();
  if (pos_diff > kPositionJumpTolerance_) {
    ROS_WARN(
      "[%s] The received trajectory does not start at current "
      "position, will ignore trajectory",
      pnh_.getNamespace().c_str());
    return false;
  }
  return true;
}

// Callback for planning when a new depth image comes
void PlannerNode::msgCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
  cv_bridge::CvImageConstPtr cv_img_ptr =
    cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  cv::Mat depth_mat;
  // RAPPIDS use depth images in 16UC1 format
  cv_img_ptr->image.convertTo(depth_mat, CV_16UC1, 65535);

  double width = depth_mat.cols;
  double height = depth_mat.rows;
  double fov = 90.0f;
  double cx = width / 2.0f;
  double cy = height / 2.0f;
  // double fx = (width / 2.0f) / tan((M_PI * fov / 180.0f) / 2.0f);
  // double fy = (height / 2.0f) / tan((M_PI * fov / 180.0f) / 2.0f);
  // We use camera intrinsics matrix value from Flightmare
  // with FoV 90 degrees and resolution 320x240
  double fx, fy;
  fx = fy = 130.839769;

  // pixelValues = depth / depth_scale
  // e.g. 1 meter = 1000 pixel value for depth_scale = 0.001
  // depth_scale in 32FC1 is 100,
  // then in 16UC1, depth_scale is 100/(2^16-1) = 0.00152590218f
  SteeringPlanner planner(depth_mat, 0.0015259f, fx, cx, cy, 0.26f, 0.55f,
                          0.65f);

  geometry_msgs::TransformStamped transform_to_camera, transform_to_world;
  geometry_msgs::Vector3 velocity_world_frame, acceleration_world_frame;
  geometry_msgs::Vector3 velocity_camera_frame, acceleration_camera_frame;
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    // Lookup for transforms in the TF2 transforming tree
    try {
      transform_to_world = to_world_buffer.lookupTransform(
        world_frame, camera_frame, ros::Time(0));
      transform_to_camera = to_camera_buffer.lookupTransform(
        camera_frame, world_frame, ros::Time(0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
    }
    velocity_world_frame = _state.velocity.linear;
    acceleration_world_frame = _state.acceleration.linear;
    tf2::doTransform(velocity_world_frame, velocity_camera_frame,
                    transform_to_camera);
    tf2::doTransform(acceleration_world_frame, acceleration_camera_frame,
                    transform_to_camera);
  }

  ruckig::InputParameter<3> initial_state;
  initial_state.current_position = {0.0, 0.0, 0.0};
  initial_state.current_velocity = {-velocity_camera_frame.y,
                                    -velocity_camera_frame.z,
                                    velocity_camera_frame.x};
  initial_state.current_acceleration = {-acceleration_world_frame.y,
                                    -acceleration_world_frame.z,
                                    acceleration_world_frame.x};
  initial_state.target_velocity = {0.0, 0.0, 0.0};
  initial_state.target_acceleration = {0.0, 0.0, 0.0};
  initial_state.max_velocity = {5.0, 5.0, 3.0};
  initial_state.max_acceleration = {7.0, 7.0, 5.0};
  initial_state.max_jerk = {20.0, 20.0, 15.0};

  // Transform the coordinate of goal_in_world_frame to
  // the coordinate of goal_in_camera_frame
  geometry_msgs::PointStamped goal_in_camera_frame, goal_in_world_frame;
  goal_in_world_frame.header.frame_id = world_frame;
  goal_in_world_frame.header.stamp = ros::Time::now();
  goal_in_world_frame.point.x = goal_x_world_coordinate;
  goal_in_world_frame.point.y = goal_y_world_coordinate;
  goal_in_world_frame.point.z = goal_z_world_coordinate;
  try {
    tf2::doTransform(goal_in_world_frame, goal_in_camera_frame,
                     transform_to_camera);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
  }

  // Build exploration_vector from the coordinate of goal_in_camera_frame
  Vec3 exploration_vector(-goal_in_camera_frame.point.y,
                          -goal_in_camera_frame.point.z,
                          goal_in_camera_frame.point.x);


  ruckig::Trajectory<3> opt_traj;
  // Find the fastest trajectory candidate
  if (!planner.FindFastestTrajRandomCandidates(
        sampling_mode, initial_state, opt_traj, 0.02, exploration_vector)) {
    // We only sent steering commands when we could not find
    // any feasible trajectory for 1 second in a row.
    const std::lock_guard<std::mutex> lock(state_mutex_);
    if (ros::Duration(ros::Time::now() - last_feasible_planning_time).toSec() < 0.2) {
      steering_value = planner.get_steering() / 8;
      return;
    }
    return;
  }

  // New traj generated
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    steering_value = 0.0f;
    opt_traj.assign_body_to_world_transform(transform_to_world);
    opt_traj.assign_world_to_body_transform(transform_to_camera);
    trajectory_queue_.push_back(opt_traj);
    last_feasible_planning_time = ros::Time::now();
  }
  // TODO:
  // if last_cost
  // last_cost =

  // Visualisation
  // parameters for visualization
  visualization_msgs::Marker pyramids_bases, pyramids_edges,
    polynomial_trajectory;
  pyramids_bases.header.frame_id = pyramids_edges.header.frame_id =
    polynomial_trajectory.header.frame_id = "camera";
  pyramids_bases.header.stamp = pyramids_edges.header.stamp =
    polynomial_trajectory.header.stamp = ros::Time::now();
  pyramids_bases.ns = pyramids_edges.ns = polynomial_trajectory.ns =
    "visualization";
  pyramids_bases.action = pyramids_edges.action = polynomial_trajectory.action =
    visualization_msgs::Marker::ADD;
  pyramids_bases.pose.orientation.w = pyramids_edges.pose.orientation.w =
    polynomial_trajectory.pose.orientation.w = 1.0;
  pyramids_bases.id = 1;
  pyramids_edges.id = 2;
  polynomial_trajectory.id = 3;
  pyramids_bases.type = visualization_msgs::Marker::LINE_STRIP;
  pyramids_edges.type = visualization_msgs::Marker::LINE_LIST;
  polynomial_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  // LINE_STRIP markers use only the x component of scale, for the line width
  pyramids_bases.scale.x = pyramids_edges.scale.x =
    polynomial_trajectory.scale.x = 0.02;
  // Line strip is blue
  pyramids_bases.color.g = pyramids_edges.color.g = 1.0;
  polynomial_trajectory.color.b = 1.0;
  pyramids_bases.color.a = pyramids_edges.color.a =
    polynomial_trajectory.color.a = 1.0;
  geometry_msgs::Point p;

  double trajectory_duration = opt_traj.get_duration();
  std::array<double, 3> trajectory_point_position;
  for (int i = 0; i <= 100; i++) {
    opt_traj.at_time(trajectory_duration * i / 100, trajectory_point_position);
    p.x = trajectory_point_position[2];
    p.y = -trajectory_point_position[0];
    p.z = -trajectory_point_position[1];
    polynomial_trajectory.points.push_back(p);
  }
  // if (autopilot_state_ == States::START)
  visual_pub.publish(polynomial_trajectory);

  // Publish pyramids
  std::vector<Pyramid> pyramids = planner.GetPyramids();
  if (pyramids.empty()) {
    return;
  }
  for (std::vector<Pyramid>::iterator it = pyramids.begin();
       it != pyramids.end(); ++it) {
    for (size_t i = 0; i < 4; i++) {
      p.x = p.y = p.z = 0;
      pyramids_edges.points.push_back(p);
      p.x = (*it)._corners[i].z;
      p.y = -(*it)._corners[i].x;
      p.z = -(*it)._corners[i].y;
      pyramids_bases.points.push_back(p);
      pyramids_edges.points.push_back(p);
    }
    // close the rectangle base
    p.x = (*it)._corners[0].z;
    p.y = -(*it)._corners[0].x;
    p.z = -(*it)._corners[0].y;
    pyramids_bases.points.push_back(p);
  }
  // visual_pub.publish(pyramids_bases);
  // visual_pub.publish(pyramids_edges);
}

bool PlannerNode::loadParameters() {
  if (!quadrotor_common::getParam("goal_x_world_coordinate",
                                  goal_x_world_coordinate, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("goal_y_world_coordinate",
                                  goal_y_world_coordinate, pnh_)) {
    return false;
  }
  if (!quadrotor_common::getParam("goal_z_world_coordinate",
                                  goal_z_world_coordinate, pnh_)) {
    return false;
  }
  if (!base_controller_params_.loadParameters(pnh_)) {
    return false;
  }
  return true;
}

quadrotor_common::QuadStateEstimate
PlannerNode::quad_common_state_from_dodgedrone_state(
  const dodgeros_msgs::QuadState& _state) {
  quadrotor_common::QuadStateEstimate quad_state_;

  // frame ID
  quad_state_.coordinate_frame =
    quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD;

  // velocity
  quad_state_.velocity =
    Eigen::Vector3d(_state.velocity.linear.x, _state.velocity.linear.y,
                    _state.velocity.linear.z);

  // position
  quad_state_.position = Eigen::Vector3d(
    _state.pose.position.x, _state.pose.position.y, _state.pose.position.z);

  // attitude
  quad_state_.orientation =
    Eigen::Quaterniond(_state.pose.orientation.w, _state.pose.orientation.x,
                       _state.pose.orientation.y, _state.pose.orientation.z);

  // angular velocity
  quad_state_.bodyrates =
    Eigen::Vector3d(_state.velocity.angular.x, _state.velocity.angular.y,
                    _state.velocity.angular.z);

  return quad_state_;
}
