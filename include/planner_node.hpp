#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <unistd.h>

#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

#include "RectangularPyramidPlanner/steering_planner.hpp"

// ROS base
#include <ros/console.h>
#include "ros/ros.h"

// autopilot
#include "autopilot_states.h"

// msg
#include <geometry_msgs/TransformStamped.h>

#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

// CV
#include <cv_bridge/cv_bridge.h>

#include <sstream>

// dodgelib
#include "dodgelib/math/types.hpp"
#include "dodgeros_msgs/QuadState.h"

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen catkin
#include <Eigen/Dense>

// quadrotor message
#include <quadrotor_msgs/ControlCommand.h>
#include <dodgeros_msgs/Command.h>

// RPG quad common and control
#include <position_controller/position_controller.h>
#include <position_controller/position_controller_params.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <quadrotor_common/geometry_eigen_conversions.h>

// Ruckig
#include <ruckig/ruckig.hpp>

// ROS TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// this is a global definition of the points to be used
// changes to omit color would need adaptations in
// the visualization too
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace sm = sensor_msgs;
typedef pcl::PointXYZ point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

using namespace CommonMath;
using namespace RectangularPyramidPlanner;
using namespace autopilot;

class PlannerNode {
 public:
  PlannerNode();  // Constructor
  void sampling_mode_callback(const std_msgs::Int8::ConstPtr& msg);
  void start_callback(const std_msgs::Empty::ConstPtr& msg);
  void reset_callback(const std_msgs::Empty::ConstPtr& msg);
  void state_callback(const dodgeros_msgs::QuadState& state);
  void msgCallback(const sensor_msgs::ImageConstPtr& depth_msg);

 private:
  // Member variables and private functions
  std::string camera_frame, world_frame;
  tf2_ros::Buffer to_world_buffer, to_camera_buffer;
  tf2_ros::TransformListener to_world_tf2, to_camera_tf2;
  ros::NodeHandle n_;
  ros::NodeHandle pnh_;
  ros::Publisher trajectoty_pub, point_cloud_pub, visual_pub,
    control_command_pub_;
  ros::Subscriber reset_sub, start_sub, sampling_mode_sub, image_sub, state_sub;
  dodgeros_msgs::QuadState _state;
  bool steering_sent;
  int8_t sampling_mode;
  double last_generated_time;
  std::mutex state_mutex_;

  // Autopilot
  ruckig::Trajectory<3> reference_trajectory_;
  position_controller::PositionController base_controller_;
  position_controller::PositionControllerParams base_controller_params_;
  quadrotor_common::TrajectoryPoint reference_state_;
  States autopilot_state_;

  // State switching variables
  bool state_estimate_available_;
  ros::Time time_of_switch_to_current_state_;
  bool first_time_in_new_state_;
  Eigen::Vector3d initial_start_position_;
  Eigen::Vector3d initial_land_position_;

  // Trajectory execution variables
  std::list<ruckig::Trajectory<3>> trajectory_queue_;
  ros::Time time_start_trajectory_execution_, last_tracking_loop;

  // position controller functions
  quadrotor_common::QuadStateEstimate quad_common_state_from_dodgedrone_state(
    const dodgeros_msgs::QuadState& _state);
  Eigen::Vector3d array3d_to_eigen3d(const std::array<double, 3>& arr);
  void track_trajectory();
  quadrotor_common::ControlCommand track_trajectory_point(
    const quadrotor_common::TrajectoryPoint& reference_point);
  void publishControlCommand(
    const quadrotor_common::ControlCommand& control_cmd);
  bool check_valid_trajectory(const dodgeros_msgs::QuadState& _state,
                              const ruckig::Trajectory<3>& trajectory);
  void get_reference_point_at_time(
    const ruckig::Trajectory<3>& reference_trajectory, const double& point_time,
    quadrotor_common::TrajectoryPoint& reference_point);
  bool loadParameters();
  void setAutoPilotStateForced(const States& new_state);

  // Constants
  static constexpr double kPositionJumpTolerance_ = 0.5;
  double goal_x_world_coordinate;  // [meters]
  double goal_y_world_coordinate;  // [meters]
  double goal_z_world_coordinate;  // [meters]
};

#endif  // PLANNER_NODE_HPP
