/**
Software License Agreement (BSD)

/**
\file      controller.cpp
\brief     Main Heron Controller Implementation
\details   Implements a PID-based controller for the Heron USV.
           Handles multiple control modes:
           - Course Control (Yaw + Speed)
           - Helm Control (Thrust + Yaw Rate)
           - Twist Control (Linear X + Angular Z)
           - Wrench Control (Force X + Torque Z)

           Subscribes to odometry for feedback and publishes low-level thrust commands.

\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
*/
#include <heron_controller/controller.h>
#include <rclcpp/create_timer.hpp>
#include <limits>

namespace {

control_toolbox::AntiWindupStrategy pid_antiwindup_strategy(double i_max, double i_min) {
  control_toolbox::AntiWindupStrategy strategy;
  strategy.type = control_toolbox::AntiWindupStrategy::LEGACY;
  strategy.i_max = i_max;
  strategy.i_min = i_min;
  strategy.legacy_antiwindup = false;
  return strategy;
}

}  // namespace

Controller::Controller(rclcpp::Node::SharedPtr node)
    : node_(node),
      fvel_pid_(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity(),
                pid_antiwindup_strategy(std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity())),
      yr_pid_(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity(),
              pid_antiwindup_strategy(std::numeric_limits<double>::infinity(),
                                      -std::numeric_limits<double>::infinity())),
      y_pid_(0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(),
             pid_antiwindup_strategy(std::numeric_limits<double>::infinity(),
                                     -std::numeric_limits<double>::infinity())) {
  force_compensator_ = new ForceCompensator(node_);

  // Assume no messages are being received. Don't send out anything new until commands are received
  control_mode = NO_CONTROL;

  active_control_srv = node_->create_service<std_srvs::srv::SetBool>(
      "activate_control",
      std::bind(&Controller::activate_control_service, this,
                std::placeholders::_1, std::placeholders::_2));
  is_active_control = false;

  // Timeouts for sensors
  // if no data has been received in a while, disable certain PID controls
  imu_data_timeout_ = node_->declare_parameter<double>("imu_data_timeout", 1 / 5.0);
  imu_data_time_ = 0;
  imu_timeout_ = true;

  vel_data_timeout_ = node_->declare_parameter<double>("vel_data_timeout", 1 / 5.0);
  vel_data_time_ = 0;
  vel_timeout_ = true;

  // Timeouts for control formats
  // if no command has been received in a while, stop sending drive commands
  course_cmd_timeout_ = node_->declare_parameter<double>("course_cmd.timeout", 0.5);
  course_cmd_time_ = 0;

  helm_cmd_timeout_ = node_->declare_parameter<double>("helm_cmd.timeout", 0.5);
  helm_cmd_time_ = 0;

  wrench_cmd_timeout_ = node_->declare_parameter<double>("wrench_cmd.timeout", 0.5);
  wrench_cmd_time_ = 0;

  // If the commands dont show up in this much time don't send out drive commands
  twist_cmd_timeout_ = node_->declare_parameter<double>("twist_cmd.timeout", 0.5);
  twist_cmd_time_ = 0;

  // Setup Fwd Vel Controller
  fvel_dbg_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("fwd_vel_debug", 1000);
  fvel_kf_ = node_->declare_parameter<double>("fwd_vel.kf", 10);       // Feedforward Gain
  fvel_kp_ = node_->declare_parameter<double>("fwd_vel.kp", 90.0);     // Proportional Gain
  fvel_kd_ = node_->declare_parameter<double>("fwd_vel.kd", 1.0);      // Derivative Gain
  fvel_ki_ = node_->declare_parameter<double>("fwd_vel.ki", 0.0);      // Integral Gain
  fvel_imax_ = node_->declare_parameter<double>("fwd_vel.imax", 0.0);  // Clamp Integral Outputs
  fvel_imin_ = node_->declare_parameter<double>("fwd_vel.imin", 0.0);
  fvel_meas_ = 0;

  // Setup Yaw Rate Controller
  yr_dbg_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("yaw_rate_debug", 1000);
  yr_kf_ = node_->declare_parameter<double>("yaw_rate.kf", 10);       // Feedforward Gain
  yr_kp_ = node_->declare_parameter<double>("yaw_rate.kp", 2.0);      // Proportional Gain
  yr_kd_ = node_->declare_parameter<double>("yaw_rate.kd", 1.0);      // Derivative Gain
  yr_ki_ = node_->declare_parameter<double>("yaw_rate.ki", 0.0);      // Integral Gain
  yr_imax_ = node_->declare_parameter<double>("yaw_rate.imax", 0.0);  // Clamp Integral Outputs
  yr_imin_ = node_->declare_parameter<double>("yaw_rate.imin", 0.0);
  yr_meas_ = 0;

  // Setup Yaw Controller
  y_dbg_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("yaw_debug", 1000);

  y_kf_ = node_->declare_parameter<double>("yaw.kf", 5.0);
  y_kp_ = node_->declare_parameter<double>("yaw.kp", 5.0);
  y_kd_ = node_->declare_parameter<double>("yaw.kd", 1.0);
  y_ki_ = node_->declare_parameter<double>("yaw.ki", 0.5);
  y_imax_ = node_->declare_parameter<double>("yaw.imax", 0.0);  // clamp integral output at max yaw yorque
  y_imin_ = node_->declare_parameter<double>("yaw.imin", 0.0);
  y_meas_ = 0;

  RCLCPP_DEBUG(node_->get_logger(), "Fwd Vel Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",
               fvel_kf_, fvel_kp_, fvel_ki_, fvel_kd_, fvel_imax_, fvel_imin_);
  RCLCPP_DEBUG(node_->get_logger(), "Yaw Rate Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",
               yr_kf_, yr_kp_, yr_ki_, yr_kd_, yr_imax_, yr_imin_);
  RCLCPP_DEBUG(node_->get_logger(), "Yaw Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",
               y_kf_, y_kp_, y_ki_, y_kd_, y_imax_, y_imin_);

  fvel_pid_.reset();
  fvel_pid_.initialize(fvel_kp_, fvel_ki_, fvel_kd_, std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(),
                       pid_antiwindup_strategy(fvel_imax_, fvel_imin_));
  fvel_cmd_ = 0;

  yr_pid_.reset();
  yr_pid_.initialize(yr_kp_, yr_ki_, yr_kd_, std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity(),
                     pid_antiwindup_strategy(yr_imax_, yr_imin_));
  yr_cmd_ = 0;

  // Setup Yaw Controller
  y_pid_.reset();
  y_pid_.initialize(y_kp_, y_ki_, y_kd_, std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    pid_antiwindup_strategy(y_imax_, y_imin_));
  y_cmd_ = 0;

  max_fwd_vel_ = node_->declare_parameter<double>("max.fwd_vel", MAX_FWD_VEL);
  max_fwd_force_ = node_->declare_parameter<double>("max.fwd_force", 2 * MAX_FWD_THRUST);  // 2 thrusters
  max_bck_vel_ = node_->declare_parameter<double>("max.bck_vel", MAX_BCK_VEL);
  max_bck_force_ = node_->declare_parameter<double>("max.bck_force", 2 * MAX_BCK_THRUST);

  vel_cov_limit_ = node_->declare_parameter<double>("cov_limits.velocity", 0.28);
  imu_cov_limit_ = node_->declare_parameter<double>("cov_limits.imu", 1.0);

  force_output_.force.x = 0;
  force_output_.force.y = 0;
  force_output_.force.z = 0;

  force_output_.torque.x = 0;
  force_output_.torque.y = 0;
  force_output_.torque.z = 0;
}

double Controller::fvel_compensator() {
  // calculate pid force X
  double fvel_error = fvel_cmd_ - fvel_meas_;
  double fvel_comp_output = fvel_pid_.compute_command(fvel_error, rclcpp::Duration::from_seconds(1.0 / 20.0));
  fvel_comp_output = fvel_comp_output + fvel_kf_ * fvel_cmd_;

  geometry_msgs::msg::Vector3 dbg_info;
  dbg_info.x = fvel_cmd_;
  dbg_info.y = fvel_meas_;
  dbg_info.z = fvel_comp_output;
  fvel_dbg_pub_->publish(dbg_info);

  return fvel_comp_output;
}

double Controller::yr_compensator() {
  // calculate pid torque z
  double yr_error = yr_cmd_ - yr_meas_;
  double yr_comp_output = yr_pid_.compute_command(yr_error, rclcpp::Duration::from_seconds(1.0 / 20.0));
  yr_comp_output = yr_comp_output + yr_kf_ * yr_cmd_;  // feedforward

  geometry_msgs::msg::Vector3 dbg_info;
  dbg_info.x = yr_cmd_;
  dbg_info.y = yr_meas_;
  dbg_info.z = yr_comp_output;
  yr_dbg_pub_->publish(dbg_info);

  return yr_comp_output;
}

double Controller::y_compensator() {
  // calculate pid torque z

  if (y_meas_ < 0) y_meas_ = y_meas_ + 2 * HERON_PI;

  double y_error = y_cmd_ - y_meas_;

  if (fabs(y_error) > HERON_PI) {
    if (y_cmd_ > HERON_PI)  // presumably y_meas_ < PI
      y_error = -(y_meas_ + (2 * HERON_PI - y_cmd_));
    else  // y_cmd_ < pi, y_meas > pi
      y_error = y_cmd_ + (2 * HERON_PI - y_meas_);
  }

  double y_comp_output = y_pid_.compute_command(y_error, rclcpp::Duration::from_seconds(1.0 / 20.0));

  geometry_msgs::msg::Vector3 dbg_info;
  dbg_info.x = y_cmd_;
  dbg_info.y = y_meas_;
  dbg_info.z = y_comp_output;
  y_dbg_pub_->publish(dbg_info);

  return y_comp_output;
}

double deadzone_force(double force, double pos_limit, double neg_limit) {
  if (force > 0) {
    if (force < pos_limit) {
      return 0;
    }  // if
  } else {
    if (force > -neg_limit) {
      return 0;
    }  // if
  }    // else

  return force;
}

void Controller::fwd_vel_mapping() {
  if (fvel_cmd_ > 0) {
    force_output_.force.x =
        deadzone_force(max_fwd_force_ * fvel_cmd_ / max_fwd_vel_, max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
  } else {
    force_output_.force.x =
        deadzone_force(max_bck_force_ * fvel_cmd_ / max_bck_vel_, max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
  }  // else
}  // fwd_vel_mapping

void Controller::update_fwd_vel_control() {
  force_output_.force.x = deadzone_force(fvel_compensator(), max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
}

void Controller::update_yaw_rate_control() { force_output_.torque.z = deadzone_force(yr_compensator(), 2, 2); }

void Controller::update_yaw_control() {
  yr_cmd_ = y_compensator();
  force_output_.torque.z = deadzone_force(yr_compensator(), 2, 2);
}

// Callback to receive twist msgs (cmd_vel style)
void Controller::twist_callback(const geometry_msgs::msg::Twist msg) {
  yr_cmd_ = msg.angular.z;
  update_yaw_rate_control();

  fvel_cmd_ = msg.linear.x;

  if (control_mode == TWIST_LIN_CONTROL) {
    fwd_vel_mapping();
  } else {
    update_fwd_vel_control();
  }

  twist_cmd_time_ = node_->now().seconds();
}  // twist_callback

// Callback to receive raw wrench commands (force along x axis and torque about z axis).
void Controller::wrench_callback(const geometry_msgs::msg::Wrench msg) {
  force_output_.force.x = msg.force.x;
  force_output_.torque.z = msg.torque.z;
  wrench_cmd_time_ = node_->now().seconds();
}

// Callback for yaw command which receives a yaw (rad) and speed (m/s) command
void Controller::course_callback(const heron_msgs::msg::Course msg) {
  // Save Yaw Command and process it
  y_cmd_ = msg.yaw;
  update_yaw_control();

  // Calculate speed command
  fvel_cmd_ = msg.speed;
  update_fwd_vel_control();

  course_cmd_time_ = node_->now().seconds();
}

// Callback for helm commands which receives a thrust percentage (0..1) and a yaw rate (rad/s)
void Controller::helm_callback(const heron_msgs::msg::Helm msg) {
  // Basic Helm Control

  // Calculate Thrust control
  double thrust = msg.thrust;
  if (thrust >= 0)
    force_output_.force.x = thrust * (max_fwd_force_ / 1);
  else
    force_output_.force.x = thrust * (max_bck_force_ / 1);

  // Save yaw rate command to be processed when feedback is available
  yr_cmd_ = msg.yaw_rate;
  update_yaw_rate_control();

  helm_cmd_time_ = node_->now().seconds();
}

// ENU
void Controller::odom_callback(const nav_msgs::msg::Odometry msg) {
  // check if navsat/vel is being integrated into odometry
  if (msg.twist.covariance[0] < vel_cov_limit_ && msg.twist.covariance[7] < vel_cov_limit_) {
    vel_data_time_ = node_->now().seconds();
  }  // if

  // check if imu/data is being integrated into odometry
  if (msg.pose.covariance[35] < imu_cov_limit_ && msg.twist.covariance[35] < imu_cov_limit_) {
    imu_data_time_ = node_->now().seconds();
  }  // if

  y_meas_ = tf2::getYaw(msg.pose.pose.orientation);
  yr_meas_ = msg.twist.twist.angular.z;
  fvel_meas_ = msg.twist.twist.linear.x * std::cos(y_meas_) + msg.twist.twist.linear.y * std::sin(y_meas_);

  switch (control_mode) {
    case COURSE_CONTROL:
      update_fwd_vel_control();
      update_yaw_control();
      break;
    case HELM_CONTROL:
      update_yaw_rate_control();
      break;
    case WRENCH_CONTROL:
      break;
    case TWIST_CONTROL:
      update_fwd_vel_control();
      update_yaw_rate_control();
      break;
    case TWIST_LIN_CONTROL:
      update_yaw_rate_control();
      break;
    case NO_CONTROL:
      break;
  }  // switch
}  // odom_callback

void Controller::console_update() {
  std::string output = "";
  switch (control_mode) {
    case COURSE_CONTROL:
      output = "Boat controlling yaw position";
      break;
    case HELM_CONTROL:
      output = "Boat Controlling yaw rate";
      break;
    case WRENCH_CONTROL:
      output = "Boat in raw wrench/RC control";
      break;
    case TWIST_CONTROL:
      output = "Boat controlling forward and yaw velocity";
      break;
    case TWIST_LIN_CONTROL:
      output = "Boat controlling yaw velocity and mapping velocity linearly";
      break;
    case NO_CONTROL:
      output = "No commands being processed";
      break;
    default:
      break;
  }

  if (imu_timeout_) output += ": IMU data not received or being received too slowly";

  if (vel_timeout_) output += ": GPS Velocity data not received or being received too slowly";

  RCLCPP_INFO(node_->get_logger(), "%s", output.c_str());
}

void Controller::control_update() {
  if (node_->now().seconds() - imu_data_time_ > imu_data_timeout_) {
    imu_timeout_ = true;
  } else {
    imu_timeout_ = false;
  }  // else

  if (node_->now().seconds() - vel_data_time_ > vel_data_timeout_) {
    vel_timeout_ = true;
  } else {
    vel_timeout_ = false;
  }  // else

  if (is_active_control) {
    std::vector<double> find_latest;

    if (!imu_timeout_) {
      find_latest.push_back(twist_cmd_time_);
      find_latest.push_back(helm_cmd_time_);
    }  // if

    if (!imu_timeout_ && !vel_timeout_) {
      find_latest.push_back(course_cmd_time_);
    }  // if

    find_latest.push_back(wrench_cmd_time_);
    double max = *std::max_element(find_latest.begin(), find_latest.end());

    if (max == 0) {
      control_mode = NO_CONTROL;
      force_output_.torque.z = 0;
      force_output_.force.x = 0;
      return;
    } else if (max == twist_cmd_time_ && !imu_timeout_ && !vel_timeout_) {
      control_mode = TWIST_CONTROL;
    } else if (max == twist_cmd_time_ && !imu_timeout_) {
      control_mode = TWIST_LIN_CONTROL;
    } else if (max == course_cmd_time_ && !imu_timeout_ && !vel_timeout_) {
      control_mode = COURSE_CONTROL;
    } else if (max == helm_cmd_time_ && !imu_timeout_) {
      control_mode = HELM_CONTROL;
    } else if (max == wrench_cmd_time_) {
      control_mode = WRENCH_CONTROL;
    }  // elseif
  } else {
    if (node_->now().seconds() - twist_cmd_time_ < twist_cmd_timeout_ && !imu_timeout_ && !vel_timeout_) {
      control_mode = TWIST_CONTROL;
    } else if (node_->now().seconds() - twist_cmd_time_ < twist_cmd_timeout_ && !imu_timeout_) {
      control_mode = TWIST_LIN_CONTROL;
    } else if (node_->now().seconds() - course_cmd_time_ < course_cmd_timeout_ && !imu_timeout_ && !vel_timeout_) {
      control_mode = COURSE_CONTROL;
    } else if (node_->now().seconds() - helm_cmd_time_ < helm_cmd_timeout_ && !imu_timeout_) {
      control_mode = HELM_CONTROL;
    } else if (node_->now().seconds() - wrench_cmd_time_ < wrench_cmd_timeout_) {
      control_mode = WRENCH_CONTROL;
    } else {
      control_mode = NO_CONTROL;
      force_output_.torque.z = 0;
      force_output_.force.x = 0;
      return;
    }  // else
  }    // else

  force_compensator_->pub_thrust_cmd(force_output_);
}

void Controller::activate_control_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
  is_active_control = req->data;
  resp->success = is_active_control;
  resp->message = "Activated Control.";
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller");
  Controller heron_control(node);

  auto twist_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&Controller::twist_callback, &heron_control, std::placeholders::_1));
  auto wrench_sub = node->create_subscription<geometry_msgs::msg::Wrench>(
      "cmd_wrench", 1, std::bind(&Controller::wrench_callback, &heron_control, std::placeholders::_1));
  auto helm_sub = node->create_subscription<heron_msgs::msg::Helm>(
      "cmd_helm", 1, std::bind(&Controller::helm_callback, &heron_control, std::placeholders::_1));
  auto course_sub = node->create_subscription<heron_msgs::msg::Course>(
      "cmd_course", 1, std::bind(&Controller::course_callback, &heron_control, std::placeholders::_1));

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "state/odometry", 1,
      std::bind(&Controller::odom_callback, &heron_control, std::placeholders::_1));
  auto control_output = rclcpp::create_timer(
      node->get_node_base_interface(), node->get_node_timers_interface(), node->get_clock(),
      std::chrono::duration<double>(1.0 / 50.0),
      std::bind(&Controller::control_update, &heron_control));
  auto console_update_timer = rclcpp::create_timer(
      node->get_node_base_interface(), node->get_node_timers_interface(), node->get_clock(),
      std::chrono::duration<double>(1.0),
      std::bind(&Controller::console_update, &heron_control));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
