// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <riemannian_motion_policy/riemannian_motion_policy.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <Eigen/Eigen>
#include <chrono>
#include <yaml-cpp/yaml.h>


using namespace std::chrono;

namespace {

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

template <typename T>
T yamlValueOr(const YAML::Node& node, const std::string& key, const T& default_value) {
  if (node && node[key]) {
    return node[key].as<T>();
  }
  return default_value;
}
}

namespace riemannian_motion_policy {

void RiemannianMotionPolicy::update_stiffness_and_references(){
  //target by filtering
  /** at the moment we do not use dynamic reconfigure and control the robot via D, K and T **/
  //K = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * K;
  //D = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * D;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  //std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

//RMP calculation for obstacle avoidance
Eigen::VectorXd RiemannianMotionPolicy::calculate_f_obstacle(const Eigen::VectorXd& d_obs, const Eigen::MatrixXd& Jp_obstacle) {
  
  Eigen::Matrix<double, 3, 1> nabla_d = Eigen::MatrixXd::Zero(3,1);
  double alpha_rep; //repulsive potential scalar
  double alpha_damp; //repulsive damping scalar
  double distance; //from given point to nearest obstacle point
  Eigen::Matrix<double, 3, 1> f_repulsive = Eigen::MatrixXd::Zero(3,1);
  Eigen::Matrix<double, 3, 1> P_obs = Eigen::MatrixXd::Zero(3,1);
  Eigen::Matrix<double, 3, 1> f_damping = Eigen::MatrixXd::Zero(3,1);
  Eigen::Matrix<double, 3, 1> f_obstacle = Eigen::MatrixXd::Zero(3,1);
  Eigen::Matrix<double, 3, 1> w = Eigen::MatrixXd::Zero(3,1); // task space linear velocity
  
  // Compute nabla_d
  nabla_d = d_obs/(std::max(d_obs.norm(),0.001));
  distance = d_obs.norm();
  w = Jp_obstacle * dq_;
  
  // Compute f_repulsive
  alpha_rep = eta_rep * std::exp(-distance / mu_rep);
  f_repulsive = alpha_rep * nabla_d.array();

  // Compute alpha_damp
  alpha_damp = eta_damp / ((distance / mu_damp) + epsilon);
  // stretching matrix (only consider velocities when moving towards the obstacle)
  P_obs = std::max(0.0, -w.transpose().dot(nabla_d)) * nabla_d * nabla_d.transpose() * w;
  // Compute f_damping
  f_damping = -alpha_damp * P_obs.array();
  
  // Compute total f_obstacle
  f_obstacle = f_repulsive + f_damping;

  // Assign to f_obstacle_tilde
  Eigen::VectorXd f_obstacle_tilde = Eigen::VectorXd::Zero(6);
  f_obstacle_tilde.topRows(3) = f_obstacle;

  return f_obstacle_tilde;
}

Eigen::MatrixXd RiemannianMotionPolicy::calculate_A_obstacle(const Eigen::VectorXd& d_obs,
                                                             const Eigen::VectorXd& f_obstacle_tilde, double r_a,
                                                             const Eigen::MatrixXd& Jp_obstacle) {
  double alpha_a = 1.0; // tuning paramter for directional stretching
  double beta_x = 1.0; // tuning parameter wight for directional stretching vs Identity
  
  Eigen::Matrix3d H_obs = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d A_stretch = Eigen::Matrix3d::Identity();
  Eigen::Vector3d xi = Eigen::Vector3d::Zero();
  Eigen::Vector3d f_obstacle = Eigen::Vector3d::Zero();
  Eigen::Matrix3d A_obs = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd A_obs_tilde = Eigen::MatrixXd::Zero(6, 6);
  Eigen::Matrix3d identity_3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 3, 1> nabla_d = Eigen::MatrixXd::Zero(3,1);
  Eigen::Matrix<double, 3, 1> w = Eigen::MatrixXd::Zero(3,1); // task space linear velocity
  // define interpolation polynomial (omega)
  double c_1 = (-2.0 / r_a);
  double c_2 = 1.0 / std::pow(r_a, 2);
  double w_r;

  // define linear components of repulsion force
  f_obstacle = f_obstacle_tilde.topRows(3);
  // define repulsion direction and linear velocity
  nabla_d = d_obs/(std::max(d_obs.norm(), 0.001));
  w = Jp_obstacle * dq_;

  // Check for valid values
  if (!d_obs.allFinite() || !f_obstacle.allFinite()) {
      throw std::runtime_error("d_obs or f_obstacle contains invalid values (NaN or inf).");
  }
  if (d_obs.norm() < r_a) {
    w_r = c_2 * d_obs.norm() * d_obs.norm() + c_1 * d_obs.norm() + 1.0;

  }
  else {
    w_r = 0.0;
  }

  double h_v = f_obstacle.norm() + 1/alpha_a * log(1.0 + exp(-2 * alpha_a * f_obstacle.norm()));
  xi = f_obstacle / (h_v);
  A_stretch = xi * xi.transpose();

  H_obs = beta_x * A_stretch + (1.0 - beta_x) * identity_3;
  A_obs = w_r * H_obs ;
  A_obs_tilde.topLeftCorner(3, 3) = A_obs;

  return weight_obstacle * A_obs_tilde;
}

//RMP for target attraction/axis
Eigen::MatrixXd RiemannianMotionPolicy::calculate_target_attraction(const Eigen::VectorXd& error, const Eigen::MatrixXd& jacobian) {
  //declarations
  Eigen::VectorXd f_attract = Eigen::VectorXd::Zero(6);  
  Eigen::MatrixXd A_attract = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd j_translational = jacobian.topRows(3);
  Eigen::MatrixXd j_rotational = jacobian.bottomRows(3);
  Eigen::Vector3d error_position = error.head(3);
  Eigen::Vector3d error_orientation = error.tail(3);
  Eigen::Matrix3d M_far = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d M_near = Eigen::Matrix3d::Zero();
  //target metric
  Eigen::Matrix3d A_position = Eigen::Matrix3d::Zero();
  double alpha = (1 - alpha_min) *exp((-1 * error_position.squaredNorm()) / (2*sigma_a)) + alpha_min;
  double beta = exp((-1 * error_position.squaredNorm()) / (2*sigma_b));
  M_near = Eigen::Matrix3d::Identity();
  M_far = 1.0/(error_position.squaredNorm()) * error_position * error_position.transpose();
  A_position = (beta * b + (1 - beta)) * (alpha * M_near + (1 - alpha) * M_far);
  A_attract.topLeftCorner(3, 3) = A_position;
  
  //axis metric
  Eigen::Matrix3d A_orientation = Eigen::Matrix3d::Zero();
  double beta_axis = exp((-1 * std::pow(error_position.norm(), 2)) / (2*sigma_o));
  A_orientation = (beta_axis * b_axis + 1 - beta_axis) * Eigen::Matrix3d::Identity();
  A_attract.bottomRightCorner(3, 3) = A_orientation;
  
  return  weight_attractor * A_attract;
}

//RMP for global damping
std::pair<Eigen::VectorXd, Eigen::MatrixXd> RiemannianMotionPolicy::calculate_global_damping(const Eigen::MatrixXd& Jp_obstacle) {
  Eigen::VectorXd f_damping = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd velocity = Jp_obstacle * dq_;
  f_damping.topRows(3) = -k_damp * velocity * velocity.norm();
  Eigen::MatrixXd A_damping = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd identity_3 = Eigen::Matrix3d::Identity();
  A_damping.topLeftCorner(3, 3) = velocity.norm() * identity_3 * weight_damping;
  //return A_damping and f_damping
  return std::make_pair(f_damping, A_damping);
}

//RMP calculation of joint limit avoidance
void RiemannianMotionPolicy::rmp_joint_limit_avoidance(){
  //TODO: Implement the calculation of D_sigma fro joint limits
  //calculate sigma_u = 1/(1 + exp(-q))
  Eigen::VectorXd x_lower = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd dx_lower = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd x_upper = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd dx_upper = Eigen::VectorXd::Zero(7);
  for (size_t i = 0; i < 7; ++i) {
    x_lower(i) = (q_(i) - q_lower_limit(i)) / (q_upper_limit(i) - q_lower_limit(i));
    dx_lower(i) = dq_(i) / (q_upper_limit(i) - q_lower_limit(i));

    x_upper(i) = (q_upper_limit(i) - q_(i)) / (q_upper_limit(i) - q_lower_limit(i));
    dx_upper(i) = -dq_(i) / (q_upper_limit(i) - q_lower_limit(i));

    f_joint_limits_lower(i) = kp_joint_limits/((std::pow(x_lower(i),2)/std::pow(l_p,2)) + accel_eps)  - kd_joint_limits * dx_lower(i);
    f_joint_limits_upper(i) = kp_joint_limits/((std::pow(x_upper(i),2)/std::pow(l_p,2)) + accel_eps)  - kd_joint_limits * dx_upper(i);
    A_joint_limits_lower(i,i) = weight_joint_limits * (1 - (1/(1 + exp(-dx_lower(i)/v_m)))) * (1/((x_lower(i)/l_m)+ epsilon_joint_limits));
    A_joint_limits_upper(i,i) = weight_joint_limits * (1 - (1/(1 + exp(-dx_upper(i)/v_m)))) * (1/((x_upper(i)/l_m)+ epsilon_joint_limits));
   
  }
}
 //RMP for joint limit velocity
 void RiemannianMotionPolicy::rmp_joint_velocity_limits(){
  Eigen::VectorXd q_dot_max = Eigen::VectorXd::Zero(7);
  q_dot_max << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61;
  
  for (size_t i = 0; i < 7; ++i) {
    double sign = std::copysign(1.0, dq_(i)); // Returns 1.0 for positive x, -1.0 for negative x.
    double dq_abs = std::abs(dq_(i));
    f_joint_velocity(i) = -k_joint_velocity * sign * (dq_abs - (q_dot_max(i) - 1.5));
    if(dq_abs < (q_dot_max(i) - 1.5)){
      A_joint_velocity(i,i) = 0.0;
    }
    else{
      A_joint_velocity(i,i) = weight_joint_velocity/(1 - std::pow((dq_abs - (q_dot_max(i) - 1.5)), 2)/(std::pow(1.5, 2)));
    }
  }
}

void RiemannianMotionPolicy::rmp_cspacetarget(){
  for (size_t i = 0; i < 7; ++i) {
    if(std::abs(q_(i)) < theta_cspace){
      f_c_space_target(i) = kp_c_space_target(i,i) * (q_0(i) - q_(i)) - kd_c_space_target(i,i) * dq_(i);
    }
    else{
      double diff = q_0(i) - q_(i);  // Scalar difference
      double abs_diff = std::abs(diff) + 1e-6;
      f_c_space_target(i) = kp_c_space_target(i,i) * theta_cspace * diff/abs_diff - kd_c_space_target(i,i) * dq_(i);
    }
  }
    if (f_obs_tildeEE.cols() <= 1) {
      A_c_space_target.setZero();
      return;
    }
    Eigen::MatrixXd f_obs_head = f_obs_tildeEE.block(0, 1, 3, f_obs_tildeEE.cols() - 1);

    Eigen::Vector3d x_dd_d_head = x_dd_des.head<3>(); // dimension 3 x 1
    Eigen::VectorXd dotProducts = f_obs_head.transpose() * x_dd_d_head;

    // Find the index of the maximum dot product.
    int maxIndex;
    dotProducts.maxCoeff(&maxIndex);
    
    // Extract the column that has the highest dot product.
    Eigen::Vector3d bestColumn = f_obs_head.col(maxIndex);
    double gain = std::abs(x_dd_d_head.dot(bestColumn) / (x_dd_d_head.norm() * bestColumn.norm() + 0.001));

    A_c_space_target = weight_c_space_target * gain  * Eigen::MatrixXd::Identity(7,7) ;
}

void RiemannianMotionPolicy::rmp_force_controller() {
  A_force_controller.setZero();
  f_force_controller.setZero();

  const double low = std::max(0.0, force_controller_low_torque_threshold);
  const double high = std::max(low + 1e-6, force_controller_high_torque_threshold);
  const double midpoint = 0.5 * (low + high);
  const double slope = force_controller_sigmoid_steepness / (high - low);

  for (size_t i = 0; i < 7; ++i) {
    const double torque_abs = std::abs(tau_ext_hat_filtered_M(i));
    const double activation = 1.0 / (1.0 + std::exp(-slope * (torque_abs - midpoint)));
    const double inertia = weight_force_controller * force_controller_max_inertia * activation;

    A_force_controller(i, i) = inertia;
    f_force_controller(i) = tau_ext_hat_filtered_M(i) - force_controller_damping * dq_(i);
  }
}
//get global joint acceleration for torque calculation
void RiemannianMotionPolicy::get_ddq(){

  Eigen::MatrixXd I_77 = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd I_66 = Eigen::MatrixXd::Identity(6, 6);
  // get A_total and f_total for obstacle
  Eigen::MatrixXd A_total = Eigen::MatrixXd::Zero(7, 7);
  Eigen::MatrixXd A_total_inv = Eigen::MatrixXd::Zero(7, 7);
  Eigen::VectorXd f_total = Eigen::VectorXd::Zero(7);
  const double obstacle_on = use_obstacle_avoidance ? 1.0 : 0.0;
  const double joint_limits_on = use_joint_limits ? 1.0 : 0.0;
  const double joint_velocity_on = use_joint_velocity_limits ? 1.0 : 0.0;
  const double c_space_on = use_c_space_target ? 1.0 : 0.0;
  const double attractor_on = use_attractor ? 1.0 : 0.0;
  const double damping_on = use_global_damping ? 1.0 : 0.0;
  const double force_on = use_force_controller ? 1.0 : 0.0;

  for (int i = 0; i < number_obstacles; i++) {
    A_total += obstacle_on * jacobian2_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde2.block(0, 6 * i, 6, 6) * jacobian2_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian2_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde2.block(0, 6 * i, 6, 6) * f_obs_tilde2.col(i);
    A_total += obstacle_on * jacobian3_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde3.block(0, 6 * i, 6, 6) * jacobian3_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian3_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde3.block(0, 6 * i, 6, 6) * f_obs_tilde3.col(i);
    A_total += obstacle_on * jacobian4_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde4.block(0, 6 * i, 6, 6) * jacobian4_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian4_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde4.block(0, 6 * i, 6, 6) * f_obs_tilde4.col(i);
    A_total += obstacle_on * jacobian5_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde5.block(0, 6 * i, 6, 6) * jacobian5_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian5_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde5.block(0, 6 * i, 6, 6) * f_obs_tilde5.col(i);
    A_total += obstacle_on * jacobian6_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde6.block(0, 6 * i, 6, 6) * jacobian6_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian6_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde6.block(0, 6 * i, 6, 6) * f_obs_tilde6.col(i);
    A_total += obstacle_on * jacobian7_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde7.block(0, 6 * i, 6, 6) * jacobian7_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobian7_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tilde7.block(0, 6 * i, 6, 6) * f_obs_tilde7.col(i);
    A_total += obstacle_on * jacobianhand_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tildehand.block(0, 6 * i, 6, 6) * jacobianhand_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobianhand_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tildehand.block(0, 6 * i, 6, 6) * f_obs_tildehand.col(i);
    A_total += obstacle_on * jacobianEE_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tildeEE.block(0, 6 * i, 6, 6) * jacobianEE_obstacle.block(0, 7 * i, 6, 7);
    f_total += obstacle_on * jacobianEE_obstacle.block(0, 7 * i, 6, 7).transpose() * A_obs_tildeEE.block(0, 6 * i, 6, 6) * f_obs_tildeEE.col(i);
  }

  A_total += joint_limits_on * (A_joint_limits_upper + A_joint_limits_lower);
  f_total += joint_limits_on * (A_joint_limits_upper * f_joint_limits_upper + A_joint_limits_lower * f_joint_limits_lower);

  A_total += joint_velocity_on * A_joint_velocity;
  f_total += joint_velocity_on * (A_joint_velocity * f_joint_velocity);

  A_total += c_space_on * A_c_space_target;
  f_total += c_space_on * (A_c_space_target * f_c_space_target);

  A_total += attractor_on * jacobian.transpose() * A_attract * jacobian;
  f_total += attractor_on * jacobian.transpose() * A_attract * x_dd_des;

  A_total += damping_on * jacobian.transpose() * A_dampingEE * jacobian;
  f_total += damping_on * jacobian.transpose() * A_dampingEE * f_dampingEE;

  A_total += force_on * A_force_controller;
  f_total += force_on * (A_force_controller * f_force_controller);

  pseudoInverse(A_total, A_total_inv); // get pseudoinverse for pullback 
  ddq_ =  A_total_inv * f_total;
}

void RiemannianMotionPolicy::arrayToMatrix(const std::array<double,7>& inputArray, Eigen::Matrix<double,7,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 7; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

void RiemannianMotionPolicy::arrayToMatrix(const std::array<double,6>& inputArray, Eigen::Matrix<double,6,1>& resultMatrix)
{
 for(long unsigned int i = 0; i < 6; ++i){
     resultMatrix(i,0) = inputArray[i];
   }
}

Eigen::Matrix<double, 7, 1> RiemannianMotionPolicy::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d_M) {  
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
  double difference = tau_d_calculated[i] - tau_J_d_M[i];
  tau_d_saturated[i] =
         tau_J_d_M[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

controller_interface::InterfaceConfiguration
RiemannianMotionPolicy::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}


controller_interface::InterfaceConfiguration RiemannianMotionPolicy::state_interface_configuration()
  const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(robot_name_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
    std::cout << franka_robot_model_name << std::endl;
  }

  const std::string full_interface_name = robot_name_ + "/" + state_interface_name_;

  return state_interfaces_config;
}


CallbackReturn RiemannianMotionPolicy::on_init() {
   UserInputServer input_server_obj(&position_d_target_, &rotation_d_target_, &K, &D, &T);
   std::thread input_thread(&UserInputServer::main, input_server_obj, 0, nullptr);
   input_thread.detach();
   return CallbackReturn::SUCCESS;
}


CallbackReturn RiemannianMotionPolicy::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
  franka_semantic_components::FrankaRobotModel(robot_name_ + "/" + k_robot_model_interface_name,
                                               robot_name_ + "/" + k_robot_state_interface_name));
                                               
  try {
    rclcpp::QoS qos_profile(1); // Depth of the message queue
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    franka_state_subscriber = get_node()->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "franka_robot_state_broadcaster/robot_state", qos_profile, 
    std::bind(&RiemannianMotionPolicy::topic_callback, this, std::placeholders::_1));
    std::cout << "Succesfully subscribed to robot_state_broadcaster" << std::endl;
  }

  catch (const std::exception& e) {
    fprintf(stderr,  "Exception thrown during publisher creation at configure stage with message : %s \n",e.what());
    return CallbackReturn::ERROR;
    }


  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RiemannianMotionPolicy::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_); 
  // Load parameters from yaml file
  YAML::Node config;
  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("riemannian_motion_policy") + "/config.yaml";
    config = YAML::LoadFile(config_path);
  } catch (const std::exception&) {
    const char* home = std::getenv("HOME");
    if (home == nullptr) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not resolve HOME for fallback config path.");
      return CallbackReturn::ERROR;
    }
    try {
      config_path = std::string(home) +
                    "/franka_ros2_ws/src/Riemannian-Motion-Policies-Franka-Emika-Robot/src/config.yaml";
      config = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load config file: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded RMP config: %s", config_path.c_str());

  // Load obstacle avoidance parameters
  const YAML::Node obstacle_cfg = config["obstacle_avoidance"];
  eta_rep = obstacle_cfg["eta_rep"].as<double>();
  mu_rep = obstacle_cfg["mu_rep"].as<double>();
  eta_damp = obstacle_cfg["eta_damp"].as<double>();
  mu_damp = obstacle_cfg["mu_damp"].as<double>();
  epsilon = obstacle_cfg["epsilon"].as<double>();
  weight_obstacle = obstacle_cfg["weight_obstacle"].as<double>();
  use_obstacle_avoidance = yamlValueOr<bool>(obstacle_cfg, "enabled", true);

  // Load attractor parameters
  const YAML::Node attractor_cfg = config["attractor"];
  alpha_min = attractor_cfg["alpha_min"].as<double>();
  sigma_a = attractor_cfg["sigma_a"].as<double>();
  sigma_b = attractor_cfg["sigma_b"].as<double>();
  b = attractor_cfg["b"].as<double>();
  sigma_o = attractor_cfg["sigma_o"].as<double>();
  b_axis = attractor_cfg["b_axis"].as<double>();
  weight_attractor = attractor_cfg["weight_attractor"].as<double>();
  use_attractor = yamlValueOr<bool>(attractor_cfg, "enabled", true);

  // Load global damping parameters
  const YAML::Node global_damping_cfg = config["global_damping"];
  k_damp = global_damping_cfg["k_damp"].as<double>();
  weight_damping = global_damping_cfg["weight_damping"].as<double>();
  use_global_damping = yamlValueOr<bool>(global_damping_cfg, "enabled", true);

  // Load velocity limits parameters
  const YAML::Node velocity_limits_cfg = config["velocity_limits"];
  k_joint_velocity = velocity_limits_cfg["k_joint_velocity"].as<double>();
  weight_joint_velocity = velocity_limits_cfg["weight_joint_velocity"].as<double>();
  use_joint_velocity_limits = yamlValueOr<bool>(velocity_limits_cfg, "enabled", true);

  // Load joint limits parameters
  const YAML::Node joint_limits_cfg = config["joint_limits"];
  kp_joint_limits = joint_limits_cfg["kp_joint_limits"].as<double>();
  kd_joint_limits = joint_limits_cfg["kd_joint_limits"].as<double>();
  l_m = joint_limits_cfg["metric_length_scale"].as<double>();
  epsilon_joint_limits = joint_limits_cfg["epsilon_joint_limits"].as<double>();
  v_m = joint_limits_cfg["metric_velocity_length_scale"].as<double>();
  l_p = joint_limits_cfg["accel_exploder_length_scale"].as<double>();
  accel_eps = joint_limits_cfg["accel_eps"].as<double>();
  weight_joint_limits = joint_limits_cfg["weight_joint_limits"].as<double>();
  use_joint_limits = yamlValueOr<bool>(joint_limits_cfg, "enabled", true);

  // Load C-space Target parameters
  const YAML::Node c_space_target_cfg = config["c_space_target"];
  // Read kp_c_space_target into an Eigen::VectorXd
  std::vector<double> kp_values = c_space_target_cfg["kp_c_space_target"].as<std::vector<double>>();
  Eigen::VectorXd kp_vector = Eigen::Map<Eigen::VectorXd>(kp_values.data(), kp_values.size());
  // Read kd_c_space_target into an Eigen::VectorXd
  std::vector<double> kd_values = c_space_target_cfg["kd_c_space_target"].as<std::vector<double>>();
  Eigen::VectorXd kd_vector = Eigen::Map<Eigen::VectorXd>(kd_values.data(), kd_values.size());
  // Create diagonal matrices
  kp_c_space_target = kp_vector.asDiagonal();
  kd_c_space_target = kd_vector.asDiagonal();
  theta_cspace = c_space_target_cfg["theta"].as<double>();
  weight_c_space_target = c_space_target_cfg["weight_c_space_target"].as<double>();
  use_c_space_target = yamlValueOr<bool>(c_space_target_cfg, "enabled", true);

  // Load force controller primitive parameters
  const YAML::Node force_controller_cfg = config["force_controller"];
  use_force_controller = yamlValueOr<bool>(force_controller_cfg, "enabled", true);
  force_controller_low_torque_threshold =
      yamlValueOr<double>(force_controller_cfg, "low_torque_threshold", 1.0);
  force_controller_high_torque_threshold =
      yamlValueOr<double>(force_controller_cfg, "high_torque_threshold", 5.0);
  force_controller_max_inertia = yamlValueOr<double>(force_controller_cfg, "max_inertia", 5.0);
  force_controller_sigmoid_steepness =
      yamlValueOr<double>(force_controller_cfg, "sigmoid_steepness", 10.0);
  force_controller_damping = yamlValueOr<double>(force_controller_cfg, "damping", 2.0);
  weight_force_controller = yamlValueOr<double>(force_controller_cfg, "weight_force_controller", 1.0);


  // Create the subscriber in the on_activate method
  desired_pose_sub = get_node()->create_subscription<geometry_msgs::msg::Pose>(
        "/riemannian_motion_policy/reference_pose", 
        10,  // Queue size
        std::bind(&RiemannianMotionPolicy::reference_pose_callback, this, std::placeholders::_1)
    );
  std::cout << "Succesfully subscribed to reference pose publisher" << std::endl;

  closest_point_sub_ = get_node()->create_subscription<messages_fr3::msg::ClosestPoint>(
        "/closest_point", 
        10,  // Queue size
        std::bind(&RiemannianMotionPolicy::closestPointCallback, this, std::placeholders::_1)
    );
  std::array<double, 16> initial_pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  std::cout << "Completed Activation process" << std::endl;
  std::array<double, 7> gravity_force_vector_array = franka_robot_model_->getGravityForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_force_vector(gravity_force_vector_array.data());
  tau_gravity = gravity_force_vector;
  return CallbackReturn::SUCCESS;
  
}


controller_interface::CallbackReturn RiemannianMotionPolicy::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

std::array<double, 6> RiemannianMotionPolicy::convertToStdArray(const geometry_msgs::msg::WrenchStamped& wrench) {
    std::array<double, 6> result;
    result[0] = wrench.wrench.force.x;
    result[1] = wrench.wrench.force.y;
    result[2] = wrench.wrench.force.z;
    result[3] = wrench.wrench.torque.x;
    result[4] = wrench.wrench.torque.y;
    result[5] = wrench.wrench.torque.z;
    return result;
}

void RiemannianMotionPolicy::topic_callback(const std::shared_ptr<franka_msgs::msg::FrankaRobotState> msg) {
  // Existing handling of external forces
  O_F_ext_hat_K = convertToStdArray(msg->o_f_ext_hat_k);
  arrayToMatrix(O_F_ext_hat_K, O_F_ext_hat_K_M);
  tau_ext_hat_filtered = msg->tau_ext_hat_filtered;
  arrayToMatrix(tau_ext_hat_filtered, tau_ext_hat_filtered_M);
}


void RiemannianMotionPolicy::reference_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Handle the incoming pose message
    std::cout << "received reference posistion as " <<  msg->position.x << ", " << msg->position.y << ", " << msg->position.z << std::endl;
    position_d_target_ << msg->position.x, msg->position.y,msg->position.z;
    orientation_d_target_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    // You can add more processing logic here
}

void RiemannianMotionPolicy::closestPointCallback(const messages_fr3::msg::ClosestPoint::SharedPtr msg) {
    // Handle the incoming closest point message
    //std::cout << "received closest point as " <<  msg->x << ", " << msg->y << ", " << msg->z << std::endl;
    //std::cout << "closest frame is " << msg->frame << std::endl;
    
    //check the size of frame2x
    number_obstacles = msg->frame2x.size();
    
    // Handle the closest point loop for each obstacle
    if (number_obstacles > 0) {
        d_obs2.resize(3, number_obstacles);
        d_obs3.resize(3, number_obstacles);
        d_obs4.resize(3, number_obstacles);
        d_obs5.resize(3, number_obstacles);
        d_obs6.resize(3, number_obstacles);
        d_obs7.resize(3, number_obstacles);
        d_obshand.resize(3, number_obstacles);
        d_obsEE.resize(3, number_obstacles);
        jacobian_array2.resize(42 * number_obstacles);
        jacobian_array3.resize(42 * number_obstacles);
        jacobian_array4.resize(42 * number_obstacles);
        jacobian_array5.resize(42 * number_obstacles);
        jacobian_array6.resize(42 * number_obstacles);
        jacobian_array7.resize(42 * number_obstacles);
        jacobian_arrayhand.resize(42 * number_obstacles);
        jacobian_arrayEE.resize(42 * number_obstacles);
        jacobian2_obstacle.resize(6, 7 * number_obstacles);
        jacobian3_obstacle.resize(6, 7 * number_obstacles);
        jacobian4_obstacle.resize(6, 7 * number_obstacles);
        jacobian5_obstacle.resize(6, 7 * number_obstacles);
        jacobian6_obstacle.resize(6, 7 * number_obstacles);
        jacobian7_obstacle.resize(6, 7 * number_obstacles);
        jacobianhand_obstacle.resize(6, 7 * number_obstacles);
        jacobianEE_obstacle.resize(6, 7 * number_obstacles);
    }
    for (int i = 0; i < number_obstacles; i++){
      d_obs2.col(i) << msg->frame2x[i], msg->frame2y[i], msg->frame2z[i];
      d_obs3.col(i) << msg->frame3x[i], msg->frame3y[i], msg->frame3z[i];
      d_obs4.col(i) << msg->frame4x[i], msg->frame4y[i], msg->frame4z[i];
      d_obs5.col(i) << msg->frame5x[i], msg->frame5y[i], msg->frame5z[i];
      d_obs6.col(i) << msg->frame6x[i], msg->frame6y[i], msg->frame6z[i];
      d_obs7.col(i) << msg->frame7x[i], msg->frame7y[i], msg->frame7z[i];
      d_obshand.col(i) << msg->framehandx[i], msg->framehandy[i], msg->framehandz[i];
      d_obsEE.col(i) << msg->frameeex[i], msg->frameeey[i], msg->frameeez[i];
    }
    
    // Handle the Jacobian of the closest point
    jacobian_array2 = msg->jacobian2;
    jacobian_array3 = msg->jacobian3;
    jacobian_array4 = msg->jacobian4;
    jacobian_array5 = msg->jacobian5;
    jacobian_array6 = msg->jacobian6;
    jacobian_array7 = msg->jacobian7;
    jacobian_arrayhand = msg->jacobianhand;
    jacobian_arrayEE = msg->jacobianee;
    
    // Convert the Jacobian array to Eigen matrix 6 x (7 * number_obstacles). take 42 entries and reshape to 6 x 7. loop over according size
    for (int i = 0; i < number_obstacles; i++) {
        // Using Eigen::Map to treat a block of 42 doubles as a 6x7 matrix.
        jacobian2_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array2.data() + 42 * i);
        
        jacobian3_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array3.data() + 42 * i);
        
        jacobian4_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array4.data() + 42 * i);
        
        jacobian5_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array5.data() + 42 * i);
        
        jacobian6_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array6.data() + 42 * i);
        
        jacobian7_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array7.data() + 42 * i);
        
        jacobianhand_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_arrayhand.data() + 42 * i);
        
        jacobianEE_obstacle.block(0, 7 * i, 6, 7) =
            Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_arrayEE.data() + 42 * i);
    }
    
    // first three rows of the Jacobian are the translational part
    Jp_obstacle2 = jacobian2_obstacle.topRows(3);
    Jp_obstacle3 = jacobian3_obstacle.topRows(3);
    Jp_obstacle4 = jacobian4_obstacle.topRows(3);
    Jp_obstacle5 = jacobian5_obstacle.topRows(3);
    Jp_obstacle6 = jacobian6_obstacle.topRows(3);
    Jp_obstacle7 = jacobian7_obstacle.topRows(3);
    Jp_obstaclehand = jacobianhand_obstacle.topRows(3);
    Jp_obstacleEE = jacobianEE_obstacle.topRows(3);
}



void RiemannianMotionPolicy::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Check if the size of the effort vector is correct (should match the number of joints, e.g., 7 for Franka)
    if (msg->effort.size() == 7) {
        // Convert std::vector from the message into an Eigen matrix for tau_J
        for (size_t i = 0; i < 7; ++i) {
            tau_J(i) = msg->effort[i];  // Extract the measured joint torques
        }
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "JointState message has incorrect effort size");
    }
}


void RiemannianMotionPolicy::updateJointStates() {
  q_prev = q_;
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

controller_interface::return_type RiemannianMotionPolicy::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {  
  // if (outcounter == 0){
  // std::cout << "Enter 1 if you want to track a desired position or 2 if you want to use free floating with optionally shaped inertia" << std::endl;
  // std::cin >> mode_;
  // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // std::cout << "Mode selected" << std::endl;
  // while (mode_ != 1 && mode_ != 2){
  //   std::cout << "Invalid mode, try again" << std::endl;
  //   std::cin >> mode_;
  // }
  // }
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity_force_vector_array = franka_robot_model_->getGravityForceVector();
  
  jacobian_array =  franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_force_vector(gravity_force_vector_array.data());
  jacobian = Eigen::Map<Eigen::Matrix<double, 6, 7>> (jacobian_array.data());
  
  
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  pseudoInverse(jacobian, jacobian_pinv);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  orientation_d_target_ = Eigen::AngleAxisd(rotation_d_target_[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rotation_d_target_[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rotation_d_target_[2], Eigen::Vector3d::UnitZ());
  updateJointStates(); 

  
  error.head(3) << position - position_d_;

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);
  error.head(3) << position - position_d_;


  //d_obs1 = calculateNearestPointOnSphere(position, sphere_center, sphere_radius);
  //d_obs1 = d_obs_prev1 * 0.99 + d_obs1 * 0.01;
  Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
  x_dd_des = (-K_RMP * (error) - D_RMP * jacobian * dq_);
  A_attract = calculate_target_attraction(error, jacobian);

  if (number_obstacles != 0){
    f_obs_tilde2.resize(6, number_obstacles);
    A_obs_tilde2.resize(6, 6 * number_obstacles);
    f_obs_tilde3.resize(6, number_obstacles);
    A_obs_tilde3.resize(6, 6 * number_obstacles);
    f_obs_tilde4.resize(6, number_obstacles);
    A_obs_tilde4.resize(6, 6 * number_obstacles);
    f_obs_tilde5.resize(6, number_obstacles);
    A_obs_tilde5.resize(6, 6 * number_obstacles);
    f_obs_tilde6.resize(6, number_obstacles);
    A_obs_tilde6.resize(6, 6 * number_obstacles);
    f_obs_tilde7.resize(6, number_obstacles);
    A_obs_tilde7.resize(6, 6 * number_obstacles);
    f_obs_tildehand.resize(6, number_obstacles);
    A_obs_tildehand.resize(6, 6 * number_obstacles);
    f_obs_tildeEE.resize(6, number_obstacles);
    A_obs_tildeEE.resize(6, 6 * number_obstacles);
    for (int i = 0; i < number_obstacles; i++){
      f_obs_tilde2.col(i) = calculate_f_obstacle(d_obs2.col(i), Jp_obstacle2.block(0, 7 * i, 3, 7));
      A_obs_tilde2.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs2.col(i), f_obs_tilde2.col(i), 0.5, Jp_obstacle2.block(0, 7 * i, 3, 7));
      f_obs_tilde3.col(i) = calculate_f_obstacle(d_obs3.col(i), Jp_obstacle3.block(0, 7 * i, 3, 7));
      A_obs_tilde3.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs3.col(i), f_obs_tilde3.col(i), 0.5, Jp_obstacle3.block(0, 7 * i, 3, 7));
      f_obs_tilde4.col(i) = calculate_f_obstacle(d_obs4.col(i), Jp_obstacle4.block(0, 7 * i, 3, 7));
      A_obs_tilde4.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs4.col(i), f_obs_tilde4.col(i), 0.5, Jp_obstacle4.block(0, 7 * i, 3, 7));
      f_obs_tilde5.col(i) = calculate_f_obstacle(d_obs5.col(i), Jp_obstacle5.block(0, 7 * i, 3, 7));
      A_obs_tilde5.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs5.col(i), f_obs_tilde5.col(i), 0.5, Jp_obstacle5.block(0, 7 * i, 3, 7));
      f_obs_tilde6.col(i) = calculate_f_obstacle(d_obs6.col(i), Jp_obstacle6.block(0, 7 * i, 3, 7));
      A_obs_tilde6.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs6.col(i), f_obs_tilde6.col(i), 0.5, Jp_obstacle6.block(0, 7 * i, 3, 7));
      f_obs_tilde7.col(i) = calculate_f_obstacle(d_obs7.col(i), Jp_obstacle7.block(0, 7 * i, 3, 7));
      A_obs_tilde7.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obs7.col(i), f_obs_tilde7.col(i), 0.5, Jp_obstacle7.block(0, 7 * i, 3, 7));
      f_obs_tildehand.col(i) = calculate_f_obstacle(d_obshand.col(i), Jp_obstaclehand.block(0, 7 * i, 3, 7));
      A_obs_tildehand.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obshand.col(i), f_obs_tildehand.col(i), 0.5, Jp_obstaclehand.block(0, 7 * i, 3, 7));
      f_obs_tildeEE.col(i) = calculate_f_obstacle(d_obsEE.col(i), Jp_obstacleEE.block(0, 7 * i, 3, 7));
      A_obs_tildeEE.block(0, 6 * i, 6, 6) = calculate_A_obstacle(d_obsEE.col(i), f_obs_tildeEE.col(i), 0.5, Jp_obstacleEE.block(0, 7 * i, 3, 7));  
    }
  } else {
    f_obs_tilde2 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde2 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tilde3 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde3 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tilde4 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde4 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tilde5 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde5 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tilde6 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde6 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tilde7 = Eigen::MatrixXd::Zero(6,1);
    A_obs_tilde7 = Eigen::MatrixXd::Zero(6,6);
    f_obs_tildehand = Eigen::MatrixXd::Zero(6,1);
    A_obs_tildehand = Eigen::MatrixXd::Zero(6,6);
    f_obs_tildeEE = Eigen::MatrixXd::Zero(6,1);
    A_obs_tildeEE = Eigen::MatrixXd::Zero(6,6);
  }

  const auto damping = calculate_global_damping(jacobian.topRows(3));
  f_dampingEE = damping.first;
  A_dampingEE = damping.second;

  rmp_joint_limit_avoidance();
  rmp_joint_velocity_limits();
  if(number_obstacles > 0 && position_d_target_(0) < 0){
    q_0(0) = -2.4;
  }
  rmp_cspacetarget();
  rmp_force_controller();

  get_ddq();
  
  // Calculate the desired torque
  tau_RMP = ddq_;
  // Calculate friction torques
  N = (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian);
  
  calculate_tau_friction();
  //tau_gravity_error = tau_gravity - gravity_force_vector;

  auto tau_d_placeholder = tau_RMP + coriolis + tau_friction; //add nullspace, friction, gravity and coriolis components to desired torque
  tau_d << tau_d_placeholder;
  tau_d << saturateTorqueRate(tau_d, tau_J_d_M);  // Saturate torque rate to avoid discontinuities
  tau_J_d_M = tau_d;

  // Step 5: Implement a logger that logs every 5 seconds
  static steady_clock::time_point last_log_time = steady_clock::now();
  steady_clock::time_point current_time = steady_clock::now();

  // Check if 5 seconds have passed since the last log
  if (duration_cast<seconds>(current_time - last_log_time).count() >= 0.1) {
    last_log_time = current_time;  // Reset the last log time
        
  }

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  if (outcounter % 1000/update_frequency == 0){
    std::cout<<"ddq" << std::endl;
    std::cout << ddq_<< std::endl;
    std::cout << "error_pose" << std::endl;
    std::cout << error << std::endl;  
  }
  outcounter++;

  
  update_stiffness_and_references();
  return controller_interface::return_type::OK;
}
}

// namespace riemannian_motion_policy
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(riemannian_motion_policy::RiemannianMotionPolicy,
                       controller_interface::ControllerInterface)