/**
 * Copyright (c) 2024-2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/
#include <chrono>
#include "rclcpp/logger.hpp"

#include "adi_tmc_coe_core/tmc_coe_stepper_motor.hpp"

using namespace std::chrono_literals;
#include <bitset> // For debug

TmcCoeStepperMotor::TmcCoeStepperMotor(
  rclcpp::Node::SharedPtr p_node,
  TmcCoeInterpreter * p_tmc_coe_interpreter,
  uint8_t slave_number,
  uint8_t motor_number,
  std::string device_name)
: TmcCoeMotor(p_node, p_tmc_coe_interpreter, slave_number, motor_number, device_name),
  logger_prefix_(p_node_->get_logger().get_name()),
  logger_(rclcpp::get_logger(logger_prefix_ + ".TmcCoeStepperMotor.slave" + std::to_string(
      slave_number_) + ".motor" + std::to_string(motor_number_)))
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

}

TmcCoeStepperMotor::~TmcCoeStepperMotor()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
}

void TmcCoeStepperMotor::init()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  const std::string s_commutation_mode = "Commutation Mode";
  const std::string s_position_scaler = "Position Scaler";
  const std::string s_encoder_steps = "Encoder Settings - Steps";
  std::string val = "";
  position_scaler_ = 0;
  encoder_steps_ = 0;
  // Get Commutation Mode
  if (p_tmc_coe_interpreter_->readSDO(slave_number_, s_commutation_mode, &val))
  {
    commutation_mode_ = (stepper_comm_mode_t) std::stoi(val);
    val = "";
    RCLCPP_DEBUG_STREAM(logger_, "Commutation Mode " << commutation_mode_);
  }
  else
  {
    commutation_mode_ = STEPPER_DISABLED_MOTOR;
    RCLCPP_ERROR_STREAM(logger_, "Object Name for Commutation Mode is not Available");
  }

  if (commutation_mode_ > STEPPER_DISABLED_MOTOR)
  {
    if (p_tmc_coe_interpreter_->readSDO(slave_number_, s_position_scaler, &val))
    {
      position_scaler_ = std::stoi(val);
      RCLCPP_INFO_STREAM(logger_, "Position Scaler is " << position_scaler_);
      val = "";
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Object Name for Position Scaler is not Available");
    }
  }

  if ((0 == position_scaler_) && (commutation_mode_ > STEPPER_OPENLOOP_MOTOR))
  {
    if (p_tmc_coe_interpreter_->readSDO(slave_number_, s_encoder_steps, &val))
    {
      encoder_steps_ = std::stoi(val);
      RCLCPP_INFO_STREAM(logger_, "Encoder Steps is " << encoder_steps_);
      val = "";
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Object Name for Encoder Steps is not Available");
    }
  }

  if (0 == param_wheel_diameter_)
  {
    RCLCPP_INFO_STREAM(logger_, "  Velocity unit: rpm");
  }
  else
  {
    RCLCPP_INFO_STREAM(logger_, "  Velocity unit: m/s");
  }

  if ((0 == position_scaler_) && (0 == encoder_steps_))
  {
    RCLCPP_INFO_STREAM(logger_, "  Position unit: pulses");
  }
  else
  {
    RCLCPP_INFO_STREAM(logger_, "  Position unit: angular degrees");
  }

  RCLCPP_INFO_STREAM(logger_, "  Torque unit: mA");

  TmcCoeMotor::initPublisher();
  this->initSubscriber();
  return;
}

void TmcCoeStepperMotor::publishTmcCoeInfo()
{
  RCLCPP_INFO_STREAM_ONCE(logger_, "[" << __func__ << "] called");
  auto message = adi_tmc_coe_interfaces::msg::TmcCoeInfo();
  int8_t mode_of_operation = 0;
  std::string mode_of_operation_str = "";
  message.header.stamp = p_node_->now();
  message.header.frame_id = tmc_coe_info_frame_id_;
  message.interface_name = p_node_->get_parameter("interface_name").as_string();
  message.slave_number = slave_number_;
  message.motor_number = motor_number_;

  mode_of_operation = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display;
  switch (static_cast<mode_of_operation_t>(mode_of_operation))
  {
    case MODE_OF_OPERATION_NONE:
      mode_of_operation_str = "None";
      break;
    case PROFILE_POSITION:
      mode_of_operation_str = "Profile Position";
      break;
    case PROFILE_VELOCITY:
      mode_of_operation_str = "Profile Velocity";
      break;
    case HOMING_MODE:
      mode_of_operation_str = "Homing Mode";
      break;
    case CYCLIC_SYNC_POS:
      mode_of_operation_str = "Cyclic Synchronous Position Mode";
      break;
    case CYCLIC_SYNC_VEL:
      mode_of_operation_str = "Cyclic Synchronous Velocity Mode";
      break;
    case CYCLIC_SYNC_TRQ:
      mode_of_operation_str = "Cyclic Synchronous Torque Mode";
      break;
    default:
      mode_of_operation_str = "None";
      break;
  }
  message.mode_of_operation = mode_of_operation_str;
  message.status_word = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->status_word;

  if (param_pub_actual_vel_)
  {
    if (0 == param_wheel_diameter_)
    {
      message.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value *
        param_add_ratio_vel_;
    }
    else
    {
      message.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value *
        ((PI * param_wheel_diameter_) / SECS_TO_MIN) * param_add_ratio_vel_;
    }

  }

  if (param_pub_actual_pos_)
  {
    if (position_scaler_ > 0)
    {
      message.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
        (ANGULAR_FULL_ROTATION / static_cast<float>(position_scaler_)) * param_add_ratio_pos_;
    }
    else if (encoder_steps_ > 0)
    {
      message.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
        (ANGULAR_FULL_ROTATION / static_cast<float>(encoder_steps_)) * param_add_ratio_pos_;
    }
    else
    {
      message.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
        param_add_ratio_pos_;
    }
  }

  if (param_pub_actual_trq_)
  {
    message.torque = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->torque_actual_value *
      param_add_ratio_trq_;
  }
  tmc_coe_info_pub_->publish(message);
  return;
}

void TmcCoeStepperMotor::initSubscriber()
{
  RCLCPP_INFO_STREAM_ONCE(logger_, "[" << __func__ << "] called");
  if (STEPPER_DISABLED_MOTOR == commutation_mode_)
  {
    RCLCPP_WARN_STREAM(logger_, "Commutation mode is DISABLED. No subscribers");
  }
  else if (STEPPER_OPENLOOP_MOTOR == commutation_mode_)
  {
    RCLCPP_WARN_STREAM(logger_, "Commutation mode is OPEN LOOP.");
    TmcCoeMotor::initSubscriber();
  }
  else if (STEPPER_CLOSEDLOOP_MOTOR <= commutation_mode_)
  {
    RCLCPP_WARN_STREAM(logger_, "Commutation mode is CLOSED LOOP.");
    TmcCoeMotor::initSubscriber();
  }
  return;
}

void TmcCoeStepperMotor::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  float val = msg->linear.x;
  int32_t board_val = 0;
  int prev_cycle_count = 0;
  int retries = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();

  if (0 == param_wheel_diameter_)
  {
    board_val = static_cast<int32_t>(val / param_add_ratio_vel_);
  }
  else
  {
    board_val = val * (SECS_TO_MIN / (PI * param_wheel_diameter_)) * (1 / param_add_ratio_vel_);
  }

  RCLCPP_DEBUG_STREAM(logger_, "Received: " << val << " Target in RPM: " << board_val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= retries)
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_VELOCITY
      if (PROFILE_VELOCITY !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_VELOCITY;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity = board_val;

      while ((p_tmc_coe_interpreter_->getCycleCounter() - prev_cycle_count) < 1)
      {
        // Wait until 1 cycle has elapsed
      }

      if (board_val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity)
      {
        RCLCPP_DEBUG_STREAM(logger_, "Target velocity set successfully");
        break;
      }
      else
      {
        prev_cycle_count = p_tmc_coe_interpreter_->getCycleCounter();
        retries++;
      }
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  if (board_val != p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity)
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set velocity");
  }
  return;
}

void TmcCoeStepperMotor::cmdAbsPosCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int32_t val = msg->data;
  float convert_const_deg = 0.0;
  int32_t unit_val = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();

  if (position_scaler_ > 0)
  {
    convert_const_deg = (static_cast<float>(position_scaler_) / ANGULAR_FULL_ROTATION) * (1 /
      param_add_ratio_pos_);
  }
  else if (encoder_steps_ > 0)
  {
    convert_const_deg = (static_cast<float>(encoder_steps_) / ANGULAR_FULL_ROTATION) * (1 /
      param_add_ratio_pos_);
  }
  else
  {
    convert_const_deg = (1 / param_add_ratio_pos_);
  }
  unit_val = static_cast<int32_t>(val * convert_const_deg);
  RCLCPP_DEBUG_STREAM(logger_, "Received: " << val << " Target: " << unit_val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_POSITION
      if (PROFILE_POSITION !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = unit_val;

      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_START <<
        CW_OMS_NEW_SET_POINT_BIT) |
        (CW_NEW_POSITION_ABSOLUTE << CW_OMS_ABSOLUTE_RELATIVE_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }

    if (p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  p_tmc_coe_interpreter_->startCycleCounter();
  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Set Controlword to ENABLE_OPERATION
      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_STOP <<
        CW_OMS_NEW_SET_POINT_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }
    if (!p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if (unit_val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Target position set successfully");
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set Absolute Position");
  }
  return;
}


void TmcCoeStepperMotor::cmdRelPosCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int32_t val = msg->data;
  float convert_const_deg = 0.0;
  int32_t unit_val = 0;
  int SDO_PDO_retries = p_node_->get_parameter("SDO_PDO_retries").as_int();

  if (position_scaler_ > 0)
  {
    convert_const_deg = (static_cast<float>(position_scaler_) / ANGULAR_FULL_ROTATION) * (1 /
      param_add_ratio_pos_);
  }
  else if (encoder_steps_ > 0)
  {
    convert_const_deg = (static_cast<float>(encoder_steps_) / ANGULAR_FULL_ROTATION) * (1 /
      param_add_ratio_pos_);
  }
  else
  {
    convert_const_deg = (1 / param_add_ratio_pos_);
  }
  unit_val = static_cast<int32_t>(val * convert_const_deg);
  RCLCPP_DEBUG_STREAM(logger_, "Received: " << val << " Target: " << unit_val);

  p_tmc_coe_interpreter_->startCycleCounter();

  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Make sure mode of operation is PROFILE_POSITION
      if (PROFILE_POSITION !=
        p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }

      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = unit_val;

      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_START <<
        CW_OMS_NEW_SET_POINT_BIT) |
        (CW_NEW_POSITION_RELATIVE << CW_OMS_ABSOLUTE_RELATIVE_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }

    if (p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }

  p_tmc_coe_interpreter_->stopCycleCounter();

  p_tmc_coe_interpreter_->startCycleCounter();
  while (SDO_PDO_retries >= p_tmc_coe_interpreter_->getCycleCounter())
  {
    if (p_tmc_coe_interpreter_->isCycleFinished())
    {
      // Set Controlword to ENABLE_OPERATION
      uint32_t control_word = ENABLE_OPERATION | (CW_NEW_SET_POINT_STOP <<
        CW_OMS_NEW_SET_POINT_BIT);
      RCLCPP_DEBUG_STREAM(logger_, "Set controlword to 0b" << std::bitset<16>(control_word));
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = control_word;
    }
    if (!p_tmc_coe_interpreter_->isStatusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if (unit_val == p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Target position set successfully");
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Failed to set Absolute Position");
  }
  return;
}
