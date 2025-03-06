/**
 * Copyright (c) 2024-2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <chrono>
#include "rclcpp/logger.hpp"

#include "adi_tmc_coe_core/tmc_coe_ros2.hpp"
#include "adi_tmc_coe_core/tmc_coe_bldc_motor.hpp"
#include "adi_tmc_coe_core/tmc_coe_stepper_motor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

TmcCoeROS2::TmcCoeROS2(rclcpp::Node::SharedPtr p_node)
: p_node_(p_node),
  p_motor_(1),
  p_tmc_coe_interpreter_(nullptr),
  logger_prefix_(p_node_->get_logger().get_name()),
  logger_(rclcpp::get_logger(logger_prefix_ + ".TmcCoeROS2"))
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  total_slaves_ = 0;
}

TmcCoeROS2::~TmcCoeROS2()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  uint8_t slave_number = 0;
  uint8_t motor_number = 0;
  for (slave_number = 0; slave_number < p_motor_.size(); slave_number++)
  {
    for (motor_number = 0; motor_number < p_motor_[slave_number].size(); motor_number++)
    {
      delete p_motor_[slave_number][motor_number];
      p_motor_[slave_number][motor_number] = nullptr;
    }
  }

  if (nullptr == p_tmc_coe_interpreter_)
  {
    delete p_tmc_coe_interpreter_;
    p_tmc_coe_interpreter_ = nullptr;
  }
  p_node_ = nullptr;
}

bool TmcCoeROS2::initialize()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  bool b_result = false;

  p_param_callback_handle_ = p_node_->add_on_set_parameters_callback(
    std::bind(&TmcCoeROS2::parametersCallback, this, std::placeholders::_1));

  this->initInterfaceParams();

  p_tmc_coe_interpreter_ = new TmcCoeInterpreter(
    param_SDO_PDO_retries_, param_interface_timeout_,
    logger_prefix_);
  total_slaves_ = p_tmc_coe_interpreter_->initInterface(param_interface_name_);

  if (0 < total_slaves_)
  {
    slave_name_str_.resize(total_slaves_ + 1);
    total_motor_per_slave_.resize(total_slaves_ + 1);

    for (int slave_number = 1; slave_number <= total_slaves_; slave_number++)
    {
      // Get TMCM Name per slave (e.g. "1461" for "TMCM-1461")
      slave_name_str_[slave_number] = p_tmc_coe_interpreter_->getSlaveName(slave_number);

      // Get maximum motors per slave
      total_motor_per_slave_[slave_number] = ((std::stoi(slave_name_str_[slave_number])) / 1000);
    }

    this->initTmcParams();
    b_result = this->initAutogenParams();
    if (b_result && this->enableDeviceMotors())
    {
      this->initService();
    }

    // Change all Slaves to SAFE_OP
    RCLCPP_INFO_STREAM(logger_, "All slaves change to SAFE_OP");
    if (b_result)
    {
      for (int slave_number = 1; slave_number <= total_slaves_; slave_number++)
      {
        if (b_result && (1 == param_en_slave_[slave_number - 1]))
        {
          b_result = p_tmc_coe_interpreter_->safeOperationalState(slave_number);
        }
      }
      if (b_result)
      {
        // Create processData thread
        p_tmc_coe_interpreter_->createProcessDataThread();
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Will not create processData thread");
      }
    }

    if (b_result)
    {
      RCLCPP_INFO_STREAM(logger_, "All slaves change to OPERATIONAL");
      // Change all Slaves to OPERATIONAL
      for (int slave_number = 1; slave_number <= total_slaves_; slave_number++)
      {
        if (b_result && (1 == param_en_slave_[slave_number - 1]))
        {
          b_result = p_tmc_coe_interpreter_->operationalState(slave_number, true);
        }
      }
      if (b_result)
      {
        // Create errorCheck thread
        p_tmc_coe_interpreter_->createErrorCheckThread();
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Will not create errorCheck thread");
      }
    }
  }
  RCLCPP_INFO_STREAM_EXPRESSION(logger_, b_result, "Successfully initialized");
  RCLCPP_ERROR_STREAM_EXPRESSION(logger_, !b_result, "Initialization unsuccessful");
  return b_result;
}

void TmcCoeROS2::deInitialize()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  p_tmc_coe_interpreter_->stopInterface();
}

bool TmcCoeROS2::enableDeviceMotors()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  bool b_result = false;
  int slave_number = 1; // Slave Index starts at 1

  while (slave_number <= total_slaves_)
  {
    b_result = this->createMotor(slave_number);
    if (!b_result)
    {
      RCLCPP_ERROR_STREAM(logger_, "Unable to create motor for slave " << slave_number);
    }
    slave_number++;
  }

  return b_result;
}

void TmcCoeROS2::initTmcParams()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  int slave_number = 1;

  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.name = "en_slave";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
  param_desc.description = "Enables/disables active device/slave";
  param_desc.additional_constraints = "1 - Enabled; 0 - Disabled";
  param_desc.read_only = true;
  std::vector<int64_t> default_en_slave = {0};
  p_node_->declare_parameter(param_desc.name, default_en_slave, param_desc);
  param_en_slave_ = p_node_->get_parameter(param_desc.name).as_integer_array();

  param_desc.name = "adhoc_mode";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
  param_desc.description = "Enabled when device/slave is not known";
  param_desc.additional_constraints = "1 - Enabled adhoc mode; 0 - Disabled adhoc mode";
  param_desc.read_only = true;
  std::vector<int64_t> default_adhoc_mode = {0};
  p_node_->declare_parameter(param_desc.name, default_adhoc_mode, param_desc);
  param_adhoc_mode_ = p_node_->get_parameter(param_desc.name).as_integer_array();

  do
  {
    if (1 == param_en_slave_[slave_number - 1]) // If slave is enabled
    {
      param_desc.name = "slv" + std::to_string(slave_number) + ".en_motor";
      param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
      param_desc.description = "Enables/disables active motors per slave";
      param_desc.additional_constraints = "1 - Enabled; 0 - Disabled";
      param_desc.read_only = true;
      std::vector<int64_t> default_en_motor = {0};
      p_node_->declare_parameter(param_desc.name, default_en_motor, param_desc);
    }
    slave_number++;
  }while(slave_number <= total_slaves_);
  return;
}

void TmcCoeROS2::initInterfaceParams()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  rcl_interfaces::msg::FloatingPointRange param_float_range;

  param_desc.name = "interface_name";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "Name of the interface or device as detected by the PC";
  param_desc.additional_constraints = "Possible values: eth0, ... etc";
  param_desc.read_only = true;
  p_node_->declare_parameter(param_desc.name, "", param_desc);
  param_interface_name_ = p_node_->get_parameter(param_desc.name).as_string();

  param_desc.name = "interface_timeout";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Indicates how long the node will wait for the device to be responsive";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  param_float_range.from_value = 3.0;
  param_float_range.to_value = 5.0;
  param_float_range.step = 0.1;
  param_desc.floating_point_range.push_back(param_float_range);
  p_node_->declare_parameter(param_desc.name, 3.0, param_desc);
  param_interface_timeout_ = p_node_->get_parameter(param_desc.name).as_double();
  param_desc.floating_point_range.clear();

  param_desc.name = "SDO_PDO_retries";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Indicates number of retries for SDO and PDO processes";
  param_desc.additional_constraints.clear();
  param_desc.read_only = true;
  param_int_range.from_value = 1;
  param_int_range.to_value = 10;
  param_int_range.step = 1;
  param_desc.integer_range.push_back(param_int_range);
  p_node_->declare_parameter(param_desc.name, 1, param_desc);
  param_SDO_PDO_retries_ = p_node_->get_parameter(param_desc.name).as_int();
  param_desc.integer_range.clear();
  return;
}

bool TmcCoeROS2::initAutogenParams()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  bool b_result = false;
  int slave_number = 1;
  std::vector<std::string> all_obj_name;
  std::vector<std::string> all_index;
  std::vector<std::string> all_sub_index;
  std::vector<std::string> all_datatype;
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  std::string device_name = "";

  do
  {
    if (1 == param_en_slave_[slave_number - 1]) // If slave is enabled
    {
      device_name = slave_name_str_[slave_number];
      param_desc.name = "tmcm_" + device_name + ".obj_name";
      param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
      param_desc.description = "List of Object Name";
      param_desc.additional_constraints.clear();
      param_desc.read_only = true;
      std::vector<std::string> default_obj_name = {};
      p_node_->declare_parameter(param_desc.name, default_obj_name, param_desc);
      all_obj_name = p_node_->get_parameter(param_desc.name).as_string_array();

      param_desc.name = "tmcm_" + device_name + ".index";
      param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
      param_desc.description = "List of Index";
      param_desc.additional_constraints.clear();
      param_desc.read_only = true;
      std::vector<std::string> default_index = {};
      p_node_->declare_parameter(param_desc.name, default_index, param_desc);
      all_index = p_node_->get_parameter(param_desc.name).as_string_array();

      param_desc.name = "tmcm_" + device_name + ".sub_index";
      param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
      param_desc.description = "List of Sub Index";
      param_desc.additional_constraints.clear();
      param_desc.read_only = true;
      std::vector<std::string> default_sub_index = {};
      p_node_->declare_parameter(param_desc.name, default_index, param_desc);
      all_sub_index = p_node_->get_parameter(param_desc.name).as_string_array();

      param_desc.name = "tmcm_" + device_name + ".datatype";
      param_desc.type = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
      param_desc.description = "List of Datatype";
      param_desc.additional_constraints.clear();
      param_desc.read_only = true;
      std::vector<std::string> default_datatype = {};
      p_node_->declare_parameter(param_desc.name, default_datatype, param_desc);
      all_datatype = p_node_->get_parameter(param_desc.name).as_string_array();

      b_result = p_tmc_coe_interpreter_->initDictionary(
        slave_number, all_obj_name, all_index,
        all_sub_index, all_datatype);
      RCLCPP_DEBUG_STREAM_EXPRESSION(
        logger_, b_result, "Parameters tmcm_" << device_name << ".obj_name, tmcm_" << device_name <<
          ".index, tmcm_" << device_name << ".sub_index, tmcm_" << device_name <<
          ".datatpe have equal element size");
      RCLCPP_ERROR_STREAM_EXPRESSION(
        logger_, !b_result, "Parameters tmcm_" << device_name <<
          ".obj_name, tmcm_" << device_name << ".index, tmcm_" << device_name <<
          ".sub_index, tmcm_" <<
          device_name <<
          ".datatpe have inequal element size.\
      All of these must have same number of elements.");
      slave_number++;
    }
  }while(slave_number <= total_slaves_);

  return b_result;
}

void TmcCoeROS2::initService()
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  srv_callback_group_ = p_node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  read_sdo_service_srv_ = p_node_->create_service<adi_tmc_coe_interfaces::srv::ReadWriteSDO>(
    "read_SDO", std::bind(&TmcCoeROS2::readSDOCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  write_sdo_service_srv_ = p_node_->create_service<adi_tmc_coe_interfaces::srv::ReadWriteSDO>(
    "write_SDO", std::bind(&TmcCoeROS2::writeSDOCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  read_pdo_service_srv_ = p_node_->create_service<adi_tmc_coe_interfaces::srv::ReadWritePDO>(
    "read_PDO", std::bind(&TmcCoeROS2::readPDOCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  write_pdo_service_srv_ = p_node_->create_service<adi_tmc_coe_interfaces::srv::ReadWritePDO>(
    "write_PDO", std::bind(&TmcCoeROS2::writePDOCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  change_nmt_state_service_srv_ =
    p_node_->create_service<adi_tmc_coe_interfaces::srv::ChangeNMTState>(
    "change_nmt_state", std::bind(&TmcCoeROS2::changeNMTCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  change_cia402_state_service_srv_ =
    p_node_->create_service<adi_tmc_coe_interfaces::srv::ChangeCia402State>(
    "change_cia402_state", std::bind(&TmcCoeROS2::changeCiA402Callback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);

  cyclic_sync_mode_service_srv_ =
    p_node_->create_service<adi_tmc_coe_interfaces::srv::CyclicSyncMode>(
    "cyclic_sync_mode", std::bind(&TmcCoeROS2::cyclicSyncModeCallback, this, _1, _2),
    rmw_qos_profile_services_default, srv_callback_group_);
  return;
}

rcl_interfaces::msg::SetParametersResult TmcCoeROS2::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG_STREAM(logger_, "[" << __func__ << "] called");
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto & parameter : parameters)
  {
    RCLCPP_DEBUG_STREAM(logger_, "Parameter: " << parameter.get_name());
    RCLCPP_DEBUG_STREAM(logger_, "Parameter Type: " << parameter.get_type());

    if (("en_slave" == parameter.get_name()) || ("adhoc_mode" == parameter.get_name()))
    {
      std::vector<int64_t> param_value = parameter.as_integer_array();
      if (param_value.size() != total_slaves_)
      {
        result.successful = false;
        result.reason = "Incorrect " + parameter.get_name() + " parameter values.";
        RCLCPP_ERROR_STREAM(
          logger_, parameter.get_name() << " has " <<
            param_value.size() << " slave/s. Expecting " << std::to_string(total_slaves_) <<
            " slave/s");

      }
      else
      {
        for (int i = 0; i < total_slaves_; i++)
        {
          if ((0 == param_value[i]) || (1 == param_value[i]))
          {
            result.successful = true;
          }
          else
          {
            result.successful = false;
            result.reason = "Incorrect parameter value; Should be integer array with 0 or 1 only";
          }
        }
      }
    }
    // If parameter is slv<...>.en_motor (e.g. slv1.en_motor)
    else if (std::string::npos != parameter.get_name().find("en_motor"))
    {
      std::vector<std::int64_t> param_value = parameter.as_integer_array();
      int param_size = param_value.size();
      for (int i = 0; i < param_size; i++)
      {
        if ((0 == param_value[i]) || (1 == param_value[i]))
        {
          result.successful = true;
        }
        else
        {
          result.successful = false;
          result.reason = "Incorrect parameter value; Should be integer array with 0 or 1 only";
        }
      }
    }
    // If parameter is <...>.obj_name (e.g. tmcm_1241.obj_name)
    else if (std::string::npos != parameter.get_name().find("obj_name"))
    {
      std::vector<std::string> param_value = parameter.as_string_array();
      if (param_value.empty())
      {
        result.successful = false;
        result.reason = "Empty " + parameter.get_name() + " parameter.";
      }
      else
      {
        result.successful = true;
      }
    }
    else
    {
      result.successful = true;
    }
  }

  return result;
}

bool TmcCoeROS2::createMotor(int slave_number)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");

  bool b_result = false;
  int motor_number = 0;
  uint8_t motor_type = 0;

  motor_type = (((std::stoi(slave_name_str_[slave_number])) % 1000) / 100);

  // Set p_motor_ vector to total_slaves_
  p_motor_.resize(total_slaves_ + 1);

  p_motor_[slave_number].resize(total_motor_per_slave_[slave_number]);
  RCLCPP_INFO_STREAM(
    logger_, "Total motors for Slave " << slave_number << " is " <<
      p_motor_[slave_number].size());

  if (1 == param_adhoc_mode_[slave_number - 1])
  {
    while (motor_number < total_motor_per_slave_[slave_number])
    {
      RCLCPP_DEBUG_STREAM(
        logger_, "Creating Motor " << motor_number << " for Slave " <<
          slave_number);
      p_motor_[slave_number][motor_number] = new TmcCoeMotor(
        p_node_, p_tmc_coe_interpreter_,
        slave_number, motor_number, slave_name_str_[slave_number]);
      p_motor_[slave_number][motor_number]->init();
      motor_number++;
    }
    b_result = true;
  }
  else if (MOTOR_TYPE_BLDC == motor_type)
  {
    while (motor_number < total_motor_per_slave_[slave_number])
    {
      RCLCPP_DEBUG_STREAM(
        logger_, "Creating BLDC Motor " << motor_number << " for Slave " <<
          slave_number);
      p_motor_[slave_number][motor_number] = new TmcCoeBldcMotor(
        p_node_, p_tmc_coe_interpreter_,
        slave_number, motor_number, slave_name_str_[slave_number]);
      p_motor_[slave_number][motor_number]->init();
      motor_number++;
    }
    b_result = true;
  }
  else if ((MOTOR_TYPE_STEPPER_MIN <= motor_type) && (MOTOR_TYPE_STEPPER_MAX >= motor_type))
  {
    while (motor_number < total_motor_per_slave_[slave_number])
    {
      RCLCPP_DEBUG_STREAM(
        logger_, "Creating Stepper Motor " << motor_number << " for Slave " <<
          slave_number);
      p_motor_[slave_number][motor_number] = new TmcCoeStepperMotor(
        p_node_, p_tmc_coe_interpreter_,
        slave_number, motor_number, slave_name_str_[slave_number]);
      p_motor_[slave_number][motor_number]->init();
      motor_number++;
    }
    b_result = true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid motor type");
  }
  return b_result;
}

bool TmcCoeROS2::readSDOCallback(
  adi_tmc_coe_interfaces::srv::ReadWriteSDO::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ReadWriteSDO::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      req->slave_number << " object_name=" << req->object_name << " value=" << req->value);
  std::string local_value = "";
  res->result = false;

  if (total_slaves_ >= req->slave_number)
  {
    if (p_tmc_coe_interpreter_->readSDO(req->slave_number, req->object_name, &local_value))
    {
      res->output = local_value;
      res->result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Failed to read " << req->object_name);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Slave number invalid.");
  }

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, res->result, "Slave " << std::to_string(req->slave_number) <<
      " read SDO successful; " << req->object_name << " = " << res->output);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !res->result, "Slave " << std::to_string(req->slave_number)
                                    << " read SDO failed");
  return true;
}

bool TmcCoeROS2::writeSDOCallback(
  adi_tmc_coe_interfaces::srv::ReadWriteSDO::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ReadWriteSDO::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      req->slave_number << " object_name=" << req->object_name << " value=" << req->value);

  std::string local_value = req->value;
  res->result = false;

  if (total_slaves_ >= req->slave_number)
  {
    if (p_tmc_coe_interpreter_->writeSDO(req->slave_number, req->object_name, &local_value))
    {
      res->output = local_value;
      res->result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Failed to write to " << req->object_name);
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Slave number invalid.");
  }

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, res->result, "Slave " << std::to_string(req->slave_number) <<
      " write SDO successful; " << req->object_name << " = " << res->output);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !res->result, "Slave " << std::to_string(req->slave_number)
                                    << " write SDO failed");
  return true;
}

bool TmcCoeROS2::readPDOCallback(
  adi_tmc_coe_interfaces::srv::ReadWritePDO::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ReadWritePDO::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      req->slave_number << " cmd=" << req->cmd << " value=" << req->value);
  int actual_value = 0;
  std::string cmd_upper_case = "";

  std::transform(req->cmd.begin(), req->cmd.end(), req->cmd.begin(), ::toupper);
  cmd_upper_case = req->cmd;

  if (req->slave_number <= total_slaves_)
  {
    if (cmd_upper_case.compare("MODES OF OPERATION DISPLAY") == 0)
    {
      actual_value =
        p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display;
      res->result = true;
    }
    else if (cmd_upper_case.compare("STATUS WORD") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->status_word;
      res->result = true;
    }
    else if (cmd_upper_case.compare("ACTUAL POSITION") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->position_actual_value;
      res->result = true;
    }
    else if (cmd_upper_case.compare("DEMAND POSITION") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->position_demand_value;
      res->result = true;
    }
    else if (cmd_upper_case.compare("ACTUAL VELOCITY") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->velocity_actual_value;
      res->result = true;
    }
    else if (cmd_upper_case.compare("DEMAND VELOCITY") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->velocity_demand_value;
      res->result = true;
    }
    else if (cmd_upper_case.compare("ACTUAL TORQUE") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->torque_actual_value;
      res->result = true;
    }
    else if (cmd_upper_case.compare("DEMAND TORQUE") == 0)
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->torque_demand_value;
      res->result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Invalid PDO read request");
      res->result = false;
    }
    res->actual_value = actual_value;
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid slave number");
    res->result = false;
  }

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, res->result, "Slave " << std::to_string(req->slave_number) <<
      " read PDO successful; " << req->cmd << " = " << res->actual_value);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !res->result, "Slave " << std::to_string(req->slave_number)
                                    << " read PDO failed");
  return true;
}

bool TmcCoeROS2::writePDOCallback(
  adi_tmc_coe_interfaces::srv::ReadWritePDO::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ReadWritePDO::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      req->slave_number << " cmd=" << req->cmd << " value=" << req->value);
  bool b_result = false;
  int actual_value = 0;
  std::string cmd_upper_case = "";

  std::transform(req->cmd.begin(), req->cmd.end(), req->cmd.begin(), ::toupper);
  cmd_upper_case = req->cmd;

  if (req->slave_number <= total_slaves_)
  {
    p_tmc_coe_interpreter_->startCycleCounter();
    if (cmd_upper_case.compare("MODES OF OPERATION") == 0)
    {
      while (param_SDO_PDO_retries_ >= p_tmc_coe_interpreter_->getCycleCounter())
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->modes_of_operation =
            static_cast<int8_t>(req->value);
          actual_value =
            p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display;
          if (actual_value == req->value)
          {
            b_result = true;
            break;
          }
        }
      }
      actual_value =
        p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display;
      p_tmc_coe_interpreter_->stopCycleCounter();
      res->result = b_result;
      RCLCPP_ERROR_STREAM_EXPRESSION(logger_, !b_result, "Setting Modes of Operation failed");
    }
    else if (cmd_upper_case.compare("CONTROLWORD") == 0)
    {
      while (param_SDO_PDO_retries_ >= p_tmc_coe_interpreter_->getCycleCounter())
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->control_word =
            static_cast<uint16_t>(req->value);
          b_result = true; // Note no validation if controlword is set successfully.
        }
      }
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->status_word;
      p_tmc_coe_interpreter_->stopCycleCounter();
      res->result = b_result;
    }
    else if (cmd_upper_case.compare("TARGET POSITION") == 0)
    {
      while (param_SDO_PDO_retries_ >= p_tmc_coe_interpreter_->getCycleCounter())
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_position =
            static_cast<int32_t>(req->value);
          b_result = true; // Note no validation if target position is set successfully.
        }
      }
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->position_actual_value;
      p_tmc_coe_interpreter_->stopCycleCounter();
      res->result = b_result;
    }
    else if (cmd_upper_case.compare("TARGET VELOCITY") == 0)
    {
      while (param_SDO_PDO_retries_ >= p_tmc_coe_interpreter_->getCycleCounter())
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_velocity =
            static_cast<int32_t>(req->value);
          b_result = true; // Note no validation if target position is set successfully.
        }
      }
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->velocity_actual_value;
      p_tmc_coe_interpreter_->stopCycleCounter();
      res->result = b_result;
    }
    else if (cmd_upper_case.compare("TARGET TORQUE") == 0)
    {
      while (param_SDO_PDO_retries_ >= p_tmc_coe_interpreter_->getCycleCounter())
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_torque =
            static_cast<int16_t>(req->value);
          b_result = true; // Note no validation if target position is set successfully.
        }
      }
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->torque_actual_value;
      p_tmc_coe_interpreter_->stopCycleCounter();
      res->result = b_result;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Invalid PDO write request");
      res->result = false;
    }
    res->actual_value = actual_value;
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid slave number");
    res->result = false;
  }

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, res->result, "Slave " << std::to_string(req->slave_number) <<
      " write PDO successful; " << req->cmd << " = " << res->actual_value);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !res->result, "Slave " << std::to_string(req->slave_number)
                                    << " write PDO failed");
  return true;
}

bool TmcCoeROS2::changeNMTCallback(
  adi_tmc_coe_interfaces::srv::ChangeNMTState::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ChangeNMTState::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      std::to_string(req->slave_number) << " request_state=" << req->request_state);
  bool b_result = false;
  nmt_state_t current_state = NONE;
  nmt_state_t request_state = NONE;
  std::string current_state_str = "";
  std::string request_state_upper_case = "";

  std::transform(
    req->request_state.begin(), req->request_state.end(), req->request_state.begin(),
    ::toupper);
  request_state_upper_case = req->request_state;

  if (req->slave_number <= total_slaves_)
  {
    if (request_state_upper_case.compare("INIT") == 0)
    {
      request_state = INIT;
      b_result = true;
    }
    else if (request_state_upper_case.compare("PREOP") == 0)
    {
      request_state = PRE_OP;
      b_result = true;
    }
    else if (request_state_upper_case.compare("SAFEOP") == 0)
    {
      request_state = SAFE_OP;
      b_result = true;
    }
    else if (request_state_upper_case.compare("OPERATIONAL") == 0)
    {
      request_state = OPERATIONAL;
      b_result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Invalid NMT state");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid slave number");
    current_state_str = "UNKNOWN";
  }

  if (b_result)
  {
    current_state = p_tmc_coe_interpreter_->changeNMTState(req->slave_number, request_state);
    if (request_state != current_state)
    {
      RCLCPP_ERROR_STREAM(logger_, "State Change Failed.");
      b_result = false;
    }

    switch (current_state)
    {
      case INIT:
        current_state_str = "INIT";
        break;
      case PRE_OP:
        current_state_str = "PREOP";
        break;
      case SAFE_OP:
        current_state_str = "SAFEOP";
        break;
      case OPERATIONAL:
        current_state_str = "OPERATIONAL";
        if (b_result)
        {
          b_result = p_tmc_coe_interpreter_->operationalState(req->slave_number, false);
        }
        break;
      default:
        current_state_str = "UNKNOWN";
    }
    res->result = b_result;
  }

  res->current_state = current_state_str;

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, b_result, "Slave " << std::to_string(req->slave_number) <<
      " change NMT State successful; current state is " << res->current_state);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !b_result, "Slave " << std::to_string(req->slave_number) <<
      " change NMT State failed; current state is " << res->current_state);
  return true;
}

bool TmcCoeROS2::changeCiA402Callback(
  adi_tmc_coe_interfaces::srv::ChangeCia402State::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::ChangeCia402State::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      std::to_string(req->slave_number) << " request_state=" << req->request_state);

  bool b_result = false;
  fsa_state_t current_state = UNKNOWN;
  fsa_state_t request_state = UNKNOWN;
  std::string current_state_str = "";
  std::string request_state_upper_case = "";

  std::transform(
    req->request_state.begin(), req->request_state.end(), req->request_state.begin(),
    ::toupper);
  request_state_upper_case = req->request_state;

  if (req->slave_number <= total_slaves_)
  {
    if (request_state_upper_case.compare("SWITCH ON DISABLE") == 0)
    {
      request_state = SWITCH_ON_DISABLE;
      b_result = true;
    }
    else if (request_state_upper_case.compare("READY TO SWITCH ON") == 0)
    {
      request_state = READY_TO_SWITCH_ON;
      b_result = true;
    }
    else if (request_state_upper_case.compare("SWITCHED ON") == 0)
    {
      request_state = SWITCHED_ON;
      b_result = true;
    }
    else if (request_state_upper_case.compare("OPERATION ENABLED") == 0)
    {
      request_state = OPERATION_ENABLED;
      b_result = true;
    }
    else if (request_state_upper_case.compare("QUICK STOP ACTIVE") == 0)
    {
      request_state = QUICK_STOP_ACTIVE;
      b_result = true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Invalid state");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Invalid slave number");
    current_state_str = "UNKNOWN";
  }

  if (b_result)
  {
    current_state = p_tmc_coe_interpreter_->changeCiA402State(req->slave_number, request_state);
    if (request_state == current_state)
    {
      b_result = true;
    }
    else
    {
      b_result = false;
    }

    switch (current_state)
    {
      case NOT_READY_TO_SWITCH_ON:
        current_state_str = "NOT READY TO SWITCH ON";
        break;
      case SWITCH_ON_DISABLE:
        current_state_str = "SWITCH ON DISABLE";
        break;
      case READY_TO_SWITCH_ON:
        current_state_str = "READY TO SWITCH ON";
        break;
      case SWITCHED_ON:
        current_state_str = "SWITCHED ON";
        break;
      case OPERATION_ENABLED:
        current_state_str = "OPERATION ENABLED";
        break;
      case QUICK_STOP_ACTIVE:
        current_state_str = "QUICK STOP ACTIVE";
        break;
      case FAULT_REACTION_ACTIVE:
        current_state_str = "FAULT REACTION ACTIVE";
        break;
      case FAULT:
        current_state_str = "FAULT";
        break;
      default:
        current_state_str = "UNKNOWN";
    }
  }

  res->result = b_result;
  res->current_state = current_state_str;

  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, b_result, "Slave " << std::to_string(req->slave_number) <<
      " change CiA402 State successful; current state is " << res->current_state);
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !b_result, "Slave " << std::to_string(req->slave_number) <<
      " change CiA402 State failed; current state is " << res->current_state);
  return true;
}

bool TmcCoeROS2::cyclicSyncModeCallback(
  adi_tmc_coe_interfaces::srv::CyclicSyncMode::Request::SharedPtr req,
  adi_tmc_coe_interfaces::srv::CyclicSyncMode::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(logger_, "[" << __func__ << "] called");
  RCLCPP_DEBUG_STREAM(
    logger_, " Request: slave_number=" <<
      std::to_string(req->slave_number) << " cs_cmd=" << req->cs_cmd <<
      " interpolation_time_period= "
                                       << std::to_string(req->interpolation_time_period) <<
      " interpolation_time_index= " <<
      std::to_string(req->interpolation_time_index));

  bool b_result = false;
  double interpolation_time = req->interpolation_time_period * pow(
    10,
    req->interpolation_time_index);
  int interpolation_time_us = interpolation_time * 1000000;
  RCLCPP_DEBUG_STREAM(logger_, "interpolation time: " << interpolation_time << " s");
  std::string interpolation_time_period_str = std::to_string(req->interpolation_time_period);
  std::string interpolation_time_index_str = std::to_string(req->interpolation_time_index);
  std::string cs_cmd_upper = req->cs_cmd;
  uint16_t motor_type = 0;
  uint32_t index = 0;

  std::transform(cs_cmd_upper.begin(), cs_cmd_upper.end(), cs_cmd_upper.begin(), ::toupper);

  if ((req->slave_number <= total_slaves_) && (req->slave_number > 0))
  {
    motor_type = (((std::stoi(slave_name_str_[req->slave_number])) % 1000) / 100);
    //req->interpolation_time_period will always be in UINT8 range
    if ((req->interpolation_time_index >= INTERPOLATION_TIME_INDEX_MIN) &&
      (req->interpolation_time_index <= INTERPOLATION_TIME_INDEX_MAX))
    {
      if (MOTOR_TYPE_BLDC == motor_type)
      {
        b_result = p_tmc_coe_interpreter_->writeSDO(
          req->slave_number,
          "Interpolation time period - Time units", &interpolation_time_period_str);
        if (b_result)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Interpolation Time period set");
        }
        b_result = p_tmc_coe_interpreter_->writeSDO(
          req->slave_number,
          "Interpolation time period - Time index", &interpolation_time_index_str);
        if (b_result)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Interpolation Time index set");
        }
      }
      else if ((MOTOR_TYPE_STEPPER_MAX >= motor_type) && (MOTOR_TYPE_STEPPER_MIN <= motor_type))
      {
        b_result = p_tmc_coe_interpreter_->writeSDO(
          req->slave_number,
          "Interpolation Time Period - Interpolation time period value",
          &interpolation_time_period_str);
        if (b_result)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Interpolation Time period set");
        }
        b_result = p_tmc_coe_interpreter_->writeSDO(
          req->slave_number,
          "Interpolation Time Period - Interpolation time index", &interpolation_time_index_str);
        if (b_result)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Interpolation Time index set");
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Cyclic mode service failed: invalid motor type");
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(logger_, "Cyclic mode service failed: invalid interpolation time input");
    }

    p_tmc_coe_interpreter_->startCycleCounter();

    if (cs_cmd_upper.compare("CSP") == 0)
    {
      RCLCPP_INFO_STREAM(logger_, "Cyclic Synchronous Position mode");
      while (p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->modes_of_operation =
            CYCLIC_SYNC_POS;
          if (CYCLIC_SYNC_POS ==
            p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if (CYCLIC_SYNC_POS !=
        p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
      {
        RCLCPP_ERROR_STREAM(logger_, "Modes of operation not set to Cyclic Sync Pos");
        b_result = false;
      }

      if (b_result)
      {
        for (index = 0; index < req->value.size(); index++)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Value[" << index << "]: " << req->value[index]);
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_position =
            req->value[index];
          rclcpp::sleep_for(std::chrono::microseconds(interpolation_time_us));
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Cyclic Synchronous Position failed.");
      }
    }
    else if (cs_cmd_upper.compare("CSV") == 0)
    {
      RCLCPP_INFO_STREAM(logger_, "Cyclic Synchronous Velocity mode");
      while (p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->modes_of_operation =
            CYCLIC_SYNC_VEL;
          if (CYCLIC_SYNC_VEL ==
            p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if (CYCLIC_SYNC_VEL !=
        p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
      {
        RCLCPP_ERROR_STREAM(logger_, "Modes of operation not set to Cyclic Sync Vel");
        b_result = false;
      }

      if (b_result)
      {
        for (index = 0; index < req->value.size(); index++)
        {
          RCLCPP_DEBUG_STREAM(logger_, "Value[" << index << "]: " << req->value[index]);
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_velocity =
            req->value[index];
          rclcpp::sleep_for(std::chrono::microseconds(interpolation_time_us));
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Cyclic Synchronous Velocity failed.");
      }
    }
    else if (cs_cmd_upper.compare("CST") == 0)
    {
      while (p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if (p_tmc_coe_interpreter_->isCycleFinished())
        {
          RCLCPP_DEBUG_STREAM(logger_, "Value[" << index << "]: " << req->value[index]);
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->modes_of_operation =
            CYCLIC_SYNC_TRQ;
          if (CYCLIC_SYNC_TRQ ==
            p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if (CYCLIC_SYNC_TRQ !=
        p_tmc_coe_interpreter_->input_pdo_[req->slave_number]->modes_of_operation_display)
      {
        RCLCPP_ERROR_STREAM(logger_, "Modes of operation not set to Cyclic Sync Trq");
        b_result = false;
      }

      if (b_result)
      {
        for (index = 0; index < req->value.size(); index++)
        {
          p_tmc_coe_interpreter_->output_pdo_[req->slave_number]->target_torque = req->value[index];
          rclcpp::sleep_for(std::chrono::microseconds(interpolation_time_us));
        }
      }
      else
      {
        RCLCPP_ERROR_STREAM(logger_, "Cyclic Synchronous Torque failed.");
      }
    }

  }
  else
  {
    RCLCPP_ERROR_STREAM(logger_, "Slave number invalid");
  }
  res->result = b_result;
  RCLCPP_INFO_STREAM_EXPRESSION(
    logger_, b_result, "Slave " << std::to_string(req->slave_number) <<
      " " << req->cs_cmd << " successful");
  RCLCPP_WARN_STREAM_EXPRESSION(
    logger_, !b_result, "Slave " << std::to_string(req->slave_number) <<
      " " << req->cs_cmd << " failed");
  return true;
}
