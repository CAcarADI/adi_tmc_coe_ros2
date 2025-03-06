/**
 * Copyright (c) 2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "adi_tmc_coe_core/adi_ethercat_grant.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  std::string shell_command;
  int32_t return_value;

  (void) argc;

  cap_t cap_handle = cap_from_text(cap_string_);

  unlink(executable_.c_str());

  shell_command = "cp " + std::string(argv[1]) + " " + executable_;
  return_value = system(shell_command.c_str());
  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("shell command"), "Executable successfully copied to " <<
        executable_);

    return_value = chown(executable_.c_str(), getuid(), getgid());
  }
  else
  {

  }

  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("chown command"),
      "Executable successfully changed ownership");

    if (NULL != cap_handle)
    {
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("cap_from_text"),
        "Successfully created capability set");

      return_value = cap_set_file(executable_.c_str(), cap_handle);
    }
  }

  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("cap_set_file"), "Successfully set file capability");

    if (cap_handle)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("cap_free"), "Freeing capability");
      cap_free(cap_handle);
    }

    return_value = setuid(getuid());
  }

  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("setuid"), "Successfully set user ID");

    return_value = setgid(getgid());
  }

  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("setgid"), "Successfully set group ID");

    // Allow core dumps
    prctl(PR_SET_DUMPABLE, 1, 0, 0, 0);
    return_value = execv(executable_.c_str(), argv + 1);
  }

  if (0 == return_value)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("execv"), "Successfully set user ID");
  }

  return 0;
}
