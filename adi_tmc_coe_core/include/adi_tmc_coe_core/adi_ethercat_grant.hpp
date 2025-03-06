/**
 * Copyright (c) 2025 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef ADI_ETHERCAT_GRANT_HPP
#define ADI_ETHERCAT_GRANT_HPP

#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cstdlib>

#include <sys/capability.h>
#include <sys/prctl.h>

const std::string executable_ = "/var/tmp/granted";
const char * cap_string_ = "cap_ipc_lock=ep cap_net_raw=ep cap_sys_nice=ep cap_net_admin=ep";

#endif //ADI_ETHERCAT_GRANT_HPP
