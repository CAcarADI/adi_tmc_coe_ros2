#!/bin/bash

echo "====================================Ethercat Grant============================================"

workspace_root=$(pwd)
WS_found=false


if [ "$(whoami)" != root ]
then
  echo "Please run this script with sudo. Exiting!"
  exit
else
	# Check if the install directory exists
	# Check 3 levels below (assuming user run the script inside the package)
	if [ -d "$workspace_root/install" ]; then
	  echo "The install directory is located at: $workspace_root/install"
	  WS_found=true
	else
	  cd ../../../..
	  workspace_root=$(pwd)
	fi

	if [ "$WS_found" = false ] && [ -d "$workspace_root/install" ]; then
	  echo "The install directory is located at: $workspace_root/install"
	  WS_found=true
	fi

	if [ "$WS_found" = true ]; then
	  cp $workspace_root/install/adi_tmc_coe_core/lib/adi_tmc_coe_core/adi_ethercat_grant_node /usr/local/bin
	  echo "Executable copied to local binary directory"
	  chmod +s /usr/local/bin/adi_ethercat_grant_node
	  echo "Added sticky bit to executable"
	else
	  echo "Cannot find install directory"
	fi
	
fi
