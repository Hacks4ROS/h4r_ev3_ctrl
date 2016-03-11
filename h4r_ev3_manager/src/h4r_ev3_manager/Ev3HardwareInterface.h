/*
 * This file (Ev3HardwareInterface.h) is part of h4r_ev3_manager.
 * Date: 19.11.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with h4r_ev3_joint_settings.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/controller_info.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <control_toolbox/pid.h>
#include <list>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <h4r_ev3_control/Ev3SensorInterface.h>
#include <h4r_ev3_joint_setup/ev3_joint_settings_interface.h>
#include <h4r_ev3_control/H4REv3Port.h>

#ifndef EV3HARDWAREINTERFACE_H_
#define EV3HARDWAREINTERFACE_H_

using namespace std;
using namespace hardware_interface;

namespace ev3_control {

class Ev3HardwareInterface : public hardware_interface::RobotHW
{
	ros::NodeHandle *nh_;

	std::vector<Ev3JointSettings*> joint_settings_;
	std::vector<ev3_control::H4REv3Sensor*> ev3_sensors_;


	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::EffortJointInterface jnt_eff_interface;
	ev3_control::Ev3SensorInterface sensor_interface;

	joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface;
	Ev3JointInterface jnt_ev3_joint_interface;

public:
	Ev3HardwareInterface(
			const std::vector<std::string> &in_ports,
			const std::vector<std::string> &out_ports
			);
	virtual ~Ev3HardwareInterface();

	void write(const ros::Duration &d);
	void read();

	bool canSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> &stop_list) const;
	void doSwitch(const std::list<ControllerInfo> &start_list,  const std::list<ControllerInfo> &stop_list);
};

} /* namespace ev3_control */

#endif /* EV3HARDWAREINTERFACE_H_ */
