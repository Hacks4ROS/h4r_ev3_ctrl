/*
 * This file (Ev3HardwareInterface.cpp) is part of h4r_ev3_ctrl.
 * Date: 16.11.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_ctrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_ctrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with h4r_ev3_ctrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "Ev3HardwareInterface.h"

namespace h4r_ev3_ctrl {
Ev3HardwareInterface::Ev3HardwareInterface(
		const std::vector<ev3dev::port_type> &in_ports,
		const std::vector<ev3dev::port_type> &out_ports
		)
{
	registerInterface(&jnt_eff_interface);
	registerInterface(&jnt_pos_interface);
	registerInterface(&jnt_vel_interface);
	registerInterface(&jnt_state_interface);

	for (int p = 0; p < out_ports.size(); ++p)
	{
		//int p=0;
		//Create new out port data - pointer because of vector reallocations...
		joint_settings_.push_back(new Ev3JointSettings(out_ports[p]));

		//Create jointname with letter
		string joint_name;
		joint_name+="Joint_";
		joint_name+='A'+p;

		//Register motor joints
		hardware_interface::JointStateHandle state_handle(joint_name, &joint_settings_[p]->position_out, &joint_settings_[p]->velocity_out, &joint_settings_[p]->effort_out);
		jnt_state_interface.registerHandle(state_handle);

		hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(joint_name), &joint_settings_[p]->command);
		jnt_pos_interface.registerHandle(joint_handle);
		jnt_vel_interface.registerHandle(joint_handle);
		jnt_eff_interface.registerHandle(joint_handle);


		//Joint limits
		joint_limits_interface::JointLimits limits;
		limits.has_velocity_limits=true;
		limits.max_velocity=0.1;

		joint_limits_interface::SoftJointLimits soft_limits;
		joint_limits_interface::PositionJointSoftLimitsHandle pos_limit_handle(jnt_pos_interface.getHandle(joint_name), limits, soft_limits);
		jnt_limits_interface.registerHandle(pos_limit_handle);

		Ev3JointInterfaceHandle ev3_joint_handle(joint_handle,joint_settings_[p]);
		jnt_ev3_joint_interface.registerHandle(ev3_joint_handle);
	}
}


Ev3HardwareInterface::~Ev3HardwareInterface()
{
	for (int i = 0; i < joint_settings_.size(); ++i)
	{
		delete joint_settings_[i];
	}
	joint_settings_.clear();
}

void Ev3HardwareInterface::write(const ros::Duration &d)
{
    //jnt_limits_interface.enforceLimits(d);
	for (int i = 0; i < joint_settings_.size(); ++i)
	{
		joint_settings_[i]->write();


	}
}

void Ev3HardwareInterface::read()
{

	for (int i = 0; i < joint_settings_.size(); ++i)
	{
		joint_settings_[i]->read();
	}
}


bool Ev3HardwareInterface::canSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> &stop_list) const
{
	return jnt_ev3_joint_interface.ControllerTestChange(start_list);
}

void Ev3HardwareInterface::doSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> &stop_list)
{
	jnt_ev3_joint_interface.ControllerChange(start_list);
}

} /* namespace h4r_ev3_ctrl */
