/*
 * This file (Ev3HardwareInterface.cpp}) is part of h4r_ev3_ctrl.
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
Ev3HardwareInterface::Ev3HardwareInterface(const std::vector<ev3dev::port_type> &out_ports)
{

	registerInterface(&jnt_eff_interface);
	registerInterface(&jnt_pos_interface);
	registerInterface(&jnt_vel_interface);
	registerInterface(&jnt_state_interface);

	for (int p = 0; p < out_ports.size(); ++p)
	{
		//int p=0;
		//Create new out port data - pointer because of vector reallocations...
		out_data_.push_back(new OutPortData(out_ports[p]));

		//Create jointname with letter
		string joint_name;
		joint_name+="Joint_";
		joint_name+='A'+p;

		//Register motor joints
		hardware_interface::JointStateHandle state_handle(joint_name, &out_data_[p]->position_out, &out_data_[p]->velocity_out, &out_data_[p]->effort_out);
		jnt_state_interface.registerHandle(state_handle);

		hardware_interface::JointHandle joint_handle(jnt_state_interface.getHandle(joint_name), &out_data_[p]->command);


		jnt_pos_interface.registerHandle(joint_handle);
		jnt_vel_interface.registerHandle(joint_handle);
		jnt_eff_interface.registerHandle(joint_handle);
	}
}



Ev3HardwareInterface::~Ev3HardwareInterface()
{
	for (int i = 0; i < out_data_.size(); ++i)
	{
		delete out_data_[i];
	}
	out_data_.clear();
}

void Ev3HardwareInterface::write()
{
	for (int i = 0; i < out_data_.size(); ++i)
	{

	}
}

void Ev3HardwareInterface::read()
{
	for (int i = 0; i < out_data_.size(); ++i)
	{

	}
}


bool Ev3HardwareInterface::canSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> &stop_list) const
{




	cout<<"CAN";
	cout<<"Start:"<<endl;
	for (std::list<ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); it++)
	{
		std::cout << it->name<<" "<<it->type <<" "<< endl;
		for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
		{
			std::cout<<"\t"<<*res<<endl;
		}
	}


	cout<<"STOP:"<<endl;
	for (std::list<ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); it++)
			{
				std::cout << it->name<<" "<<it->type <<" "<< endl;
				for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
				{
					std::cout<<"\t"<<*res<<endl;
				}
			}


	return true;
}

void Ev3HardwareInterface::doSwitch(const std::list<ControllerInfo> &start_list, const std::list<ControllerInfo> &stop_list)
{

	cout<<"DO";
	cout<<"Start:"<<endl;
	for (std::list<ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); it++)
	{
		std::cout << it->name<<" "<<it->type <<" "<< endl;
		for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
		{
			std::cout<<"\t"<<*res<<endl;
		}
	}


	cout<<"STOP:"<<endl;
	for (std::list<ControllerInfo>::const_iterator it = stop_list.begin(); it != stop_list.end(); it++)
	{
		std::cout << it->name<<" "<<it->type <<" "<< endl;
		for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
		{
			std::cout<<"\t"<<*res<<endl;
		}
	}

}

} /* namespace h4r_ev3_ctrl */
