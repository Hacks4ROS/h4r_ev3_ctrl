/*
 * This file (ev3_joint_settings_interface.h) is part of h4r_ev3_manager.
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
 * along with h4r_ev3_joint_setting.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EV3_JOINT_SETTINGS_INTERFACE_H
#define EV3_JOINT_SETTINGS_INTERFACE_H


#include <iostream>
#include <ros/duration.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <list>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/resource_manager.h>

#include <h4r_ev3_joint_setup/ev3_joint_settings.h>
#include <h4r_ev3_joint_setup/ev3_joint_settings_exception.h>
#include <h4r_ev3_joint_setup/ev3_joint_settings_params.h>



namespace ev3_control
{
using namespace hardware_interface;
using namespace std;


class Ev3JointInterfaceHandle
{
public:



	Ev3JointInterfaceHandle()
	:settings_(0){}

	Ev3JointInterfaceHandle(
			const JointHandle& jh,
			Ev3JointSettings *settings)
	:jh_(jh)
	,settings_(settings)
	{}

	std::string getName() const
	{
		return jh_.getName();
	}

	Ev3JointSettings &getSettings()
	{
		return *settings_;
	}


private:


	Ev3JointSettings *settings_;
	JointHandle jh_;
};

class Ev3JointInterface : public ResourceManager<Ev3JointInterfaceHandle>
{
public:
	Ev3JointInterfaceHandle getHandle(const std::string& name)
	  {
	    // Rethrow exception with a meaningful type
	    try
	    {
	      return this->ResourceManager<Ev3JointInterfaceHandle>::getHandle(name);
	    }
	    catch(const std::logic_error& e)
	    {
	      throw Ev3JointInterfaceException(e.what());
	    }
	  }

	 bool ControllerTestChange(const std::list<ControllerInfo> &start_list) const
	 {
			return true;
	 }


	 bool ControllerChange(const std::list<ControllerInfo> &start_list)
	 {
		 ROS_INFO("Controller Change");
		for (std::list<ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); it++)
		{
			for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
			{
				ROS_INFO_STREAM(it->type<<" requests Joint "<<it->name<<" "<<*res);

				try
				{
					Ev3JointInterfaceHandle handle=getHandle(*res);
					Ev3JointSettings::Ev3HwSettings ev3settings;

					handle.getSettings().port.setMotorCommand(Ev3Strings::EV3MOTORCOMMANDS_RESET);
					getJointSettings(*res, ev3settings);
					ROS_INFO("----------------------------------------------<");
					handle.getSettings().load(ev3settings,true);
				}
				catch(const Ev3JointInterfaceException& e)
				{
					ROS_ERROR_STREAM("Did not find handle: "<<*res);
				}
			}
		}
		return true;
	 }


};



}

#endif /* EV3_JOINT_SETTINGS_INTERFACE_H */
