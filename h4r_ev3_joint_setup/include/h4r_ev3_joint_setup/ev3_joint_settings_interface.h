/*
 * This file (ev3dev.h) is part of h4r_ev3_joint_setup.
 * Date: 17.11.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_joint_setup is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_joint_setup is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with h4r_ev3_joint_setup.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef ev3dev_H_
#define ev3dev_H_


#include <iostream>
#include <h4r_ev3_joint_setup/ev3_joint_settings.h>
#include <h4r_ev3_joint_setup/ev3_joint_settings_params.h>
#include <h4r_ev3_joint_setup/ev3_joint_settings_exception.h>

#include <ros/duration.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <list>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/resource_manager.h>

#include <ev3dev.h>



namespace ev3dev
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

	 bool checkUpdateSettings(const std::list<ControllerInfo> &start_list, ros::NodeHandle *nh) const
	 {
		 return true;
	 }

	 void updateSettings(const std::list<ControllerInfo> &start_list, ros::NodeHandle *nh)
	 {

			for (std::list<ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); it++)
			{
				for (std::set<std::string>::const_iterator res = it->resources.begin(); res != it->resources.end(); res++)
				{
					ROS_INFO_STREAM(it->type<<" requests Joint "<<it->name<<" "<<*res);

				    try
					{
				    	Ev3JointInterfaceHandle handle=getHandle(*res);
				    	getJointSettings(*res, handle.getSettings());
					}
					catch(const Ev3JointInterfaceException& e)
					{
						ROS_ERROR_STREAM("Did not find handle: "<<*res);
					}
				}
			}
	 }


};



}

#endif /* ev3dev_H_ */
