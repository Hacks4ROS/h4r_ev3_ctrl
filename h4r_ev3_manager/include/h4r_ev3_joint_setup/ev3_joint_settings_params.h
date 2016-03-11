/*
 * This file (ev3_joint_params.h) is part of h4r_ev3_manager.
 * Date: 16.11.2015
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
 * along with h4r_outport_setup.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EV3_JOINT_SETTINGS_PARAMS_H_
#define EV3_JOINT_SETTINGS_PARAMS_H_

#include <string>
#include <ros/ros.h>

#include <h4r_ev3_joint_setup/ev3_joint_settings.h>


namespace ev3_control
{

	inline bool getJointSettings(const std::string& joint_name,
			                     Ev3JointSettings::Ev3HwSettings& settings)
	{




		  ros::NodeHandle setup_nh;
		  const std::string joint_setup = "Ev3devJoints/"+joint_name;
		  try
		  {
			if (!setup_nh.hasParam(joint_setup))
			{
			      ROS_WARN_STREAM("No ev3 port settings found for ev3 joint '" << joint_name <<" using velocity mode and automatic port mode");
			      return false;
			}
		    setup_nh = ros::NodeHandle(setup_nh, joint_setup);
		  }
		  catch (const ros::InvalidNameException& e)
		  {
			    ROS_ERROR_STREAM(e.what());
			    return false;
		  }

		ROS_INFO_STREAM("<--------------EV3 Joint "<<joint_name<<"---------------->");


		std::string mode;
		if(setup_nh.getParam("mode", mode))
		{
			if(mode=="position")
			{
				settings.joint_mode=Ev3JointSettings::EV3_JOINT_POSITION;
			}
			else if(mode=="velocity")
			{
				settings.joint_mode=Ev3JointSettings::EV3_JOINT_VELOCITY;
			}
			else
			{
				ROS_WARN_STREAM("Unknown Mode for Ev3 Joint: "<<mode<<" expecting velocity");
				settings.joint_mode=Ev3JointSettings::EV3_JOINT_VELOCITY;
			}
			ROS_INFO_STREAM("Joint control mode: "<<mode);
		}

		// Driver Name
		if(!setup_nh.getParam("driver_name", settings.driver_name))
		{
			settings.driver_name="";
		}
		else
		{
			ROS_INFO_STREAM("Driver_name: "<<settings.driver_name);
		}

		//PID
		settings.pid.clear();
		if(!setup_nh.getParam("speed_pid", settings.pid))
		{
			settings.pid.resize(3);
			ROS_INFO("No PID Settings found, using standard  Kp:1000 Ki:60, Kd:0");
			settings.pid[0]=1000;
			settings.pid[1]=60;
			settings.pid[2]=0;
		}
		else
		{
			if(settings.pid.size()!=3)
			{
				ROS_ERROR("PID Settings error! Wrong amount of settings, required exactly 3 for P, I and D");
				return false;
			}
			else
			{
				ROS_INFO_STREAM("P: "<<settings.pid[0]<<"  I: "<<settings.pid[1]<<"   D: "<<settings.pid[2]);
			}
		}
		//PID
		ROS_INFO_STREAM("</-------------EV3 Joint "<<joint_name<<"---------------->");

		return true;
	}
}


#endif /* EV3_JOINT_SETTINGS_PARAMS_H_ */
