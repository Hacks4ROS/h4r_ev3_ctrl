/*
 * This file (ev3_joint_settings.h) is part of h4r_ev3_ctrl.
 * Date: 16.11.2015
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

#ifndef EV3_JOINT_SETTINGS_H_
#define EV3_JOINT_SETTINGS_H_

#include <h4r_ev3_ctrl/H4REv3Port.h>

namespace h4r_ev3_ctrl {
class Ev3JointSettings
{

public:

	typedef enum
	{
		EV3_JOINT_OFF,
		EV3_JOINT_POSITION,
		EV3_JOINT_VELOCITY,
	} Ev3JointMode;

	class Ev3HwSettings
 	{
	public:
		Ev3HwSettings()
		:joint_mode(EV3_JOINT_VELOCITY)
		{}

		Ev3JointMode joint_mode;
		std::string driver_name;
		std::vector<double> pid;
	};


	H4REv3Motor port;
	Ev3HwSettings ev3settings;

	double command;
	double last_command;
	double velocity_out;
	double position_out;
	double effort_out; //not supported by ev3 afaik but needed for joint state handle

	Ev3JointSettings(const std::string &out_port)
	:port(out_port)
	,command(0)
	,last_command(0)
	,velocity_out(0)
	,position_out(0)
	,effort_out(0)
	{}


	bool load(const Ev3HwSettings &settings, bool testOnly)
	{
		ROS_INFO("Loading controller...");
		return true;
	}

	bool write()
	{
			ROS_INFO_STREAM("Command: "<<command);
			switch(ev3settings.joint_mode)
			{
			case Ev3JointSettings::EV3_JOINT_POSITION:

				if
				(
						port.setDutyCycleSP(100)+
						port.setPositionSP(command)+
						port.setSpeedRegulation(Ev3Strings::EV3SWITCH_OFF)+
						port.setMotorCommand(Ev3Strings::EV3MOTORCOMMANDS_RUN_TO_ABS_POS)
						!=4
				)
				{
					return false;
				}

				break;

			case Ev3JointSettings::EV3_JOINT_VELOCITY:
				if
				(
				port.setDutyCycleSP(100)+
				port.setSpeedSP(command)+
				port.setSpeedRegulation(Ev3Strings::EV3SWITCH_ON)+
				port.setMotorCommand(Ev3Strings::EV3MOTORCOMMANDS_RUN_FOREVER)
					!= 4
				)
				return false;

				break;

			default:
				break;
			}

			return true;
	}

	bool read()
	{
		return
		(
				port.position(position_out)+
				port.speed(velocity_out)
		)==2;
	}

};

}

#endif /* EV3_JOINT_SETTINGS_H_ */
