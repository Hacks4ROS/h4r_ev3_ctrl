/*
 * This file (ev3_joint_settings.h) is part of h4r_ev3_joint_setup.
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

namespace ev3dev {
class Ev3JointSettings {

public:
	typedef enum
	{
		EV3_JOINT_POSITION,
		EV3_JOINT_VELOCITY,
	} Ev3JointMode;

	Ev3JointMode joint_mode;
	std::string driver_name;
	std::vector<double> pid;

	Ev3JointSettings()
	:joint_mode(EV3_JOINT_VELOCITY)
	{
		//TODO add standard for pid
	}
};

}

#endif /* EV3_JOINT_SETTINGS_H_ */
