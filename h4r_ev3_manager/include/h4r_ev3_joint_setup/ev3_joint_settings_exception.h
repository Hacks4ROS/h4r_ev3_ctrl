/*
 * This file (ev3_joint_settings_exception.h) is part of h4r_ev3_manager.
 * Date: 17.11.2015
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
 * along with ev3_joint_setup.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EV3_JOINT_SETTINGS_EXCEPTION_H_
#define EV3_JOINT_SETTINGS_EXCEPTION_H_

#include <string>
#include <exception>


namespace ev3_control
{

class Ev3JointInterfaceException: public std::exception
{
public:
	Ev3JointInterfaceException(const std::string& msg)
    : msg(msg) {}

  virtual ~Ev3JointInterfaceException() throw()
		  {}

  virtual const char* what() const throw()
  {
    return msg.c_str();
  }

private:
  std::string msg;
};

}

#endif /* EV3_JOINT_SETTINGS_EXCEPTION_H_ */
