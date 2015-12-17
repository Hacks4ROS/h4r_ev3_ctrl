/*
 * This file (Ev3SensorInterface.h) is part of h4r_ev3_control.
 * Date: 10.12.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ev3_control.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EV3SENSORINTERFACE_H_
#define EV3SENSORINTERFACE_H_

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
#include <h4r_ev3_control/H4REv3Port.h>
using namespace std;

namespace ev3_control
{

class Ev3SensorHandle
{
private:
	std::string port_name_;
public:

	Ev3SensorHandle()
	:port_name_()
	{}

	Ev3SensorHandle(const std::string &port_name)
	:port_name_(port_name)
	{}


	virtual ~Ev3SensorHandle()
	{}

	const std::string& getName() const
	{
		return port_name_;
	}

	bool setMode(const std::string &mode)
	{
		//TODO write mode to file
		cout<<mode<<endl;
		return true;
	}

	bool getValue(unsigned index, int &value)
	{
		//TODO

		//DUMMY
		switch(index)
		{
		case 0:
			value =1.11;
			return true;
		case 1:
			value=2.22;
			return true;

		case 2:
			value=3.33;
			return true;
		}//DUMMY

		return false;
	}


};

/**
 * The Ev3SensorInterface
 */
class Ev3SensorInterface : public hardware_interface::HardwareResourceManager< Ev3SensorHandle > {};

} /* namespace ev3_control */

#endif /* EV3SENSORINTERFACE_H_ */
