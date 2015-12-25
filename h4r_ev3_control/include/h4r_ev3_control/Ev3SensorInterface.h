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
#include <boost/shared_ptr.hpp>
using namespace std;

namespace ev3_control
{

class Ev3SensorHandle
{
private:
	std::string port_name_;
	H4REv3Sensor *sensor_;
public:

	Ev3SensorHandle()
	:port_name_()
	,sensor_(0)
	{}

	Ev3SensorHandle(const std::string &port_name, H4REv3Sensor *sensor)
	:port_name_(port_name)
	,sensor_(sensor)
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
		cout<<"SETMODE_:"<<mode<<endl;
		return false;
	}

	bool getValue(unsigned index, int &value)
	{
		return sensor_->value(index,value);
	}

	H4REv3Sensor *getSensor()
	{
		return sensor_;
	}

	Ev3Strings::Ev3DriverName getDriverName()
	{
		Ev3Strings::Ev3DriverName val;
		if(sensor_->getDriverName(val))
		{
			return val;
		}
		else
		{
			return Ev3Strings::EV3DRIVERNAME_NOT_FOUND;
		}
	}


};

/**
 * The Ev3SensorInterface
 */
class Ev3SensorInterface : public hardware_interface::HardwareResourceManager< Ev3SensorHandle > {};

} /* namespace ev3_control */

#endif /* EV3SENSORINTERFACE_H_ */
