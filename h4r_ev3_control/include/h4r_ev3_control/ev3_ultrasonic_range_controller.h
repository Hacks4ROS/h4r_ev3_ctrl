/*
 * This file (Ev3UltraSonicController.h) is part of h4r_ev3_control.
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
#ifndef EV3ULTRASONICCONTROLLER_H_
#define EV3ULTRASONICCONTROLLER_H_

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

#include <h4r_ev3_control/Ev3SensorInterface.h>

namespace ev3_control
{

class Ev3UltraSonicRangeController: public controller_interface::Controller<
		Ev3SensorInterface>
{
private:
	std::string port_;
	Ev3Strings::Ev3UltrasonicMode mode_;
	Ev3SensorHandle handle_;
	H4REv3UltraSonicSensorSpecIface us_interface_;
	bool sensor_mode_needs_init_;

	//Range Publisher
	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<sensor_msgs::Range> > RtRangePublisherPtr;
	RtRangePublisherPtr realtime_range_publisher_;

	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<std_msgs::Bool> > RtBoolPublisherPtr;
	RtBoolPublisherPtr realtime_bool_publisher_;


	ros::Time last_range_publish_time_;
	double publish_rate_;

public:
	Ev3UltraSonicRangeController();
	virtual ~Ev3UltraSonicRangeController();

	virtual bool init(Ev3SensorInterface* hw, ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh)
	{

		// get publishing period
		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set");
			return false;
		}

		if (!ctrl_nh.getParam("port", port_))
		{
			ROS_ERROR("Parameter port was not set");
			return false;
		}

		std::string mode_str;
		if (!ctrl_nh.getParam("mode", mode_str))
		{
			ROS_ERROR("Parameter mode was not set");
			return false;
		}


		mode_=Ev3Strings::ev3_ultrasonic_mode_conv[mode_str.c_str()];
		if(mode_<0)
		{
			ROS_ERROR_STREAM("Mode "<<mode_str<<" not found!");
			return false;
		}

		cout << "Port: " << port_ << endl;
		cout << "Mode: " << mode_str << endl;
		cout << "Publish rate: " << publish_rate_ << endl;



		handle_ = hw->getHandle(port_);
		us_interface_.setSensor(handle_.getSensor());

		//TODO Mode handling

		if(!us_interface_.isConnected())
		{

			ROS_ERROR_STREAM(
					"Need Ultrasonic Sensor on port: "<<port_<<"("<<handle_.getDriverName()<<"|"<<Ev3Strings::EV3DRIVERNAME_LEGO_EV3_US<<")");
			return false;
		}


		switch(mode_)
		{
		case Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM:
		case Ev3Strings::EV3ULTRASONICMODE_US_DC_CM:
			//Create publisher for Range
			std::cout<<"Range Mode Setup!"<<std::endl;
			realtime_range_publisher_ = RtRangePublisherPtr(
					new realtime_tools::RealtimePublisher<sensor_msgs::Range>(
							root_nh, port_+"_us_range", 4));
			break;

		case Ev3Strings::EV3ULTRASONICMODE_US_LISTEN:
			std::cout<<"Listen Mode Setup!"<<std::endl;
			//Create publisher for Listen
			realtime_bool_publisher_ = RtBoolPublisherPtr(
					new realtime_tools::RealtimePublisher<std_msgs::Bool>(
							root_nh, port_+"_us_listen", 4));
			break;

		default:
			ROS_ERROR_STREAM("Mode "<<mode_str<<" not supported by this controller!");
			return false;
			break;
		}



		return true;
	}

	virtual void starting(const ros::Time& time)
	{
		last_range_publish_time_ = time;
	}

	virtual void update(const ros::Time& time, const ros::Duration& /*period*/)
	{
		using namespace hardware_interface;

		int value;


		if(!us_interface_.isConnected())
		{
			//ROS_ERROR_STREAM("Lego Subsonic Sensor disconnected for port: "<<port_);
			sensor_mode_needs_init_=true;
			return;
		}

		if(sensor_mode_needs_init_)
		{
			sensor_mode_needs_init_=!(us_interface_.setMode(mode_));
			if(sensor_mode_needs_init_)
			{
				ROS_ERROR("Could not set sensor mode!");
			}
		}

		if (!handle_.getValue(0, value))
		{
			//ROS_ERROR("Could not get sensor value!");
			return;
		}

		if (publish_rate_ > 0.0
		    && last_range_publish_time_ + ros::Duration(1.0 / publish_rate_)< time)
		{
				bool published=false;
				switch(mode_)
				{
				case Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM:
				case Ev3Strings::EV3ULTRASONICMODE_US_DC_CM:
					if (realtime_range_publisher_->trylock())
					{
						realtime_range_publisher_->msg_.header.stamp = time;
						realtime_range_publisher_->msg_.range = ((double) value)
								/ 1000.0;
						realtime_range_publisher_->unlockAndPublish();
						published=true;
					}
					break;

				case Ev3Strings::EV3ULTRASONICMODE_US_LISTEN:
					if (realtime_bool_publisher_->trylock())
					{
						realtime_bool_publisher_->msg_.data=(bool)value;
						realtime_bool_publisher_->unlockAndPublish();
						published=true;
					}
					break;

				default:
					break;
				}


				if(published)
				{
					last_range_publish_time_ = last_range_publish_time_
							+ ros::Duration(1.0 / publish_rate_);
				}

		}
	}

	virtual void stopping(const ros::Time& /*time*/)
	{

	}

};

} /* namespace ev3_control */

#endif /* EV3ULTRASONICCONTROLLER_H_ */
