/*
 * This file (ev3_ultrasonic_controller.cpp) is part of h4r_ev3_control.
 * Date: 17.12.2015
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
 * along with h4r_ev3_control.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <h4r_ev3_control/ev3_ultrasonic_controller.h>

namespace ev3_control
{

Ev3UltrasonicController::Ev3UltrasonicController()
:sensor_mode_needs_init_(true)
,mode_(Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM)
,max_range_(2.0)
,min_range_(0)
,publish_rate_(10)
{
	// TODO Auto-generated constructor stub

}

Ev3UltrasonicController::~Ev3UltrasonicController()
{
	// TODO Auto-generated destructor stub
}


bool Ev3UltrasonicController::init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh)
	{

		/**
		 *
		 * \page Ev3UltrasonicController Ev3UltrasonicController
		 * \section Parameters
		 * \subsection publish_rate
		 *
		 * The rate the controller should use to publish its messages.
		 */
		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set, using 10 Hz");
		}

		/**
		 * \page Ev3UltrasonicController Ev3UltrasonicController
		 * \subsection port
		 *
		 *
		 * Specifies the EV3 port name.
		 */
		if (!ctrl_nh.getParam("port", port_))
		{
			ROS_ERROR("Parameter port was not set");
			return false;
		}

		std::string mode_str;
		/**
		 * \page Ev3UltrasonicController Ev3UltrasonicController
		 * \subsection mode
		 * 1. **distance** publishes the measured distance into a **sensor_msgs::Range** topic
		 * 2. **listen** publishes into a **std_msgs::Boolean** topic, when it detects
		 *    another Ultrasonic Sensor or a loud noise (like a clap) the value is true otherwise false.
		 *    - **max_range** sets the maximum range of values taken into account if a value is bigger it will be replaced by infinity.\n
		 *      The maximum value for this parameter is 2.5 because the value 2.550 is the sensors error condition.
		 *
		 *    - **min_range** sets the minimum range of values taken into account if a value is smaller it will be replaced by negative infinity.
		 */
		if (!ctrl_nh.getParam("mode", mode_str))
		{
			ROS_ERROR("Parameter mode was not set, using 'distance'");
			mode_=Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM;
		}
		else
		{
			if(mode_str=="distance")
			{
				mode_=Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM;
			}
			else if(mode_str=="listen")
			{
				mode_=Ev3Strings::EV3ULTRASONICMODE_US_LISTEN;
			}
			else
			{
				ROS_ERROR_STREAM("Value for parameter mode unknown, only 'distance' and 'seek' are supported!");
			}
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

		std::string topic_name;
		switch(mode_)
		{
		case Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM:
		case Ev3Strings::EV3ULTRASONICMODE_US_DC_CM:

			topic_name=port_+"_us_range";
			if (!ctrl_nh.getParam("topic_name", topic_name))
			{
				ROS_INFO_STREAM("Parameter topic name not given using"<<topic_name);
			}

			//Create publisher for Range
			std::cout<<"Range Mode Setup!"<<std::endl;
			realtime_range_publisher_ = RtRangePublisherPtr(
					new realtime_tools::RealtimePublisher<sensor_msgs::Range>(
							root_nh, topic_name, 4));




			if (!ctrl_nh.getParam("max_range", max_range_))
			{
				ROS_INFO_STREAM("Parameter max_range not given, using 2.0");
			}

			if (!ctrl_nh.getParam("min_range", min_range_))
			{
				ROS_INFO_STREAM("Parameter min_range not given or wrong type, using 0");
			}




			if (!ctrl_nh.getParam("frame_id", frame_id_))
			{
				frame_id_=port_;
				ROS_INFO_STREAM("Parameter frame_id not given or wrong type, using "<<port_);
			}

			if(max_range_>2.50)
			{
				ROS_ERROR("Parameter max_range to big! 2.550 is error condition of the sensor using 2.5");
				max_range_=2.5;
			}

			realtime_range_publisher_->msg_.min_range=min_range_;
			realtime_range_publisher_->msg_.max_range=max_range_;
			realtime_range_publisher_->msg_.radiation_type=0; //ULTRASONIC
			realtime_range_publisher_->msg_.header.frame_id=frame_id_;
			realtime_range_publisher_->msg_.field_of_view=0.872665; //50deg (in rads)
			break;

		case Ev3Strings::EV3ULTRASONICMODE_US_LISTEN:
			topic_name=port_+"_us_listen";
			if (!ctrl_nh.getParam("topic_name", topic_name))
			{
				ROS_INFO_STREAM("Parameter topic name not given using"<<topic_name);
			}

			std::cout<<"Listen Mode Setup!"<<std::endl;
			//Create publisher for Listen
			realtime_bool_publisher_ = RtBoolPublisherPtr(
					new realtime_tools::RealtimePublisher<std_msgs::Bool>(
							root_nh, topic_name, 4));
			break;

		default:
			ROS_ERROR_STREAM("Mode "<<mode_str<<" not supported by this controller!");
			return false;
			break;
		}



		return true;
	}

void Ev3UltrasonicController::starting(const ros::Time& time)
{
	last_publish_time_ = time;
}

void Ev3UltrasonicController::update(const ros::Time& time, const ros::Duration& /*period*/)
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
		    && last_publish_time_ + ros::Duration(1.0 / publish_rate_)< time)
		{
				bool published=false;
				switch(mode_)
				{
				case Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM:
				case Ev3Strings::EV3ULTRASONICMODE_US_DC_CM:
					if (realtime_range_publisher_->trylock())
					{
						realtime_range_publisher_->msg_.header.stamp = time;

						if(value!=2550)
						{
							double dval = ((double) value) / 1000.0; //to meters

							if(dval < min_range_)
							{
								dval=-std::numeric_limits<double>::infinity();
							}
							else if(dval > max_range_)
							{
								dval=std::numeric_limits<double>::infinity();
							}

							realtime_range_publisher_->msg_.range=dval;
						}
						else
						{
							//value==2550 means no response (either covered sensor or too far away)
							realtime_range_publisher_->msg_.range = std::numeric_limits<double>::infinity();
						}

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
					last_publish_time_ = last_publish_time_
							+ ros::Duration(1.0 / publish_rate_);
				}

		}
	}

PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3UltrasonicController, controller_interface::ControllerBase)

} /* namespace ev3_control */
