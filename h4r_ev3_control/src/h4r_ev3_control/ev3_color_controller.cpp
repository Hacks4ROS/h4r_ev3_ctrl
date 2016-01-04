/*
 * This file (ev3_color_controller.cpp) is part of h4r_ev3_control.
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

#include <h4r_ev3_control/ev3_color_controller.h>

namespace ev3_control
{

Ev3ColorController::Ev3ColorController()
:sensor_mode_needs_init_(true)
,mode_(Ev3Strings::EV3COLORMODE_RGB_RAW)
,max_range_(2.0)
,min_range_(0)
,publish_rate_(10)
{
	// TODO Auto-generated constructor stub

}

Ev3ColorController::~Ev3ColorController()
{
	// TODO Auto-generated destructor stub
}


bool Ev3ColorController::init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh)
	{

		// get publishing period
		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set, using 10 Hz");
		}

		if (!ctrl_nh.getParam("port", port_))
		{
			ROS_ERROR("Parameter port was not set");
			return false;
		}

		std::string mode_str;
		if (!ctrl_nh.getParam("mode", mode_str))
		{
			ROS_ERROR("Parameter mode was not set, using 'rgb_raw'");
		}
		else
		{
			if(mode_str=="rgb_raw")
			{
				mode_=Ev3Strings::EV3COLORMODE_RGB_RAW;
			}
			else if(mode_str=="color")
			{
				mode_=Ev3Strings::EV3COLORMODE_COL_COLOR;
			}
			else if(mode_str=="ambient")
			{
				mode_=Ev3Strings::EV3COLORMODE_COL_AMBIENT;
			}
			else if(mode_str=="reflect")
			{
				mode_=Ev3Strings::EV3COLORMODE_COL_REFLECT;
			}
			else
			{
				ROS_ERROR_STREAM("Value for parameter mode unknown, only 'rgb_raw', 'color', 'reflect' and 'ambient' are supported!");
			}
		}



		cout << "Port: " << port_ << endl;
		cout << "Mode: " << mode_str << endl;
		cout << "Publish rate: " << publish_rate_ << endl;



		handle_ = hw->getHandle(port_);
		color_interface_.setSensor(handle_.getSensor());

		//TODO Mode handling

		if(!color_interface_.isConnected())
		{

			ROS_ERROR_STREAM(
					"Need Color sensor on port: "<<port_);
			return false;
		}

		std::string topic_name;
		switch(mode_)
		{
		case Ev3Strings::EV3COLORMODE_COL_AMBIENT:
		case Ev3Strings::EV3COLORMODE_COL_REFLECT:
			if(mode_==Ev3Strings::EV3COLORMODE_COL_AMBIENT)
			{
				topic_name=port_+"_color_ambient";
			}
			else
			{
				topic_name=port_+"_color_reflect";
			}


			if (!ctrl_nh.getParam("topic_name", topic_name))
			{
				ROS_INFO_STREAM("Parameter 'topic_name' not given or wrong type, using "<<topic_name);
			}

			if (!ctrl_nh.getParam("frame_id", frame_id_))
			{
				frame_id_=port_;
				ROS_INFO_STREAM("Parameter frame_id not given or wrong type, using "<<port_);
			}
			realtime_illuminance_publisher_->msg_.header.frame_id=frame_id_;

			realtime_illuminance_publisher_ = RtIlluminancePublisherPtr(
					new realtime_tools::RealtimePublisher<sensor_msgs::Illuminance>(
							root_nh, topic_name, 4));

			break;


		case Ev3Strings::EV3COLORMODE_COL_COLOR:
			topic_name=port_+"_color_number";
			if (!ctrl_nh.getParam("topic_name", topic_name))
			{
				ROS_INFO_STREAM("Parameter 'topic_name' not given or wrong type, using "<<topic_name);
			}
			std::cout<<"Ambient Mode Setup!"<<std::endl;
			realtime_color_number_publisher_ = RtColorNumberPublisherPtr(
					new realtime_tools::RealtimePublisher<std_msgs::UInt8>(
							root_nh, topic_name, 4));
			break;

		case Ev3Strings::EV3COLORMODE_RGB_RAW:
			topic_name=port_+"_color_rgb_raw";
			if (!ctrl_nh.getParam("topic_name", topic_name))
			{
				ROS_INFO_STREAM("Parameter 'topic_name' not given or wrong type, using "<<topic_name);
			}
			std::cout<<"Ambient Mode Setup!"<<std::endl;
			realtime_color_number_publisher_ = RtColorNumberPublisherPtr(
					new realtime_tools::RealtimePublisher<std_msgs::UInt8>(
							root_nh, topic_name, 4));
			break;


		default:
			ROS_ERROR_STREAM("Mode "<<mode_str<<" not supported by this controller!");
			return false;
			break;
		}



		return true;
	}

void Ev3ColorController::starting(const ros::Time& time)
{
	last_publish_time_ = time;
}

void Ev3ColorController::update(const ros::Time& time, const ros::Duration& /*period*/)
	{
		using namespace hardware_interface;

		int value;


		if(!color_interface_.isConnected())
		{
			//ROS_ERROR_STREAM("Lego Subsonic Sensor disconnected for port: "<<port_);
			sensor_mode_needs_init_=true;
			return;
		}

		if(sensor_mode_needs_init_)
		{
			sensor_mode_needs_init_=!(color_interface_.setMode(mode_));
			if(sensor_mode_needs_init_)
			{
				ROS_ERROR("Could not set sensor mode!");
			}
		}

		if (!handle_.getValue(0, value))
		{
			return;
		}

		if (publish_rate_ > 0.0
		    && last_publish_time_ + ros::Duration(1.0 / publish_rate_)< time)
		{
				bool published=false;
				switch(mode_)
				{
				case Ev3Strings::EV3COLORMODE_COL_AMBIENT:
				case Ev3Strings::EV3COLORMODE_COL_REFLECT:
					if (realtime_illuminance_publisher_->trylock())
					{
						realtime_illuminance_publisher_->msg_.header.stamp = time;


					}

					break;


				case Ev3Strings::EV3COLORMODE_COL_COLOR:
				case Ev3Strings::EV3COLORMODE_RGB_RAW:
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

PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3ColorController, controller_interface::ControllerBase)

} /* namespace ev3_control */
