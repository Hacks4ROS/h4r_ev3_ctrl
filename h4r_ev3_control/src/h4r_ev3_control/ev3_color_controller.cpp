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
	    /**
	     *
	     * \page Ev3ColorController Ev3ColorController
	     * \section Parameters
	     * \subsection publish_rate
	     *
	     * The publish_rate parameter sets the rate which the controller should use to publish its
	     * messages.
	     */
		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set, using 10 Hz");
		}


		/**
		* \page Ev3ColorController Ev3ColorController
		* \subsection port
		*
		*
		* The port parameter specifies the EV3 port name.
		*/
		if (!ctrl_nh.getParam("port", port_))
		{
			ROS_ERROR("Parameter port was not set");
			return false;
		}

		std::string mode_str;


		/**
		* \page Ev3ColorController Ev3ColorController
		* \subsection mode
		*
		* The mode parameter gives the mode for the color sensor. If not set, it will use **rgb_raw**.
		*
		* The following modes are possible for the sensor:
		* 1. **rgb_raw**\n
		*    'rgb_raw' mode uses the RGB_RAW mode of this sensor.
		*    It will output the color information in RGB in a **std_msgs::ColorRGBA.msg** topic.
		* 2. **color**\n
		*    'color' mode publishes the a number for the following recognized colors into a **std_msgs::UInt8** topic\n
		*    Value | Color
		*    ------|------
		*        0 | none
		* 	      1 | black
		*        2 | blue
		*        3 | green
		*        4 | yellow
		*        5 | red
		*        6 | white
		*        7 | brown
		*
		* 3. **ambient**\n
		*	   'ambient' mode measures the ambient light and publishes into a **sensor_msgs::Illuminance** topic
		*
		* 4. **reflect**\n
		*     'reflect' mode measures the reflected light from an obstacle and publishes into a **sensor_msgs::Illuminance** topic
		*
		*  \warning Currently **ambient** and **reflect** publish the direct sensor value (percent) in percent - this will change so that it fits the output value in the message
		*  or the message type will be changed when it is found to be not right.
		*	\todo check how to calculate LUX value for message or change message type
		*/
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

			realtime_illuminance_publisher_ = RtIlluminancePublisherPtr(
					new realtime_tools::RealtimePublisher<sensor_msgs::Illuminance>(
							root_nh, topic_name, 4));
			realtime_illuminance_publisher_->msg_.header.frame_id=frame_id_;

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
			realtime_color_publisher_ = RtColorPublisherPtr(
					new realtime_tools::RealtimePublisher<std_msgs::ColorRGBA>(
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

		int value0,value1,value2;



		if(!color_interface_.isConnected())
		{
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

		if (!handle_.getValue(0, value0))
		{
			return;
		}

		if(mode_==Ev3Strings::EV3COLORMODE_RGB_RAW)
		{
			if (!handle_.getValue(1, value1))
			{
				return;
			}
			if (!handle_.getValue(2, value2))
			{
				return;
			}
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
						realtime_illuminance_publisher_->msg_.illuminance=value0;
						realtime_illuminance_publisher_->unlockAndPublish();
						published=true;
					}
					break;


				case Ev3Strings::EV3COLORMODE_COL_COLOR:
				{
					realtime_color_number_publisher_->msg_.data=value0;
					realtime_color_number_publisher_->unlockAndPublish();
					published=true;
				}
				break;

				case Ev3Strings::EV3COLORMODE_RGB_RAW:
					if (realtime_color_publisher_->trylock())
					{
						realtime_color_publisher_->msg_.r=value0;
						realtime_color_publisher_->msg_.g=value1;
						realtime_color_publisher_->msg_.b=value2;
						realtime_color_publisher_->unlockAndPublish();
						published=true;
					}

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
