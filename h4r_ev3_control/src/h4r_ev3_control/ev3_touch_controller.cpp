/*
 * This file (ev3_touch_controller.cpp) is part of h4r_ev3_control.
 * Date: 01.01.2016
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
#include <h4r_ev3_control/ev3_touch_controller.h>

namespace ev3_control
{

Ev3TouchController::Ev3TouchController()
:sensor_mode_needs_init_(true)
,publish_rate_(10)
{}

Ev3TouchController::~Ev3TouchController()
{}


bool Ev3TouchController::init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh)
	{

		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set, using 10 Hz");
		}

		if (!ctrl_nh.getParam("port", port_))
		{
			ROS_ERROR("Parameter port was not set");
			return false;
		}



		cout << "Port: " << port_ << endl;
		cout << "Publish rate: " << publish_rate_ << endl;


		handle_ = hw->getHandle(port_);
		touch_interface_.setSensor(handle_.getSensor());

		if(!touch_interface_.isConnected())
		{

			ROS_ERROR_STREAM(
					"Need touch sensor on port: "<<port_);
			return false;
		}


		if (!ctrl_nh.getParam("frame_id", frame_id_))
		{
			frame_id_=port_;
			ROS_INFO_STREAM("Parameter frame_id not given or wrong type, using "<<port_);
		}

		std::string topic_name;
		topic_name=port_+"_touch";
		if (!ctrl_nh.getParam("topic_name", topic_name))
		{
			ROS_INFO_STREAM("Parameter topic name not given using "<<topic_name);
		}

		realtime_bool_publisher_ = RtBoolPublisherPtr(
				new realtime_tools::RealtimePublisher<std_msgs::Bool>(
						root_nh, topic_name, 4));


		return true;
	}

void Ev3TouchController::starting(const ros::Time& time)
{
	last_publish_time_ = time;
}

void Ev3TouchController::update(const ros::Time& time, const ros::Duration& /*period*/)
	{
		using namespace hardware_interface;

		int value0,value1;


		if(!touch_interface_.isConnected())
		{
			sensor_mode_needs_init_=true;
			return;
		}

		if (!handle_.getValue(0, value0))return;


		if (publish_rate_ > 0.0
		    && last_publish_time_ + ros::Duration(1.0 / publish_rate_)< time)
		{
				bool published=false;

				if (realtime_bool_publisher_->trylock())
				{
					realtime_bool_publisher_->msg_.data=(double) (!!value0);

					realtime_bool_publisher_->unlockAndPublish();
					published=true;
				}


				if(published)
				{
					last_publish_time_ = last_publish_time_
							+ ros::Duration(1.0 / publish_rate_);
				}

		}
	}

PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3TouchController, controller_interface::ControllerBase)

} /* namespace ev3_control */
