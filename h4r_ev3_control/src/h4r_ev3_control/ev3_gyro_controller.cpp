/*
 * This file (ev3_gyro_controller.cpp) is part of h4r_ev3_control.
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
#include <h4r_ev3_control/ev3_gyro_controller.h>

namespace ev3_control
{

Ev3GyroController::Ev3GyroController()
:sensor_mode_needs_init_(true)
,mode_(Ev3Strings::EV3GYROMODE_GYRO_ANG)
,publish_rate_(10)
{}

Ev3GyroController::~Ev3GyroController()
{}


bool Ev3GyroController::init(Ev3SensorInterface* hw,
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

		std::string mode_str;
		if (!ctrl_nh.getParam("mode", mode_str))
		{
			ROS_ERROR("Parameter mode was not set, using 'angle'");
		}
		else
		{
			if(mode_str=="rate")
			{
				mode_=Ev3Strings::EV3GYROMODE_GYRO_ANG;
			}
			else if(mode_str=="angle")
			{
				mode_=Ev3Strings::EV3GYROMODE_GYRO_RATE;
			}
			else if(mode_str=="rate&angle")
			{
				mode_=Ev3Strings::EV3GYROMODE_GYRO_G_A;
				ROS_WARN("Setting rate and angle mode! According to documentation of EV3Dev "
						"this mode is instable, when the value of the angle reaches ~32000 it crashes the driver!");
			}
			else
			{
				ROS_ERROR_STREAM("Value for parameter mode unknown supported are: angle, rate" );
			}
		}



		cout << "Port: " << port_ << endl;
		cout << "Mode: " << mode_str << endl;
		cout << "Publish rate: " << publish_rate_ << endl;


		handle_ = hw->getHandle(port_);
		gyro_interface_.setSensor(handle_.getSensor());

		if(!gyro_interface_.isConnected())
		{

			ROS_ERROR_STREAM(
					"Need gyro sensor on port: "<<port_<<"("<<handle_.getDriverName()<<"|"<<Ev3Strings::EV3DRIVERNAME_LEGO_EV3_US<<")");
			return false;
		}


		if (!ctrl_nh.getParam("frame_id", frame_id_))
		{
			frame_id_=port_;
			ROS_INFO_STREAM("Parameter frame_id not given or wrong type, using "<<port_);
		}

		std::string topic_name;
		topic_name=port_+"_gyro";
		if (!ctrl_nh.getParam("topic_name", topic_name))
		{
			ROS_INFO_STREAM("Parameter topic name not given using "<<topic_name);
		}

		//Init orientation to be a legal value for rate mode
		realtime_imu_publisher_->msg_.orientation=tf::createQuaternionMsgFromYaw( 0.0 );


		realtime_imu_publisher_ = RtImuPublisherPtr(
				new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(
						root_nh, topic_name, 4));
		realtime_imu_publisher_->msg_.header.frame_id=frame_id_;

		return true;
	}

void Ev3GyroController::starting(const ros::Time& time)
{
	last_publish_time_ = time;
}

void Ev3GyroController::update(const ros::Time& time, const ros::Duration& /*period*/)
	{
		using namespace hardware_interface;

		int value0,value1;


		if(!gyro_interface_.isConnected())
		{
			sensor_mode_needs_init_=true;
			return;
		}

		if(sensor_mode_needs_init_)
		{
			sensor_mode_needs_init_=!(gyro_interface_.setMode(mode_));
			if(sensor_mode_needs_init_)
			{
				ROS_ERROR("Could not set sensor mode!");
			}
		}

		if (!handle_.getValue(0, value0))return;

		if (mode_==Ev3Strings::EV3GYROMODE_GYRO_G_A)
		{
			if (!handle_.getValue(1, value1))return;
		}

		if (publish_rate_ > 0.0
		    && last_publish_time_ + ros::Duration(1.0 / publish_rate_)< time)
		{
				bool published=false;

				if (realtime_imu_publisher_->trylock())
				{
					realtime_imu_publisher_->msg_.header.stamp = time;

					switch(mode_)
					{

					case Ev3Strings::EV3GYROMODE_GYRO_ANG:
						realtime_imu_publisher_->msg_.orientation=tf::createQuaternionMsgFromYaw( ((double)value0 ) * M_PI/180.0);
						break;

					case Ev3Strings::EV3GYROMODE_GYRO_RATE:
						realtime_imu_publisher_->msg_.angular_velocity.z=( (double) value0 ) * M_PI/180.0;
						break;

					case Ev3Strings::EV3GYROMODE_GYRO_G_A:
						realtime_imu_publisher_->msg_.orientation=tf::createQuaternionMsgFromYaw( ((double) value0 ) * M_PI/180.0);
						realtime_imu_publisher_->msg_.angular_velocity.z=( (double) value1 ) * M_PI/180.0;
						break;

					}

					realtime_imu_publisher_->unlockAndPublish();
					published=true;
				}


				if(published)
				{
					last_publish_time_ = last_publish_time_
							+ ros::Duration(1.0 / publish_rate_);
				}

		}
	}

PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3GyroController, controller_interface::ControllerBase)

} /* namespace ev3_control */
