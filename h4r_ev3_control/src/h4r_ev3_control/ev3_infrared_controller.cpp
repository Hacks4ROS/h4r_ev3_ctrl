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

#include <h4r_ev3_control/ev3_infrared_controller.h>

namespace ev3_control
{

Ev3InfraredController::Ev3InfraredController()
:sensor_mode_needs_init_(true)
,mode_(Ev3Strings::EV3INFRAREDMODE_IR_PROX)
,max_range_(2.0)
,min_range_(0)
,publish_rate_(10)
{
	// TODO Auto-generated constructor stub

}

Ev3InfraredController::~Ev3InfraredController()
{
	// TODO Auto-generated destructor stub
}


bool Ev3InfraredController::init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh)

	{

		for (int i = 0; i < 4; ++i) {
			first_time_[i]=true;
		}

	    /**
	     *
	     * \page Ev3InfraredController Ev3InfraredController
	     * \section Parameters
	     * \subsection publish_rate
	     *
	     * The rate which the controller should use to publish its messages.
	     */
		if (!ctrl_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter publish_rate was not set, using 10 Hz");
		}


		/**
		 * \page Ev3InfraredController Ev3InfraredController
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
		/**
		 * \page Ev3InfraredController Ev3InfraredController
		 * \subsection mode
		 * 1. **proximity** publishes the distance of the sensor to a reflective surface.
		 *
		 * 2. **seek** publishes four topics containing the heading and the distance
		 * value to ir-remotes in beacon mode for each remote channel
		 *
		 * 3. **remote** publishes the four remote channels of the ir-sensor
		 * to four different topics from type **sensor_msgs::Joy**
		 *
		 *
		 */
		std::string mode_str;
		if (!ctrl_nh.getParam("mode", mode_str))
		{
			ROS_ERROR("Parameter mode was not set, using 'proximity'");
		}
		else
		{
			if(mode_str=="proximity")
			{
				mode_=Ev3Strings::EV3INFRAREDMODE_IR_PROX;
			}
			else if(mode_str=="seek")
			{
				mode_=Ev3Strings::EV3INFRAREDMODE_IR_SEEK;
			}
			else if(mode_str=="remote")
			{
				mode_=Ev3Strings::EV3INFRAREDMODE_IR_REMOTE;
			}
			else
			{
				ROS_ERROR_STREAM("Value for parameter mode unknown, only 'proximity' and 'seek' are supported!");
			}
		}



		cout << "Port: " << port_ << endl;
		cout << "Mode: " << mode_str << endl;
		cout << "Publish rate: " << publish_rate_ << endl;



		handle_ = hw->getHandle(port_);
		ir_interface_.setSensor(handle_.getSensor());


		//TODO Mode handling

		if(!ir_interface_.isConnected())
		{

			ROS_ERROR_STREAM(
					"Need Infrared Sensor on port: "<<port_);
			return false;
		}


		/**
		 * \page Ev3InfraredController Ev3InfraredController
		 * \subsection topic_name
		 * is the topic name used for the output topic or if there are multiple
		 * existing topics for the mode like for seek and remote.
		 * The topic_name for each topic gets an appending number starting
		 * from 0 (e.g. topic_name0).
		 */

		/**
		 * \page Ev3InfraredController Ev3InfraredController
		 * \subsection frame_id
		 * is used in modes publishing a message with an header to set the frame_id
		 * for the sensor.
		 */
		if (!ctrl_nh.getParam("frame_id", frame_id_))
		{
			frame_id_=port_;
			ROS_INFO_STREAM("Parameter frame_id not given or wrong type, using "<<port_);
		}

		std::string topic_name;
		std::string param_name="topic_name";
		switch(mode_)
		{
		case Ev3Strings::EV3INFRAREDMODE_IR_PROX:

			topic_name=port_+"_ir_range";
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



			if(max_range_>2.50)
			{
				ROS_ERROR("Parameter max_range to big! 2.550 is error condition of the sensor using 2.5");
				max_range_=2.5;
			}

			realtime_range_publisher_->msg_.min_range=min_range_;
			realtime_range_publisher_->msg_.max_range=max_range_;
			realtime_range_publisher_->msg_.radiation_type=1; //INFRARED
			realtime_range_publisher_->msg_.header.frame_id=frame_id_;
			realtime_range_publisher_->msg_.field_of_view=0.872665; //50deg (in rads)
			break;

		case Ev3Strings::EV3INFRAREDMODE_IR_SEEK:
			topic_name=port_+"_ir_seek";


			std::cout<<"Seek Mode Setup!"<<std::endl;
			//Create publisher for Listen
			for (int i = 0; i < 4; ++i)
			{
				std::string number="0";
				number[0]+=i;
				if (!ctrl_nh.getParam(param_name+number, topic_name))
				{
					topic_name=port_+"_ir_seek"+number;
					ROS_INFO_STREAM("Parameter "<<param_name+number<<" not given using "<<topic_name<<" for channel "<<i);
				}
				realtime_seek_publishers_[i] = RtSeekPublisherPtr(
						new realtime_tools::RealtimePublisher<h4r_ev3_msgs::Seek>(
								root_nh, topic_name, 4));

			}
			break;

		case Ev3Strings::EV3INFRAREDMODE_IR_REMOTE:
			topic_name=port_+"_ir_remote";


			std::cout<<"Remote Mode Setup!"<<std::endl;
			//Create publisher for remotes
			for (int i = 0; i < 4; ++i)
			{
				std::string number="0";
				number[0]+=i;

				param_name="topic_name_"+number;
				if (!ctrl_nh.getParam(param_name, topic_name))
				{
					topic_name=port_+"_ir_remote_"+number;
					ROS_INFO_STREAM("Parameter "<<param_name<<" not given using "<<topic_name<<" for channel "<<i);
				}


				realtime_joy_publishers_[i] = RtJoyPublisherPtr(
						new realtime_tools::RealtimePublisher<sensor_msgs::Joy>(
								root_nh, topic_name, 4,true));
				realtime_joy_publishers_[i]->msg_.buttons.resize(5);
				realtime_joy_publishers_[i]->msg_.header.frame_id=frame_id_;
			}
			break;



		default:
			ROS_ERROR_STREAM("Mode "<<mode_str<<" not supported by this controller!");
			return false;
			break;
		}


		if(mode_==Ev3Strings::EV3INFRAREDMODE_IR_PROX)
		{
			value_number_=1;
		}
		else if(mode_==Ev3Strings::EV3INFRAREDMODE_IR_SEEK)
		{
			value_number_=8;
		}
		else if(mode_==Ev3Strings::EV3INFRAREDMODE_IR_REMOTE)
		{
			value_number_=4;
		}
		else
		{
			//Normally we should not get here but if a mode was added it saves debugging time ;-)
			ROS_ERROR("Mode not supported - value number unknown!");
			return false;
		}

		return true;
	}

void Ev3InfraredController::starting(const ros::Time& time)
{
	for (int i = 0; i < 4; ++i)
	{
		last_publish_time_[i] = time;
	}
}

void Ev3InfraredController::update(const ros::Time& time, const ros::Duration& /*period*/)
	{
		using namespace hardware_interface;

		int value[8];


		if(!ir_interface_.isConnected())
		{
			//ROS_ERROR_STREAM("Lego Subsonic Sensor disconnected for port: "<<port_);
			sensor_mode_needs_init_=true;
			return;
		}

		if(sensor_mode_needs_init_)
		{
			sensor_mode_needs_init_=!(ir_interface_.setMode(mode_));
			if(sensor_mode_needs_init_)
			{
				ROS_ERROR("Could not set sensor mode!");
			}
		}

		//Optain all values...
		for (int i = 0; i < value_number_; ++i)
		{
			if (!handle_.getValue(i, value[i]))
			{
				return;
			}
		}

		bool published[4]={false,false,false,false};
		switch(mode_)
		{
		case Ev3Strings::EV3INFRAREDMODE_IR_PROX:
			if (publish_rate_ > 0.0
				&& last_publish_time_[0] + ros::Duration(1.0 / publish_rate_)< time)
			{

				if (realtime_range_publisher_->trylock())
				{
					realtime_range_publisher_->msg_.header.stamp = time;

					if(value[0]!=2550)
					{
						double dval = ((double) value[0]) / 1000.0; //to meters

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
					published[0]=true;
				}
			}
			break;

				case Ev3Strings::EV3INFRAREDMODE_IR_SEEK:
					for (int i = 0; i < 4; ++i)
					{
						if (publish_rate_ > 0.0
						    && last_publish_time_[i] + ros::Duration(1.0 / publish_rate_)< time)
						{
							if (realtime_seek_publishers_[i]->trylock())
							{
								double distance=value[i*2+1];
								double heading=value[i*2];

								if(distance>=100.0)
								{
									realtime_seek_publishers_[i]->msg_.distance=std::numeric_limits<double>::infinity();
									realtime_seek_publishers_[i]->msg_.heading=std::numeric_limits<double>::quiet_NaN();
								}
								else if(distance>=0.0)
								{
									realtime_seek_publishers_[i]->msg_.distance=distance;
									realtime_seek_publishers_[i]->msg_.heading=heading;
								}
								else //distance<0
								{
									realtime_seek_publishers_[i]->msg_.distance=std::numeric_limits<double>::quiet_NaN();
								}

								realtime_seek_publishers_[i]->unlockAndPublish();
								published[i]=true;
							}
						}

					}
					break;

				case Ev3Strings::EV3INFRAREDMODE_IR_REMOTE:
					for (int i = 0; i < 4; ++i)
					{
						if (publish_rate_ > 0.0
						    && last_publish_time_[i] + ros::Duration(1.0 / publish_rate_)< time)
						{
							if (realtime_joy_publishers_[i]->trylock())
							{

								realtime_joy_publishers_[i]->msg_.header.stamp=ros::Time::now();

								bool red_up_pressed=
										value[i]==1
										|| value[i]==5
										|| value[i]==6
										|| value[i]==10;

								bool red_down_pressed=
										   value[i]==2
										|| value[i]==7
										|| value[i]==8
										|| value[i]==10;

								bool blue_up_pressed=
										value[i]==3
										|| value[i]==5
										|| value[i]==7
										|| value[i]==11;
								bool blue_down_pressed=
										value[i]==4
										|| value[i]==6
										|| value[i]==8
										|| value[i]==11;

								bool beacon_mode_active=
										value[i]==9;


								//Only publish on change otherwise assume published and end...
								if(
									realtime_joy_publishers_[i]->msg_.buttons[0]==red_up_pressed&&
									realtime_joy_publishers_[i]->msg_.buttons[1]==red_down_pressed&&
									realtime_joy_publishers_[i]->msg_.buttons[2]==blue_up_pressed&&
									realtime_joy_publishers_[i]->msg_.buttons[3]==blue_down_pressed&&
									realtime_joy_publishers_[i]->msg_.buttons[4]==beacon_mode_active&&
									!first_time_[i]
								)
								{
									realtime_joy_publishers_[i]->unlock();
									published[i]=true;
								}
								else
								{

									realtime_joy_publishers_[i]->msg_.buttons[0]=red_up_pressed;
									realtime_joy_publishers_[i]->msg_.buttons[1]=red_down_pressed;
									realtime_joy_publishers_[i]->msg_.buttons[2]=blue_up_pressed;
									realtime_joy_publishers_[i]->msg_.buttons[3]=blue_down_pressed;
									realtime_joy_publishers_[i]->msg_.buttons[4]=beacon_mode_active;

									realtime_joy_publishers_[i]->unlockAndPublish();
									published[i]=true;
									first_time_[i]=false;
								}

							}
						}
					}

				break;

				default:
					break;
				}

				for (int i = 0; i < 4; ++i)
				{
					if(published[i])
					{
						last_publish_time_[i] = last_publish_time_[i]
								+ ros::Duration(1.0 / publish_rate_);
					}
				}


	}

PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3InfraredController, controller_interface::ControllerBase)

} /* namespace ev3_control */
