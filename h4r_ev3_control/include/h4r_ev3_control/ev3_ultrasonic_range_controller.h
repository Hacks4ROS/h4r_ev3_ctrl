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

#include "../h4r_ev3_control/Ev3SensorInterface.h"

namespace ev3_control
{

class Ev3UltraSonicRangeController: public controller_interface::Controller<Ev3SensorInterface>
{
private:
	  std::string port_;
	  std::string mode_;
	  Ev3SensorHandle handle_;

	  //Range Publisher
	  typedef boost::shared_ptr<realtime_tools::RealtimePublisher< sensor_msgs::Range > > RtRangePublisherPtr;
	  RtRangePublisherPtr realtime_range_publisher_;


	  ros::Time last_range_publish_time_;
	  double publish_rate_;


public:
	Ev3UltraSonicRangeController();
	virtual ~Ev3UltraSonicRangeController();

	  virtual bool init(Ev3SensorInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& ctrl_nh)
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

		    if (!ctrl_nh.getParam("mode", mode_))
		    {
		      ROS_ERROR("Parameter mode was not set");
		      return false;
		    }

		    //TODO REMOVE
		    cout<<"Port: "<<port_<<endl;
		    cout<<"Mode: "<<mode_<<endl;
		    cout<<"Publish rate: "<<publish_rate_<<endl;

		    handle_=hw->getHandle(port_);


		    //TODO Mode handling

		    if(handle_.getDriverName()!=Ev3Strings::EV3DRIVERNAME_LEGO_EV3_US)
		    {

		    	ROS_ERROR_STREAM("Need Ultrasonic Sensor on port: "<<port_);
		    	return false;
		    }

		    //Create publisher for Rangesensor
		    realtime_range_publisher_=RtRangePublisherPtr(new realtime_tools::RealtimePublisher<sensor_msgs::Range>(root_nh, port_, 4));

		    return true;
	  }

	  virtual void starting(const ros::Time& time)
	  {
	      last_range_publish_time_ = time;
	  }

	  virtual void update(const ros::Time& time, const ros::Duration& /*period*/)
	  {




		    using namespace hardware_interface;

		    // limit rate of publishing
		      if (publish_rate_ > 0.0
		       && last_range_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
		      {
		        //Check if we can lock publisher
		        if (realtime_range_publisher_->trylock())
		        {
		          last_range_publish_time_ = last_range_publish_time_ + ros::Duration(1.0/publish_rate_);

				  if(handle_.getDriverName()!=Ev3Strings::EV3DRIVERNAME_LEGO_EV3_US)
				  {
					  ROS_ERROR("Lego Subsonic Sensor disconnected!");
					  return;
				  }

		          int value;
		          if(!handle_.getValue(0,value))
		          {
		        	  ROS_ERROR("Could not get sensor value!");
		        	  realtime_range_publisher_->unlock();
		        	  return;
		          }




		          realtime_range_publisher_->msg_.header.stamp = time;
		          realtime_range_publisher_->msg_.range=((double) value)/1000.0;
		          realtime_range_publisher_->unlockAndPublish();
		        }
		      }
	  }

	  virtual void stopping(const ros::Time& /*time*/)
	  {

	  }


};

} /* namespace ev3_control */

#endif /* EV3ULTRASONICCONTROLLER_H_ */
