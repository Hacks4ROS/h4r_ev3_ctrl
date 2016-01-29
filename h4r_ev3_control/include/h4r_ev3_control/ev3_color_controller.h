/*
 * This file (ev3_color_controller.h) is part of h4r_ev3_control.
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


#ifndef EV3_COLOR_CONTROLLER_H_
#define EV3_COLOR_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Illuminance.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/UInt8.h>
#include <limits>

#include <h4r_ev3_control/Ev3SensorInterface.h>

namespace ev3_control
{

/**
 * \page Ev3ColorController Ev3ColorController
 * \ingroup Ev3Controllers
 * \image html EV3Color.png
 * \section Controller Function
 * This controller is used to read the standard EV3 Color Sensor
 *
 * \example color.launch
 * \example color.yaml
 */

/**
 * \ingroup Ev3Controllers
 */
class Ev3ColorController: public controller_interface::Controller<
		Ev3SensorInterface>
{
private:
	std::string port_;
	Ev3Strings::Ev3ColorMode mode_;
	Ev3SensorHandle handle_;
	H4REv3ColorSensorSpecIface color_interface_;
	bool sensor_mode_needs_init_;
	std::string frame_id_;

	//Range Publisher
	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<sensor_msgs::Illuminance> > RtIlluminancePublisherPtr;
	RtIlluminancePublisherPtr realtime_illuminance_publisher_;

	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<std_msgs::ColorRGBA> > RtColorPublisherPtr;
	RtColorPublisherPtr realtime_color_publisher_;

	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<std_msgs::UInt8> > RtColorNumberPublisherPtr;
	RtColorNumberPublisherPtr realtime_color_number_publisher_;

	ros::Time last_publish_time_;
	double publish_rate_;

public:
	Ev3ColorController();
	virtual ~Ev3ColorController();

	virtual bool init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh);

	virtual void starting(const ros::Time& time);
	virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
	virtual void stopping(const ros::Time& /*time*/){}

};

} /* namespace ev3_control */

#endif /* EV3_COLOR_CONTROLLER_H_ */
