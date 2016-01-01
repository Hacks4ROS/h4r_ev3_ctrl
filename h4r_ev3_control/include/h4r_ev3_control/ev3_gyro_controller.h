/*
 * This file (ev3_gyro_controller.h) is part of h4r_ev3_control.
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
#ifndef GYRO_CONTROLLER_H_
#define GYRO_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/Imu.h>
#include <limits>

#include <h4r_ev3_control/Ev3SensorInterface.h>

namespace ev3_control
{

class Ev3GyroController: public controller_interface::Controller<
		Ev3SensorInterface>
{
private:
	std::string port_;
	Ev3Strings::Ev3GyroMode mode_;
	Ev3SensorHandle handle_;
	H4REv3GyroSensorSpecIface gyro_interface_;
	bool sensor_mode_needs_init_;

	std::string frame_id_;

	//Range Publisher
	typedef boost::shared_ptr<
			realtime_tools::RealtimePublisher<sensor_msgs::Imu> > RtImuPublisherPtr;
	RtImuPublisherPtr realtime_imu_publisher_;



	ros::Time last_publish_time_;
	double publish_rate_;

public:
	Ev3GyroController();
	virtual ~Ev3GyroController();

	virtual bool init(Ev3SensorInterface* hw,
			ros::NodeHandle &root_nh,
			ros::NodeHandle& ctrl_nh);

	virtual void starting(const ros::Time& time);
	virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
	virtual void stopping(const ros::Time& /*time*/){}

};

} /* namespace ev3_control */

#endif /* GYRO_CONTROLLER_H_ */
