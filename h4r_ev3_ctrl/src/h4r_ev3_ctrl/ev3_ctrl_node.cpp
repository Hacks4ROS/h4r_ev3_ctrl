/*
 * This file (ev3_ctrl_node.cpp}) is part of h4r_ev3_ctrl.
 * Date: 13.11.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_ctrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_ctrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with h4r_ev3_ctrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/callback_queue.h>
#include "controller_manager/controller_manager.h"

#include "Ev3HardwareInterface.h"
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Ev3ControlNode");
	ros::Time::init();

	ros::Rate master_wait_rate(2);
	while(!ros::master::check())
	{
		ROS_ERROR_ONCE("Waiting for rosmaster...");
		if(!ros::ok())
		{
			return 0;
		}
		master_wait_rate.sleep();
	}


	 ros::NodeHandle nh;
	 ros::CallbackQueue queue;
	 nh.setCallbackQueue(&queue);

	  ros::AsyncSpinner spinner(4, &queue);
	  spinner.start();



	 vector<ev3dev::port_type>ifaces;
	 ifaces.push_back(ev3dev::OUTPUT_A);
	 ifaces.push_back(ev3dev::OUTPUT_B);
	 ifaces.push_back(ev3dev::OUTPUT_C);
	 ifaces.push_back(ev3dev::OUTPUT_D);

	  h4r_ev3_ctrl::Ev3HardwareInterface robot(ifaces);
	  controller_manager::ControllerManager cm(&robot,nh);


	  ros::Time ts = ros::Time::now();

	  while (ros::ok())
	  {
	     ros::Duration d = ts - ros::Time::now();
	     ts = ros::Time::now();
	     robot.read();
	     cm.update(ts, d);
	     robot.write();
	     usleep(100000);
	  }

	  spinner.stop();

	return 0;
}
