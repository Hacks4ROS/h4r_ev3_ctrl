/*
 * This file (ev3_ctrl_node.cpp) is part of h4r_ev3_manager.
 * Date: 13.11.2015
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ev3_manager is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ev3_manager is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ev3_control.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <ros/ros.h>
#include <ros/master.h>
#include <iostream>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

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


	 ros::NodeHandle n;
	 ros::NodeHandle nh("~");

	 ros::CallbackQueue queue;
	 nh.setCallbackQueue(&queue);
	 n.setCallbackQueue(&queue);

	  ros::AsyncSpinner spinner(4, &queue);
	  spinner.start();


	  std::vector<string> out_ports, in_ports;
	  nh.getParam("OutPorts",out_ports);
	  nh.getParam("InPorts", in_ports);

	  if(out_ports.size()==0)
	  {
		  out_ports.push_back("outA");
		  out_ports.push_back("outB");
		  out_ports.push_back("outC");
		  out_ports.push_back("outD");
	  }

	  if(in_ports.size()==0)
	  {
		  in_ports.push_back("in1");
		  in_ports.push_back("in2");
		  in_ports.push_back("in3");
		  in_ports.push_back("in4");
	  }


	  ev3_control::Ev3HardwareInterface robot(in_ports,out_ports);
	  controller_manager::ControllerManager cm(&robot,n);

	  ros::Time ts = ros::Time::now();

	  ros::Rate loop_rate(10);
	  while (ros::ok())
	  {
	     ros::Duration d = ros::Time::now() - ts;
	     robot.read();
	     ts = ros::Time::now();
	     cm.update(ts, d);
	     robot.write(d);
	     loop_rate.sleep();
	  }

	  spinner.stop();

	return 0;
}
