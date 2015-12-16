/*
 * Ev3UltraSonicController.cpp
 *
 *  Created on: 10.12.2015
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2015)
 */

#include <h4r_ev3_sensor_control/ev3_ultrasonic_range_controller.h>

namespace h4r_ev3_sensor_control
{

Ev3UltraSonicRangeController::Ev3UltraSonicRangeController()
{
	// TODO Auto-generated constructor stub

}

Ev3UltraSonicRangeController::~Ev3UltraSonicRangeController()
{
	// TODO Auto-generated destructor stub
}


PLUGINLIB_EXPORT_CLASS(h4r_ev3_sensor_control::Ev3UltraSonicRangeController, controller_interface::ControllerBase)

} /* namespace h4r_ev3_ctrl */
