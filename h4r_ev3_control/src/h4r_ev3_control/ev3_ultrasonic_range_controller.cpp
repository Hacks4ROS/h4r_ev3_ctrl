/*
 * This file (ev3_ultrasonic_range_controller.cpp) is part of h4r_ev3_control.
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

#include <h4r_ev3_control/ev3_ultrasonic_range_controller.h>

namespace ev3_control
{

Ev3UltraSonicRangeController::Ev3UltraSonicRangeController()
:sensor_mode_needs_init_(true)
,mode_(Ev3Strings::EV3ULTRASONICMODE_US_DIST_CM)
,max_range_(2.0)
,min_range_(0)
,publish_rate_(10)
{
	// TODO Auto-generated constructor stub

}

Ev3UltraSonicRangeController::~Ev3UltraSonicRangeController()
{
	// TODO Auto-generated destructor stub
}


PLUGINLIB_EXPORT_CLASS(ev3_control::Ev3UltraSonicRangeController, controller_interface::ControllerBase)

} /* namespace ev3_control */
