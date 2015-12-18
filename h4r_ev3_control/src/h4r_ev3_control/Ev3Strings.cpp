
/*WARNING WARNING WARNING WARNING WARNING
 *This file is generated by script
 *generateMaps.py, to add strings change
 *strings.yml and run it again!
 *WARNING WARNING WARNING WARNING WARNING
 */
 #include <h4r_ev3_control/Ev3Strings.h>
namespace h4r_ev3_control{
StringEnum<Ev3Strings::Ev3Switch> init_ev3_switch_conv()
{
	StringEnum<Ev3Strings::Ev3Switch> mp;
	mp.insert("on");
	mp.insert("off");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3Switch>Ev3Strings::ev3_switch_conv = init_ev3_switch_conv();



StringEnum<Ev3Strings::Ev3UltrasonicMode> init_ev3_ultrasonic_mode_conv()
{
	StringEnum<Ev3Strings::Ev3UltrasonicMode> mp;
	mp.insert("US-DC-IN");
	mp.insert("US-DIST-CM");
	mp.insert("US-LISTEN");
	mp.insert("US-DC-CM");
	mp.insert("US-SI-IN");
	mp.insert("US-SI-CM");
	mp.insert("US-DIST-IN");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3UltrasonicMode>Ev3Strings::ev3_ultrasonic_mode_conv = init_ev3_ultrasonic_mode_conv();



StringEnum<Ev3Strings::Ev3ColorMode> init_ev3_color_mode_conv()
{
	StringEnum<Ev3Strings::Ev3ColorMode> mp;
	mp.insert("COL-COLOR");
	mp.insert("COL-AMBIENT");
	mp.insert("COL-REFLECT");
	mp.insert("RGB-RAW");
	mp.insert("COL-CAL");
	mp.insert("REF-RAW");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3ColorMode>Ev3Strings::ev3_color_mode_conv = init_ev3_color_mode_conv();



StringEnum<Ev3Strings::Ev3DriverName> init_ev3_driver_name_conv()
{
	StringEnum<Ev3Strings::Ev3DriverName> mp;
	mp.insert("lego-ev3-touch");
	mp.insert("lego-ev3-color");
	mp.insert("none");
	mp.insert("lego-ev3-us");
	mp.insert("lego-ev3-gyro");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3DriverName>Ev3Strings::ev3_driver_name_conv = init_ev3_driver_name_conv();



StringEnum<Ev3Strings::Ev3Polarity> init_ev3_polarity_conv()
{
	StringEnum<Ev3Strings::Ev3Polarity> mp;
	mp.insert("inversed");
	mp.insert("normal");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3Polarity>Ev3Strings::ev3_polarity_conv = init_ev3_polarity_conv();



StringEnum<Ev3Strings::Ev3PortStatus> init_ev3_port_status_conv()
{
	StringEnum<Ev3Strings::Ev3PortStatus> mp;
	mp.insert("ev3-uart");
	mp.insert("no-connect");
	mp.insert("ev3-analog");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3PortStatus>Ev3Strings::ev3_port_status_conv = init_ev3_port_status_conv();



StringEnum<Ev3Strings::Ev3GyroMode> init_ev3_gyro_mode_conv()
{
	StringEnum<Ev3Strings::Ev3GyroMode> mp;
	mp.insert("GYRO-FAS");
	mp.insert("GYRO-ANG");
	mp.insert("GYRO-G&A");
	mp.insert("GYRO-CAL");
	mp.insert("GYRO-RATE");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3GyroMode>Ev3Strings::ev3_gyro_mode_conv = init_ev3_gyro_mode_conv();



StringEnum<Ev3Strings::Ev3MotorCommands> init_ev3_motor_commands_conv()
{
	StringEnum<Ev3Strings::Ev3MotorCommands> mp;
	mp.insert("reset");
	mp.insert("run-to-rel-pos");
	mp.insert("run-forever");
	mp.insert("run-direct");
	mp.insert("stop");
	mp.insert("run-timed");
	mp.insert("run-to-abs-pos");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3MotorCommands>Ev3Strings::ev3_motor_commands_conv = init_ev3_motor_commands_conv();



StringEnum<Ev3Strings::Ev3TouchMode> init_ev3_touch_mode_conv()
{
	StringEnum<Ev3Strings::Ev3TouchMode> mp;
	mp.insert("TOUCH");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3TouchMode>Ev3Strings::ev3_touch_mode_conv = init_ev3_touch_mode_conv();



StringEnum<Ev3Strings::Ev3PortDrivers> init_ev3_port_drivers_conv()
{
	StringEnum<Ev3Strings::Ev3PortDrivers> mp;
	mp.insert("legoev3-input-port");
	mp.insert("legoev3-input-outport");
	mp.finalize();
	return mp;
}

StringEnum<Ev3Strings::Ev3PortDrivers>Ev3Strings::ev3_port_drivers_conv = init_ev3_port_drivers_conv();



}
