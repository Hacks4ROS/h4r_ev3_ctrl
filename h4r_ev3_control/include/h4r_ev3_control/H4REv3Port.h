/*
 * This file (H4REv3Port.h) is part of h4r_ev3_control.
 * Date: 22.11.2015
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

#ifndef H4REV3PORT_H_
#define H4REV3PORT_H_

#include <iostream>

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <map>

#include <string>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <dirent.h>

#include <h4r_ev3_control/Ev3Strings.h>
#include <h4r_ev3_control/FixedBuffer.h>
#include <h4r_ev3_control/syshelpers.h>

namespace ev3_control
{

class H4REv3Port
{
public:
	typedef enum
	{
		H4REV3PORT_OUT,
		H4REV3PORT_IN,
	}H4Ev3IoPortType;



	class OpenFile
	{
	public:
		typedef enum
		{
			MODE_RW,
			MODE_R,
			MODE_W,
		}FileMode;
		OpenFile()
		:ptr(0)
		,connect_id_(0)
		{}
		int connect_id_;
		FileNameBuffer fullpath;
		FILE *ptr;
	};




protected:


	OpenFile f_driver_name;

	Ev3Strings::Ev3DriverName last_driver_;
	int last_driver_check_connected_id_;


	const std::string port_name_;
	bool connected_;
	int connect_id_;


	H4Ev3IoPortType port_type_;
	FileNameBuffer sys_port_directory_;
	FileNameBuffer sys_device_directory_;

//	FILE* openFile(FileNameBuffer fullpath, FileMode mode);
	FILE* get_fileptr_(const char* filename, OpenFile::FileMode mode, OpenFile &file, bool device_dir=true);

	bool getDeviceDirectory();
	bool getPortDirectory();




public:


	/**
	 * The constructor
	 * @param port_name port_name
	 * @param port_type input or output port
	 */
	H4REv3Port(const std::string &port_name, H4Ev3IoPortType port_type);
	~H4REv3Port();




	/**
	 * Check if a device is connected to the port
	 * @return True if connected
	 */
	bool isConnected();


	/**
	 * Reads a integer string from a sys file
	 * @param filename The sys file to be read from (only filename without directory)
	 * @param[out] value output variable for the value only valid on return true
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @param openFile Storage for file pointer
	 * @return True if read successful, false otherwise
	 */
	bool readInt(const char *filename, int &value, OpenFile& openFile, bool device_dir=true);

	/**
	 * Writes a integer string from a sys file
	 * @param filename The sys file to be read from (only filename without directory)
	 * @param value The value to be written to the file
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @param openFile Storage for file pointer
	 * @return True if read successful, false otherwise
	 */
	bool writeInt(const char *filename, int value, OpenFile& openFile, bool device_dir=true);


	/**
	 * @return The name of the port
	 */
	const std::string& getPortName() const
	{
		return port_name_;
	}


	/**
	 * @return The ports type
	 */
	H4Ev3IoPortType getPortType() const
	{
		return port_type_;
	}

	/**
	 * get driver name from current device
	 * @param[out] drvname Driver name enum type
	 * @return True if successful, false otherwise (false normally means, nothing is connected)
	 */
	bool getDriverName(Ev3Strings::Ev3DriverName &drvname)
	{
		if(!isConnected())
		{
			return false;
		}


		//Reduce file reads and string processing ...
		//Was the last connection id the same as on last check?
		if(connect_id_==last_driver_check_connected_id_)
		{
			//If yes we can use the stored value
			drvname=last_driver_;
			return true;
		}
		else //if not we read the current driver name and convert it to enum
		{
			bool ret=readKey("driver_name",Ev3Strings::ev3_driver_name_conv,drvname,f_driver_name);
			std::cout<<"Driver Name: "<<drvname<<std::endl;

			if(ret) //success?
			{
				//Store id and driver name to be used instead of reading the file
				last_driver_=drvname;
				last_driver_check_connected_id_=connect_id_;
			}

			return ret;
		}


	}

	/**
	 * Writes a string file from a map into a /sys file according to the given key.
	 * @param filename The sys file to be written to (only filename without directory)
	 * @param strmap The string map
	 * @param key The key of the string (if return TRUE!)
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @param openFile Storage for file pointer
	 * @return True if read successful, false otherwise
	 */
	template < typename T >
	bool writeKey(const char *filename, const StringEnum< T > &strmap, T key, OpenFile& openFile, bool device_dir=true)
	{

		FILE *file=get_fileptr_(filename, OpenFile::MODE_W, openFile, device_dir);

		if(file==NULL)
			return false;

		return writeKeyToSysFile(file,strmap,key);
	}


	/**
	 * Reads a /sys string file and compares the read string with a given map of strings and gets it's number from the map
	 * @param filename The sys file to be read
	 * @param strmap The string map
	 * @param key The key of the string (if return TRUE!)
	 * @param device_dir True if it should search in the device_directory, false it will search the port directory for the file
	 * @param openFile Storage for file pointer
	 * @return True if read successful, false otherwise
	 */
	template <typename T>
	bool readKey(const char *filename,  const StringEnum< T > &strmap, T &key, OpenFile& openFile, bool device_dir=true)
	{

		FILE *file=get_fileptr_(filename, OpenFile::MODE_R, openFile, device_dir);

		if(file==NULL)
			return false;

		return readKeyFromSysFile(file,strmap,key);
	}
};


class H4REv3Motor : public H4REv3Port
{

	OpenFile f_DutyCycleSP;
	OpenFile f_SpeedRegulation;
	OpenFile f_SpeedSP;
	OpenFile f_PositionSP;
	OpenFile f_Speed;
	OpenFile f_Position;
	OpenFile f_MotorCommand;
	OpenFile f_MotorPolarity;
	OpenFile f_SpeedPID_Kp;
	OpenFile f_SpeedPID_Ki;
	OpenFile f_SpeedPID_Kd;

public:

	/**
	 *
	 * @param port_name
	 */
	H4REv3Motor(const std::string &port_name)
	:H4REv3Port(port_name,H4REV3PORT_OUT)
	{}

	/**
	 * Sets the pwm duty cycle value
	 * @param value The pwm value
	 * @return True if successful, false otherwise
	 */
	bool setDutyCycleSP(int value)
	{
		return writeInt("duty_cycle_sp", value, f_DutyCycleSP);
	}

	/**
	 * This function enables or disables the speed regulation functionality
	 * @param onoff On/Off
	 * @return True if successful, false otherwise
	 */
	bool setSpeedRegulation(Ev3Strings::Ev3Switch onoff)
	{
		return writeKey("speed_regulation", Ev3Strings::ev3_switch_conv, onoff, f_SpeedRegulation);
	}

	/**
	 * Set speed setpoint
	 * @param value The speed value in 1/10 rounds per second
	 * @return True if successful, false otherwise
	 */
	bool setSpeedSP(int value)
	{
		return writeInt("speed_sp",value,f_SpeedSP);
	}

	/**
	 * Set Position setpoint for position control
	 * @param value The position value
	 * @return True if successful, false otherwise
	 */
	bool setPositionSP(int value)
	{
		return writeInt("position_sp",value,f_PositionSP);
	}

	/**
	 * Get the current position of the motor
	 * @param[out] value The position value if return is true
	 * @return True if successful, false otherwise
	 */
	bool position(int &value)
	{
		return readInt("position",value, f_Position);
	}

	/**
	 * Get the current speed of the motor
	 * @param[out] value The position value if return is true
	 * @return True if successful, false otherwise
	 */
	bool speed(int &value)
	{
		return readInt("speed",value, f_Speed);
	}

	/**
	 * Setting the command (especially for motors)
	 * @param command The
	 * @return True if successful, false otherwise
	 */
	bool setMotorCommand(Ev3Strings::Ev3MotorCommands  command)
	{
		return writeKey("command",Ev3Strings::ev3_motor_commands_conv, command, f_MotorCommand);
	}

	/**
	 * Sets the rotation direction of the motor
	 * @param pol The direction
	 * @return True if successful
	 */
	bool setMotorPolarity(Ev3Strings::Ev3Polarity pol)
	{
		return writeKey<Ev3Strings::Ev3Polarity>("polarity",Ev3Strings::ev3_polarity_conv, pol, f_MotorPolarity);
	}

	/**
	 * Sets the Kp part of the motors speed control
	 * @param value
	 * @return True if successful
	 */
	bool setSpeedPID_Kp(unsigned int value)
	{
		return writeInt("speed_pid/Kp",value,f_SpeedPID_Kp);
	}

	/**
	 * Sets the Ki part of the motors speed control
	 * @param value
	 * @return True if successful
	 */
	bool setSpeedPID_Ki(unsigned int value)
	{
		return writeInt("speed_pid/Ki",value,f_SpeedPID_Ki);
	}

	/**
	 * Sets the Kd part of the motors speed control
	 * @param value
	 * @return True if successful
	 */
	bool setSpeedPID_Kd(unsigned int value)
	{
		return writeInt("speed_pid/Kd",value,f_SpeedPID_Kd);
	}
};


class H4REv3Sensor : public H4REv3Port
{
public:
	H4REv3Sensor(const std::string &port_name)
	:H4REv3Port(port_name,H4REV3PORT_IN)
	{}

private:
	OpenFile f_mode;
	OpenFile f_NumValues;
	OpenFile f_Value[8];

public:
	unsigned num_values()
	{
		int val=0;
		readInt("num_values",val,f_NumValues);
		return val;
	}

	bool value(unsigned number, int &value)
	{

		char numstring[]="value0";

		numstring[5]+=number;

		if(number<num_values() && number<=7)
		{
			return readInt(numstring,value,f_Value[number]);
		}
		else
		{
			return false;
		}

	}

	template < typename T >
	bool setModeT(const StringEnum< T > &modemap, T key)
	{

		FILE *file=get_fileptr_("mode", OpenFile::MODE_W, f_mode);

		if(file==NULL)
			return false;

		return writeKeyToSysFile(file,modemap,key);
	}
};


template <typename MODE_ENUM, StringEnum<MODE_ENUM>& MODE_CONV, Ev3Strings::Ev3DriverName DRV>
class H4REv3SensorSpecific
{
public:

	H4REv3Sensor *sensor_;
	H4REv3SensorSpecific()
	:sensor_(0)
	{}

	H4REv3SensorSpecific(H4REv3Sensor *sensor)
	:sensor_(sensor)
	 {}

	void setSensor(H4REv3Sensor *sensor)
	{
		sensor_=sensor;
	}

	H4REv3Sensor* getSensor()
	{
		return sensor_;
	}


	bool isConnected()
	{
		if(!sensor_)
			return false;

		if(sensor_->isConnected())
		{
			Ev3Strings::Ev3DriverName drvname;
			if(sensor_->getDriverName(drvname))
			{
				return drvname==DRV;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	bool setMode(MODE_ENUM mode)
	{
		if(!sensor_)
			return false;
		return sensor_->setModeT(MODE_CONV, mode);
	}
};

typedef H4REv3SensorSpecific<Ev3Strings::Ev3UltrasonicMode, Ev3Strings::ev3_ultrasonic_mode_conv, Ev3Strings::EV3DRIVERNAME_LEGO_EV3_US> H4REv3UltraSonicSensorSpecIface;
typedef H4REv3SensorSpecific<Ev3Strings::Ev3GyroMode, Ev3Strings::ev3_gyro_mode_conv, Ev3Strings::EV3DRIVERNAME_LEGO_EV3_GYRO> H4REv3GyroSensorSpecIface;
typedef H4REv3SensorSpecific<Ev3Strings::Ev3TouchMode, Ev3Strings::ev3_touch_mode_conv, Ev3Strings::EV3DRIVERNAME_LEGO_EV3_TOUCH> H4REv3TouchSensorSpecIface;
typedef H4REv3SensorSpecific<Ev3Strings::Ev3ColorMode, Ev3Strings::ev3_color_mode_conv, Ev3Strings::EV3DRIVERNAME_LEGO_EV3_COLOR> H4REv3ColorSensorSpecIface;
typedef H4REv3SensorSpecific<Ev3Strings::Ev3InfraredMode, Ev3Strings::ev3_infrared_mode_conv, Ev3Strings::EV3DRIVERNAME_LEGO_EV3_IR> H4REv3IRSensorSpecIface;


}/*ev3_control*/


#endif /* H4REV3PORT_H_ */
