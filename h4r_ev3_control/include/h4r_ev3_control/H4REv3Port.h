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
		{}
		FileNameBuffer fullpath;
		FILE *ptr;
	};




protected:
	const std::string port_name_;
	bool connected_;
	bool reconnect_;

	H4Ev3IoPortType port_type_;
	FileNameBuffer sys_port_directory_;
	FileNameBuffer sys_device_directory_;

//	FILE* openFile(FileNameBuffer fullpath, FileMode mode);
	FILE* get_fileptr_(const char* filename, OpenFile::FileMode mode, OpenFile &file, bool device_dir=true);
;

	bool getDeviceDirectory();
	bool getPortDirectory();




public:

	void reconnect()
	{
		reconnect_=true;
		getDeviceDirectory();
	}

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
	 * Writes a string file from a map into a /sys file according to the given key.
	 * @param filename The sys file to be written to (only filename without directory)
	 * @param strmap The string map
	 * @param key The key of the string (if return TRUE!)
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @param openFile Storage for file pointer
	 * @return True if read successful, false otherwise
	 */
	template < typename T >
	bool writeKey(const char *filename, const std::map< T, std::string > &strmap, T key, OpenFile& openFile, bool device_dir=true)
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
	bool readKey(const char *filename, const std::map< T, std::string > &strmap, T &key, OpenFile& openFile, bool device_dir=true)
	{

		FILE *file=get_fileptr_(filename, OpenFile::MODE_W, openFile, device_dir);

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


public:

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
		return writeKey("speed_regulation", Ev3Strings::ev3_switch_string, onoff, f_DutyCycleSP);
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
		return writeKey("command",Ev3Strings::ev3_motor_commands_string, command, f_MotorCommand);
	}

	/**
	 * Sets the rotation direction of the motor
	 * @param pol The direction
	 * @return True if successful
	 */
	bool setMotorPolarity(Ev3Strings::Ev3Polarity pol)
	{
		return writeKey<Ev3Strings::Ev3Polarity>("polarity",Ev3Strings::ev3_polarity_string, pol, f_MotorPolarity);
	}

};


class H4REv3Sensor : public H4REv3Port
{
public:
	H4REv3Sensor(const std::string &port_name)
	:H4REv3Port(port_name,H4REV3PORT_IN)
	{}

private:
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
};


}/*ev3_control*/


#endif /* H4REV3PORT_H_ */
