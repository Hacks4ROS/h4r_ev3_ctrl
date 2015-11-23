/*
 * This file (H4REv3Port.h) is part of h4r_ev3_ctrl.
 * Date: 22.11.2015
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

#ifndef H4REV3PORT_H_
#define H4REV3PORT_H_

#include <iostream>

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <map>

#include <h4r_ev3_ctrl/syshelpers.h>
#include <h4r_ev3_ctrl/Ev3Strings.h>


namespace ev3dev
{


class H4REv3Port
{
public:
	typedef enum
	{
		H4REV3PORT_OUT,
		H4REV3PORT_IN,
	}H4Ev3IoPortType;

protected:
	std::string port_name_;
	H4Ev3IoPortType port_type_;
	std::string sys_port_directory_;
	std::string sys_device_directory_;
	bool port_found_;
	bool connected_;


	class OpenFile
	{

	public:
		OpenFile()
		:ptr(0)
		,mode(MODE_RW)
		{};

		typedef enum
		{
			MODE_RW,
			MODE_R,
			MODE_W,
		}FileMode;

		FILE *ptr;
		FileMode mode;
	};

	typedef std::map<std::string, OpenFile>  FileCache;
	typedef std::pair<std::string, OpenFile> FileCachePair;
	FileCache file_cache;
	FILE* openFile(const std::string fullpath, OpenFile::FileMode mode);
	FILE *get_fileptr_(const std::string &filename, OpenFile::FileMode mode,bool device_dir=true);
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
	 * Reads a integer string from a sys file
	 * @param filename The sys file to be read from (only filename without directory)
	 * @param[out] value output variable for the value only valid on return true
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @return True if read successful, false otherwise
	 */
	bool readInt(const std::string &filename, int &value, bool device_dir=true);

	/**
	 * Writes a integer string from a sys file
	 * @param filename The sys file to be read from (only filename without directory)
	 * @param value The value to be written to the file
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @return True if read successful, false otherwise
	 */
	bool writeInt(const std::string &filename, int value, bool device_dir=true);

	/**
	 * Writes a string file from a map into a /sys file according to the given key.
	 * @param filename The sys file to be written to (only filename without directory)
	 * @param strmap The string map
	 * @param key The key of the string (if return TRUE!)
	 * @param device_dir True if it should open the file in the device_directory, false it will search the port directory for the file
	 * @return True if read successful, false otherwise
	 */
	template <typename T>
	bool writeKey(const std::string &filename, const std::map<T,std::string> &strmap, T key, bool device_dir=true)
	{
		FILE * file=get_fileptr_(filename, OpenFile::MODE_W, device_dir);


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
	 * @return True if read successful, false otherwise
	 */
	template <typename T>
	bool readKey(const std::string &filename, const std::map<std::string,T> &strmap, T &key, bool device_dir=true)
	{
		FILE * file=get_fileptr_(filename, OpenFile::MODE_R, device_dir);


		if(file==NULL)
			return false;

		return readKeyFromSysFile(file,strmap,key);
	}


	/**
	 * Setting the command (especially for motors)
	 * @param command The
	 * @return True if successful, false otherwise
	 */
	bool setCommand(ev3dev::Ev3Strings::Ev3MotorCommands  command)
	{
		return writeKey("command",ev3dev::Ev3Strings::ev3_motor_commands_string, command);
	}

};


class H4REv3Motor : public H4REv3Port
{
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
		return writeInt("duty_cycle_sp", value);
	}

	/**
	 * This function enables or disables the speed regulation functionality
	 * @param onoff On/Off
	 * @return True if successful, false otherwise
	 */
	bool setSpeedRegulation(ev3dev::Ev3Strings::Ev3Switch onoff)
	{
		return writeKey("speed_regulation", ev3dev::Ev3Strings::ev3_switch_string, onoff);
	}

	/**
	 * Set speed setpoint
	 * @param value The speed value in 1/10 rounds per second
	 * @return True if successful, false otherwise
	 */
	bool setSpeedSP(int value)
	{
		return writeInt("speed_sp",value);
	}

	/**
	 * Set Position setpoint for position control
	 * @param value The position value
	 * @return True if successful, false otherwise
	 */
	bool setPositionSP(int value)
	{
		return writeInt("position_sp",value);
	}

	/**
	 * Get the current position of the motor
	 * @param[out] value The position value if return is true
	 * @return True if successful, false otherwise
	 */
	bool position(int &value)
	{
		return readInt("position",value);
	}

	/**
	 * Get the current speed of the motor
	 * @param[out] value The position value if return is true
	 * @return True if successful, false otherwise
	 */
	bool speed(int &value)
	{
		return readInt("speed",value);
	}

};




}/* ev3dev */


#endif /* H4REV3PORT_H_ */
