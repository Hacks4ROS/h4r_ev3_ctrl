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

using namespace std;




class H4REv3Port
{
public:
	typedef enum
	{
		H4REV3PORT_OUT,
		H4REV3PORT_IN,
	}H4Ev3IoPortType;

private:
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

	H4REv3Port(const std::string &port_name, H4Ev3IoPortType port_type);
	~H4REv3Port();

	bool getDeviceDirectory();
	bool getPortDirectory();

	FILE* openFile(const std::string fullpath, OpenFile::FileMode mode);
	FILE *get_fileptr_(const std::string &filename, OpenFile::FileMode mode,bool device_dir=true);


public:
	bool readInt(const std::string &filename, int &value, bool device_dir=true);
	bool writeInt(const std::string &filename, int value, bool device_dir=true);

	template <typename T>
	bool writeKey(const std::string &filename, const std::map<T,string> &strmap, T key, bool device_dir=true)
	{
		FILE * file=get_fileptr_(filename, OpenFile::MODE_W, device_dir);


		if(file==NULL)
			return false;

		return writeKeyToSysFile(file,strmap,key);
	}


	template <typename T>
	bool readKey(const std::string &filename, const std::map<string,T> &strmap, T &key, bool device_dir=true)
	{
		FILE * file=get_fileptr_(filename, OpenFile::MODE_R, device_dir);


		if(file==NULL)
			return false;

		return readKeyFromSysFile(file,strmap,key);
	}

	bool setSpeedSP(int value)
	{
		return writeInt("speed_sp",value);
	}

	bool setPositionSP(int value)
	{
		return writeInt("position",value);
	}

	bool setCommand(ev3dev::Ev3Strings::Ev3MotorCommands command)
	{
		return writeKey("command",ev3dev::Ev3Strings::ev3_motor_commands_string, command);
	}



};


#endif /* H4REV3PORT_H_ */
