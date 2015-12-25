/*
 * This file (H4REv3Port.cpp) is part of h4r_ev3_control.
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
#include <h4r_ev3_control/H4REv3Port.h>


namespace ev3_control
{

H4REv3Port::H4REv3Port(const std::string &port_name, H4Ev3IoPortType port_type)
:port_name_(port_name)
,port_type_(port_type)
,connected_(false)
,last_driver_check_connected_id_(0)
,last_driver_(Ev3Strings::EV3DRIVERNAME_NOT_FOUND)
{
	if(!getPortDirectory())
	{
		//TODO throw error
	}
	connected_=getDeviceDirectory();

}


H4REv3Port::~H4REv3Port()
{
	//TODO Close files
}

bool H4REv3Port::isConnected()
{
	if(connected_)
	{
		connected_ = pathExists(sys_device_directory_.c_str());
	}
	else
	{
		getDeviceDirectory();
	}

	return connected_;
}



bool H4REv3Port::getPortDirectory()
{
	return matchFileContentInEqualSubdirectories("/sys/class/lego-port","port_name",port_name_.c_str(),sys_port_directory_);
}

bool H4REv3Port::getDeviceDirectory()
{

	connected_=false;
	switch(port_type_)
	{
	case H4REV3PORT_IN:
		if(matchFileContentInEqualSubdirectories("/sys/class/lego-sensor","port_name",port_name_.c_str(), sys_device_directory_))
		{
			connect_id_++;
			connected_=true;
		}
		break;


	case H4REV3PORT_OUT:
		if(matchFileContentInEqualSubdirectories("/sys/class/tacho-motor","port_name",port_name_.c_str(), sys_device_directory_))
		{
			connect_id_++;
			connected_=true;
		}
		break;

	default:
		break;
	}

	return connected_;
}


FILE* H4REv3Port::get_fileptr_(const char* filename, OpenFile::FileMode mode, OpenFile &file, bool device_dir)
{

		if(connected_==false)
		{
			if(!getDeviceDirectory())
			{
				return NULL;
			}
		}
		else
		{
			if(!isConnected())
				return NULL;
		}

		if(file.ptr!=NULL
		&& connect_id_ == file.connect_id_ )
		{
			return file.ptr;
		}
		else
		{
			file.ptr=NULL;
			file.connect_id_=connect_id_;

			char const *smode;
			switch(mode)
			{
			case OpenFile::MODE_R:
					smode="r";
				break;
			case OpenFile::MODE_W:
					smode="w";
				break;
			case OpenFile::MODE_RW:
					smode="rw";
			}

			char const *dir;
			if(device_dir)
			{
				dir=sys_device_directory_.c_str();
			}
			else
			{
				dir=sys_port_directory_.c_str();
			}

			file.fullpath.format("%s/%s",dir,filename);
			file.ptr=fopen(file.fullpath.c_str(),smode);

			if(file.ptr==0)
				file.fullpath="";

			std::cout<<"getFilePTR opening: "<<file.fullpath.c_str()<<" "<<file.ptr<<std::endl;

			return file.ptr;
		}
}


bool H4REv3Port::readInt(const char *filename, int &value, OpenFile& openFile, bool device_dir)
{
	FILE *file=get_fileptr_(filename, OpenFile::MODE_R, openFile, device_dir);

	if(file==NULL)
		return false;

	return readIntFromSysFile(file,value);
}

bool H4REv3Port::writeInt(const char *filename, int value, OpenFile& openFile, bool device_dir)
{
	FILE *file=get_fileptr_(filename, OpenFile::MODE_W, openFile, device_dir);

	if(file==NULL)
		return false;

	return writeIntToSysFile(file,value);
}

}/*ev3_control*/
