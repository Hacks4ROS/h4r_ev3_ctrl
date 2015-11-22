/*
 * This file (H4REv3Port.cpp) is part of h4r_ev3_ctrl.
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
#include <h4r_ev3_ctrl/H4REv3Port.h>

H4REv3Port::H4REv3Port(const std::string &port_name, H4Ev3IoPortType port_type)
:port_name_(port_name)
,port_type_(port_type)
,connected_(false)
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


bool H4REv3Port::getPortDirectory()
{
	return matchFileContentInEqualSubdirectories("/sys/class/lego-port","port_name",port_name_,sys_port_directory_);
}

bool H4REv3Port::getDeviceDirectory()
{

	connected_=false;
	switch(port_type_)
	{
	case H4REV3PORT_IN:
		if(matchFileContentInEqualSubdirectories("/sys/class/lego-sensor","port_name",port_name_,sys_device_directory_))
		{



			connected_=true;
		}
		break;


	case H4REV3PORT_OUT:
		if(matchFileContentInEqualSubdirectories("/sys/class/tacho-motor","port_name",port_name_,sys_device_directory_))
		{

			connected_=true;
		}
		break;

	default:
		break;
	}

	return connected_;
}


FILE* H4REv3Port::openFile(const std::string fullpath, OpenFile::FileMode mode)
{
	OpenFile file;
	FileCache::iterator it=file_cache.find(fullpath);
	std::string smode;

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

	bool open=false;
	bool rw=false;
	if(it==file_cache.end())
	{
		open=true;
	}
	else
	{
		if(it->second.mode!=mode && it->second.mode != OpenFile::MODE_RW)
		{
			smode="rw";
			mode=OpenFile::MODE_RW;
			fclose(it->second.ptr);
			file_cache.erase(it);
			open=true;
		}
		else
		{
			return it->second.ptr;
		}
	}


	if(open)
	{
		file.mode=mode;
		file.ptr=fopen(fullpath.c_str(),smode.c_str());

		if(file.ptr!=NULL)
		{
			FileCachePair pr=FileCachePair(fullpath,file);
			file_cache.insert(pr);
			return file.ptr;
		}
		else
		{
			return NULL;
		}
	}

}

FILE* H4REv3Port::get_fileptr_(const std::string &filename, OpenFile::FileMode mode,bool device_dir=true)
{
	std::string file;
	if(device_dir)
	{
		file=sys_device_directory_;
	}
	else
	{
		file=sys_port_directory_;
	}


	file+="/";
	file+=filename;

	return openFile(file,mode);
}

bool H4REv3Port::readInt(const std::string &filename, int &value, bool device_dir=true)
{
	FILE * file=get_fileptr_(filename, OpenFile::MODE_R, device_dir);

	if(file==NULL)
		return false;

	return readIntFromSysFile(file,value);
}

bool H4REv3Port::writeInt(const std::string &filename, int value, bool device_dir=true)
{
	FILE * file=get_fileptr_(filename, OpenFile::MODE_W, device_dir);

	if(file==NULL)
		return false;

	return writeIntToSysFile(file,value);
}
