/*
 * This file (syshelpers.h) is part of h4r_ev3_control.
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
#ifndef SYSHELPERS_H_
#define SYSHELPERS_H_


#include <string>
#include <map>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <dirent.h>

#include <h4r_ev3_control/FixedBuffer.h>
#include <h4r_ev3_control/StringEnum.h>

namespace ev3_control
{

/**
 * A file name buffer without memory allocation
 */
typedef FixedBuffer::FixedBuffer<256> FileNameBuffer;

/**
 * Check if file exists
 * @param path Path to be checked of access
 * @return True if it exists
 */
bool pathExists(const char* path);


/**
 * Write int value into a opened (sys) file.
 * @param fileptr The file pointer from fopen
 * @param value	The value
 * @return	true if everything was successful, false otherwise
 */
bool writeIntToSysFile(FILE *fileptr,int value);

/**
 * Read integer value from (sys) file
 * @param fileptr The file pointer from fopen
 * @param[out] value	The value in the file only valid if successful
 * @return	true if everything was successful, false otherwise
 */
bool readIntFromSysFile(FILE *fileptr, int &value);

/**
 * Write a String in a map to a (sys) file
 * @param fileptr The file pointer
 * @param strmap The map with the strings and the keys
 * @param key The key (enum value)
 * @return True if everything was successful, false otherwise
 */
template <typename T>
bool writeKeyToSysFile(FILE *fileptr, const StringEnum<T> &strmap, T key)
{

		char outbuf[256];
		int len=strlen(strmap[key]);
		strncpy(outbuf, strmap[key], len);

	   int wrote=fwrite(outbuf,1,len,fileptr);

	   if(fflush(fileptr))
		   return false;
	   rewind(fileptr);

	   return wrote==len;
}

/**
 * Reads a sysfile and checks if the string exists in a map and returns the value
 * @param fileptr
 * @param strmap
 * @param value The value for the string
 * @return True if everything was ok, false otherwise
 */
template <typename T>
bool readKeyFromSysFile(FILE *fileptr,const StringEnum<T> &strmap, T &value)
{
	   fflush(fileptr);
	   rewind(fileptr);
	   int64_t value_out;

	   char buffer[256];
	   char *buf=&buffer[0];
	   char **bufptr=&buf;

	   bool negative;
	   ssize_t read;
	   size_t len=256;
	   int l=0;
	   bool ok=false;
       while ((read = getline(bufptr, &len, fileptr)) != -1) {
    	   if(l==0)
    	   {
    		   buffer[read-1]=0x00;//remove linefeed!
    		   value=strmap[buffer];
    		   if(value<0)
    		   {
    			   return false;
    		   }
    		   else
    		   {
    			   ok=true;
    		   }

    	   }
    	   else
    	   {
    		   if(l!=1 || read!=1)
    		   {
    			   return false;
    		   }
    	   }

    	   return ok;
       }


	   fscanf(fileptr,"%[^\n]",buffer);
	   return true;
}

/**
 * Searches for the same file with the same one line string content in multiple subdirectories
 * Used for identifying a device by name file in sys directories
 * @param parent The class directory to search
 * @param file The file to search in the directories
 * @param content The content which should match
 * @param[out] match_dir The first matching directory
 * @return True if found, false otherwise
 */
bool matchFileContentInEqualSubdirectories(const char* parent,
		                                   const char* file,
										   const char* content,
										   FileNameBuffer &match_dir);

} /*ev3_control*/
#endif /* SYSHELPERS_H_ */
