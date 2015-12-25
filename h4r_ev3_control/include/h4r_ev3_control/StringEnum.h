/*
 * This file (StringEnum.h) is part of h4r_ev3_control.
 * Date: 18.12.2015
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

#ifndef STRINGENUM_H_
#define STRINGENUM_H_


#include <vector>
#include <algorithm>
#include <assert.h>
#include <string.h>
#include <inttypes.h>

#include <iostream>

namespace ev3_control
{

using namespace std;
template <class ENUM>
class StringEnum
{

	class Entry
	{

	public:
		Entry(unsigned key, const char* str)
		:str_(str)
		,key_(key)
		{}

		bool operator<(const Entry& e) const
		{
			return str_<e.str_;
		}

		const std::string &getString() const
		{
			return str_;
		}

		int getKey() const
		{
			return key_;
		}




	protected:
		string str_;
		int key_;



	};

	bool final_;
	unsigned string_count_;
	std::vector<Entry>	key_from_string_;
	std::vector<string> string_from_key_;

public:

	StringEnum()
	:string_count_(0)
	,final_(false)
	{}

	void insert(const char *str)
	{
		assert(final_==false && "already finalized!");

		key_from_string_.push_back(Entry(string_count_,str));
		string_from_key_.push_back(string(str));
		string_count_++;
	}

	void finalize()
	{
		std::sort(key_from_string_.begin(), key_from_string_.end());
		final_=true;
	}

	const char* operator[](ENUM key) const
	{
		assert(key < string_count_ && "Key out of range!");
		return string_from_key_[key].c_str();
	}

	ENUM operator[](const char* str) const
	{
		assert(final_ && "StringEnum not finalized!");

		cout<<"str:"<<str<<endl;
		for (int i = 0; i < key_from_string_.size(); ++i)
		{


			cout<<"Str No: "<<i<<":"<<key_from_string_[i].getString().c_str();
			if(strcmp(key_from_string_[i].getString().c_str(),str)==0)
			{
				cout<<"<---"<<endl;
				return (ENUM)key_from_string_[i].getKey();
			}
			cout<<endl;
		}
		return (ENUM)-1;
	}

private:
};


}/* namespace */
#endif /* STRINGENUM_H_ */
