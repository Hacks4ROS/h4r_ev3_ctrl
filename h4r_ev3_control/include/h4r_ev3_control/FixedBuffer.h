/*
(c) 2013 +++ Filip Stoklas, aka FipS, http://www.4FipS.com +++
THIS CODE IS FREE - LICENSED UNDER THE MIT LICENSE
ARTICLE URL: http://forums.4fips.com/viewtopic.php?f=3&t=1075

Some name changes and port to Linux by Christian Holl (http://github.com/Hacks4ROS)
*/

/*
MIT-License

Copyright (c) 2013 Filip Stoklas
Copyright (c) 2015 Christian Holl

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef FIXEDBUFFER_H_
#define FIXEDBUFFER_H_


#include <array>
#include <cassert>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>


namespace FixedBuffer
{

/// A fixed-size string buffer (typically stack-allocated).
template <size_t N>
class FixedBuffer
{
public:

    /// Constructs from a C string if provided, or sets to "" if NULL passed.
    FixedBuffer(const char *cstr = NULL)
    {
        static_assert(N > 0, "FixedBuffer has zero size!");

        if(cstr)
        {
            assert(strlen(cstr) < N && "FixedBuffer too small!");
            strcpy(_data.data(), cstr);
        }
        else
        {
            _data[0] = '\0';
        }
    }

    /// Constructs from a FixedBuffer of a bigger or equal static size.
    template <size_t M>
    FixedBuffer(const FixedBuffer<M> &rhs)
    {
        static_assert(M <= N, "FixedBuffer too small!");
        strcpy(_data.data(), rhs.cstr());
    }

    /// Copies from a C string if provided, or sets to "" if nullptr passed.
    FixedBuffer & operator = (const char *cstr)
    {
        static_assert(N > 0, "FixedBuffer of zero size!");

        if(cstr)
        {
            assert(strlen(cstr) < N && "Buffer size too small!");
            strcpy(_data.data(), cstr);
        }
        else
        {
            _data[0] = '\0';
        }

        return *this;
    }


    FixedBuffer &operator = (const std::string& str)
    {
    	return operator =(str.c_str());
    }

    /// Copies from a FixedBuffer of a bigger or equal static size.
    template <size_t M>
    FixedBuffer & operator = (const FixedBuffer<M> &rhs)
    {
        static_assert(M <= N, "FixedBuffer too small!");
        strcpy(_data.data(), rhs.cstr());
        return *this;
    }

    /// Returns a C string (always valid).
    const char * c_str() const { return _data.data(); }

    /// Formats a string in a good old printf style.
    void format(const char *format, ...)
    {
        va_list args;
        va_start(args, format);
        // if truncated, '\0' is automatically appended
        vsnprintf(_data.data(), N, format, args);
        va_end(args);
    }

private:

    std::array<char, N> _data;
};


}



#endif /* FIXEDBUFFER_H_ */
