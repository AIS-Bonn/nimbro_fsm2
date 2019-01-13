// Compile-time type names
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_TYPE_NAME_H
#define NIMBRO_FSM2_DETAIL_TYPE_NAME_H

#include <cstdlib>

#include <boost/hana/string.hpp>

namespace nimbro_fsm2
{

namespace detail
{

template<class T>
constexpr std::size_t get_name_length()
{
	// ... get_name<T>() [with T = MyType]
	const char* p = __PRETTY_FUNCTION__;

	// Skip to "= "
	while(*p && *p++ != '=');
	while(*p == ' ')
		p++;

	// Search for end
	// NOTE: Due to some gcc bug / behavior, we get the __PRETTY_FUNCTION__
	//   of get_name_idx<T, idx> here sometimes - so also check for ';'
	//   as the delimiter between T and idx.
	if(*p)
	{
		const char* p2 = p;
		int count = 1;
		for (;;++p2)
		{
			switch (*p2)
			{
			case '[':
				++count;
				break;
			case ']':
				--count;
				if (!count)
					return std::size_t(p2-p);
				break;
			case ';':
				if(count == 1)
					return std::size_t(p2-p);
				break;
			case 0:
				return 0;
			}
		}
	}

	return 0;
}

template<class T, int idx>
constexpr char get_name_idx()
{
	// ... get_name<T>() [with T = MyType]
	const char* p = __PRETTY_FUNCTION__;
	std::size_t off = 0;

	// Skip to "= "
	while(p[off] && p[off++] != '=');
	while(p[off] == ' ')
		off++;

	if(!p[off])
		return '?';

	off += idx;

	if(off >= sizeof(__PRETTY_FUNCTION__))
		return '?';

	return p[off];
}

template <typename T, std::size_t ...i>
constexpr auto type_name_impl1(std::index_sequence<i...>)
{
	return boost::hana::string<get_name_idx<T, i>()...>{};
}

template <typename T>
constexpr auto type_name()
{
	return type_name_impl1<T>(std::make_index_sequence<get_name_length<T>()>{});
}

}

}

#endif
