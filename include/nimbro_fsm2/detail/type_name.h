// Compile-time type names
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_TYPE_NAME_H
#define NIMBRO_FSM2_DETAIL_TYPE_NAME_H

#include <cstdlib>

#include <boost/hana/string.hpp>
#include <boost/hana/size.hpp>
#include <boost/hana/drop_while.hpp>
#include <boost/hana/reverse.hpp>
#include <boost/hana/not_equal.hpp>
#include <boost/hana/minus.hpp>
#include <boost/hana/greater_equal.hpp>
#include <boost/hana/greater.hpp>
#include <boost/hana/drop_front.hpp>
#include <boost/hana/tuple.hpp>
#include <boost/hana/slice.hpp>

namespace nimbro_fsm2
{

namespace detail
{

// We use a pretty ugly hack here to get a string containing the type name:
// Exploit that __PRETTY_FUNCTION__ shows type information.
// inspired by ctti: https://github.com/Manu343726/ctti
// and hana::experimental::type_name

// Until gcc supports constexpr __PRETTY_FUNCTION__, the code below is very
// ugly.
// Bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=66639

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

// https://stackoverflow.com/a/53798139
template <typename S, unsigned long ...i>
constexpr auto reverse_string_impl(S s, std::index_sequence<i...>)
{
	namespace hana = boost::hana;
	constexpr unsigned long n = sizeof...(i);
	constexpr char const* c = hana::to<char const*>(s);
	return hana::make_string(hana::char_c<c[n - i - 1]>...);
	// would be better but assumes stuff about the impl of hana::string
	//return hana::string<c[n - i - 1]...>{};
}
template <typename S>
constexpr auto reverse_string(S)
{
	namespace hana = boost::hana;
	return reverse_string_impl(S{},
		std::make_index_sequence<hana::length(S{})>{});
}

template <typename Iterable, typename T>
constexpr auto index_of(Iterable const& iterable, T const& element)
{
	namespace hana = boost::hana;
	auto size = decltype(hana::size(iterable)){};
	auto dropped = decltype(hana::size(
		hana::drop_while(iterable, hana::not_equal.to(element))
	)){};
	return size - dropped;
}

template <typename Name>
constexpr auto namespace_of(Name name)
{
	namespace hana = boost::hana;

	auto reverse_name = reverse_string(name);
	auto rev_idx = index_of(reverse_name, hana::char_c<':'>);
	auto rev_ns = hana::drop_front(reverse_name, rev_idx + hana::size_c<2>);
	return reverse_string(rev_ns);
}

template <typename T>
constexpr auto type_name()
{
	return type_name_impl1<T>(std::make_index_sequence<get_name_length<T>()>{});
}

template<class A, class B>
constexpr bool is_prefix(A a, B b)
{
	namespace hana = boost::hana;

	auto sizeA = decltype(hana::size(a)){};
	auto sizeB = decltype(hana::size(b)){};

	if constexpr (sizeA > sizeB)
		return false;

	auto bTuple = hana::to_tuple(b);
	auto slice = hana::slice_c<0, hana::value(sizeA)>(bTuple);
	auto sliceStr = hana::unpack(slice, hana::make_string);
	return hana::value(a == sliceStr);
}

template<class Name, class To>
constexpr auto relative_name_string(Name name, To to)
{
	namespace hana = boost::hana;

	auto nsPrefix = to + BOOST_HANA_STRING("::");

	if constexpr(is_prefix(nsPrefix, name))
	{
		return hana::drop_front(name, hana::size(nsPrefix));
	}
	else
		return name;
}

template<class T, class To>
constexpr auto relative_name(To to)
{
	return relative_name_string(type_name<T>(), to);
}

}

}

#endif
