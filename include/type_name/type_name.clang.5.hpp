#ifndef ADAM_BADURA_TYPE_NAME_TYPE_NAME_CLANG_5_HPP
#define ADAM_BADURA_TYPE_NAME_TYPE_NAME_CLANG_5_HPP

#include <string_view>

#include <iosfwd>

namespace type_name
{

template<unsigned int Size>
class StaticString
{
public:
	constexpr explicit StaticString(const char* src) noexcept
	 : buf{}
	{
		for(unsigned int i = 0; i < Size; ++i)
			buf[i] = src[i];
		buf[Size] = 0;
	}

	constexpr const char* data() const noexcept
	{
		return buf;
	}
	constexpr const char* c_str() const noexcept
	{
		return buf;
	}

	constexpr std::size_t rfind(const std::string_view& text) const noexcept
	{
		return std::string_view{buf, Size}.rfind(text);
	}

	template<std::size_t offset, std::size_t length>
	constexpr auto substr() const noexcept
	{
		return StaticString<length>(buf + offset);
	}

	constexpr std::size_t size() const noexcept
	{ return Size; }

	constexpr bool operator==(const StaticString<Size>& other) const noexcept
	{
		for(std::size_t i = 0; i < Size; ++i)
		{
			if(buf[i] != other.buf[i])
				return false;
		}
		return true;
	}

	constexpr bool operator==(const std::string_view& other) const noexcept
	{
		return view() == other;
	}

	constexpr std::string_view view() const noexcept
	{
		return std::string_view{buf, Size};
	}
private:
	char buf[Size+1];
};

template<unsigned int Size>
std::ostream& operator<<(std::ostream& stream, const StaticString<Size>& str)
{
	stream << str.data();
	return stream;
}

template<typename T>
struct type_name
{
private:
	static constexpr auto get() noexcept
	{
		constexpr std::string_view full_name{ __PRETTY_FUNCTION__ };
		constexpr std::string_view left_marker{ "[T = " };
		constexpr std::string_view right_marker{ "]" };

		constexpr auto left_marker_index = full_name.find(left_marker);
		static_assert(left_marker_index != std::string_view::npos);
		constexpr auto start_index = left_marker_index + left_marker.size();
		constexpr auto end_index = full_name.find(right_marker, left_marker_index);
		static_assert(end_index != std::string_view::npos);
		constexpr auto length = end_index - start_index;

		return StaticString<length>(full_name.data() + start_index);
	}

public:
	static constexpr auto value = get();
};

template<typename T>
inline constexpr auto type_name_v = type_name<T>::value;

}

#endif
