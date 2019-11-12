#ifndef ADAM_BADURA_TYPE_NAME_TYPE_NAME_HPP
#define ADAM_BADURA_TYPE_NAME_TYPE_NAME_HPP

#if defined(__clang__)
#	include "type_name.clang.hpp"
#elif defined(__GNUC__)
#	include "type_name.gcc.hpp"
#elif defined(_MSC_VER)
#	include "type_name.msvc.hpp"
#else
#	error Unsupported compiler.
#endif

namespace type_name
{

template<typename T>
struct NamespaceOf
{
private:
	static constexpr auto get() noexcept
	{
		constexpr auto fullName = type_name_v<T>;
		constexpr std::string_view sep{ "::" };

		constexpr auto idx = fullName.rfind(sep);

		if constexpr(idx == std::string_view::npos)
			return std::string_view{};
		else
#if defined(__clang__)
			return fullName.template substr<0, idx>();
#else
			return fullName.substr(0, idx);
#endif
	}
public:
	static constexpr auto value{ get() };
};

template<typename T>
inline constexpr auto namespace_of_v = NamespaceOf<T>::value;

template<typename T, typename Probe>
struct RelativeName
{
private:
	static constexpr auto get() noexcept
	{
		constexpr auto fullName = type_name_v<T>;
		constexpr auto prefix = NamespaceOf<Probe>::value;

#if defined(__clang__)
		if constexpr(!prefix.empty() && fullName.template substr<0, prefix.size()>() == prefix)
			return fullName.template substr<prefix.size() + 2, fullName.size() - prefix.size() - 2>();
#else
		if(!prefix.empty() && fullName.substr(0, prefix.size()) == prefix)
			return fullName.substr(prefix.size() + 2);
#endif
		else
			return fullName;
	}
public:
	static constexpr auto value{ get() };
};

template<typename T, typename Probe>
inline constexpr auto relative_name_v = RelativeName<T, Probe>::value;

}

#endif
