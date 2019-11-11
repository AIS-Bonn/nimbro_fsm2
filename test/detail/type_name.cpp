// Unit tests for details::type_name
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include <type_name/type_name.hpp>

namespace type_name_test
{
	namespace states
	{
		class TestA
		{
		};
	}

	class Probe;
}

TEST_CASE("type_name")
{
	constexpr auto Name = type_name::type_name_v<type_name_test::states::TestA>;
	static_assert(Name == "type_name_test::states::TestA");

	constexpr auto ns = type_name::namespace_of_v<type_name_test::states::TestA>;
	static_assert(ns == "type_name_test::states");

	constexpr auto relative = type_name::relative_name_v<
		type_name_test::states::TestA, type_name_test::Probe
	>;
	static_assert(relative == "states::TestA");
}
