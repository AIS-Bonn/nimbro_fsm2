// Unit tests for details::variant_has_type
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/variant_membership.h>

#include <catch_ros/catch.hpp>

#include <variant>

namespace
{

class TestA {};
class TestB {};
class TestC {};

}

TEST_CASE("variant_membership")
{
	using Variant = std::variant<TestA, TestB>;
	static_assert(nimbro_fsm2::detail::variant_has_type_v<TestA, Variant>);
	static_assert(nimbro_fsm2::detail::variant_has_type_v<TestB, Variant>);
	static_assert(!nimbro_fsm2::detail::variant_has_type_v<TestC, Variant>);
}
