// Unit tests for details::type_name
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/type_name.h>

#include <boost/hana/assert.hpp>
#include <boost/hana/equal.hpp>

namespace type_name_test
{

	class TestA
	{
	};

}

BOOST_HANA_CONSTANT_CHECK(
	nimbro_fsm2::detail::type_name<type_name_test::TestA>() == BOOST_HANA_STRING("type_name_test::TestA")
);
