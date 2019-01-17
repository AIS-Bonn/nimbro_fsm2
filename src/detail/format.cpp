// Pre-compiled fmt::vformat
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/detail/format.h>

namespace nimbro_fsm2
{
namespace detail
{

void vformat_to_nl(Buffer& buf, fmt::string_view format_str, fmt::format_args args)
{
	fmt::vformat_to(buf, format_str, args);
	fmt::vformat_to(buf, "\n", fmt::make_format_args());
}

}
}
