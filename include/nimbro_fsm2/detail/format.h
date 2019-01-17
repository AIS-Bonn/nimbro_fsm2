// Pre-compiled fmt::vformat
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_DETAIL_FORMAT_H
#define NIMBRO_FSM2_DETAIL_FORMAT_H

#include <fmt/format.h>

namespace nimbro_fsm2
{
namespace detail
{

using Buffer = fmt::internal::basic_buffer<char>;

/**
 * @brief Format into buffer and append newline
 *
 * @internal
 **/
void vformat_to_nl(Buffer& buf, fmt::string_view format_str, fmt::format_args args);

}
}

#endif
