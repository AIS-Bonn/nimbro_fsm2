// Utilities for querying the list of types in std::variant
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef NIMBRO_FSM2_VARIANT_MEMBERSHIP_H
#define NIMBRO_FSM2_VARIANT_MEMBERSHIP_H

#include <variant>

namespace nimbro_fsm2
{

namespace detail
{

template<typename T, typename Variant>
struct variant_has_type;

template<typename T, typename... Us>
struct variant_has_type<T, std::variant<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

template<typename T, typename... Us>
constexpr bool variant_has_type_v = variant_has_type<T, Us...>::value;

}

}

#endif
