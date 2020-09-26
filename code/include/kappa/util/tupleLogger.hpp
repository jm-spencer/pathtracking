#pragma once

// Modified from: https://stackoverflow.com/questions/1198260/how-can-you-iterate-over-the-elements-of-an-stdtuple

#include <tuple>
#include <utility>
#include <ostream>

template<std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp) - 1, void>::type
printTuple(const std::tuple<Tp...> &t, std::ostream &iout, const std::string &separator) {
  iout << std::get<I>(t);
}

template<std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I < sizeof...(Tp) - 1, void>::type
printTuple(const std::tuple<Tp...> &t, std::ostream &iout, const std::string &separator) {
  iout << std::get<I>(t) << separator;
  printTuple<I + 1, Tp...>(t, iout, separator);
}
