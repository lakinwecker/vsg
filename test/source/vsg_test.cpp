#include "vsg/vsg.hpp"

auto main() -> int
{
  auto const result = name();

  return result == "vsg" ? 0 : 1;
}
