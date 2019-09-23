#include "WAIException.h"
#include <sstream>

namespace WAI
{
std::string toMessage(const std::string& cause, const std::string& file, const int line)
{
    std::stringstream ss;
    ss << "WAIException catched on " << file << "(" << std::to_string(line) << ")"
       << ":" << std::endl;
    ss << cause;
    return ss.str();
}

WAIException::WAIException(const std::string cause, const std::string file, const int line)
  : std::runtime_error(toMessage(cause, file, line).c_str())
{
}

}
