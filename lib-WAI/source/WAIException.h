#ifndef WAI_EXCEPTION
#define WAI_EXCEPTION

#include <exception>
#include <string>
#include "WAIHelper.h"

namespace WAI
{

class WAI_API WAIException : public std::runtime_error
{
    public:
    //! creates an exception with specific cause
    WAIException(const std::string cause, const std::string file, const int line);

    private:
    const std::string _message;
};

}

#endif
