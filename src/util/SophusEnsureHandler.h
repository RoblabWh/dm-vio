#ifndef DMVIO_SOPHUSENSUREHANDLER_H
#define DMVIO_SOPHUSENSUREHANDLER_H

#include <sophus/common.hpp>

namespace Sophus
{
  struct EnsureFailed : public std::runtime_error
  {
    EnsureFailed(const std::string &message)
        : std::runtime_error(message)
    {
    }
    EnsureFailed(const char *message)
        : std::runtime_error(message)
    {
    }
  };
} // namespace Sophus

#endif // DMVIO_SOPHUSENSUREHANDLER_H
