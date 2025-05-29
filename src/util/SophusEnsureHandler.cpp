#include "SophusEnsureHandler.h"

namespace Sophus
{
  void ensureFailed(char const *function, char const *file, int line,
                    char const *description)
  {
    throw EnsureFailed(fmt::format("Sophus ensure failed in function '{}', file '{}', line {}.\n{}\n",
                                   file, function, line, description));
  }
} // namespace Sophus
