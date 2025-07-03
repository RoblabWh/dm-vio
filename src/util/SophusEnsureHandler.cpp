namespace Sophus
{
  void ensureFailed(char const *function, char const *file, int line,
                    char const *description)
  {
    // DM-VIO assumes an older version of Sophus without SOPHUS_ENSURE checks so just disable them
    (void)function;
    (void)file;
    (void)line;
    (void)description;
  }
} // namespace Sophus
