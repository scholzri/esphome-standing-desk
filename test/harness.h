#pragma once
// Minimal dependency-free test harness. Deliberately not a framework: it needs
// to compile anywhere g++ exists, with no downloads.
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace th {

struct Case {
  const char *name;
  std::function<void()> fn;
};

inline std::vector<Case> &cases() {
  static std::vector<Case> c;
  return c;
}

inline int &failures() {
  static int f = 0;
  return f;
}

struct Reg {
  Reg(const char *n, std::function<void()> f) { cases().push_back({n, f}); }
};

inline void fail(const char *file, int line, const std::string &msg) {
  std::printf("    FAIL %s:%d: %s\n", file, line, msg.c_str());
  failures()++;
}

inline int run_all() {
  int failed_cases = 0;
  for (auto &c : cases()) {
    int before = failures();
    std::printf("  %s\n", c.name);
    c.fn();
    if (failures() > before)
      failed_cases++;
  }
  std::printf("%s: %zu cases, %d failed\n", failed_cases ? "FAILED" : "PASSED",
              cases().size(), failed_cases);
  return failed_cases ? 1 : 0;
}

}  // namespace th

#define TEST(name)                          \
  static void name();                       \
  static th::Reg reg_##name(#name, name);   \
  static void name()

#define CHECK(cond)                                            \
  do {                                                         \
    if (!(cond))                                               \
      th::fail(__FILE__, __LINE__, "CHECK(" #cond ")");        \
  } while (0)

#define CHECK_EQ(a, b)                                                        \
  do {                                                                        \
    auto _a = (a);                                                            \
    auto _b = (b);                                                            \
    if (!(_a == _b))                                                          \
      th::fail(__FILE__, __LINE__,                                            \
               "CHECK_EQ(" #a ", " #b ") got " + std::to_string(_a) +         \
                   " vs " + std::to_string(_b));                              \
  } while (0)
