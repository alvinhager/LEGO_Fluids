/** Custom assert macro defined in order to debug programs and conditions more easily */
#ifndef CUSTOM_ASSERT_AH
#include <cassert>
#define CUSTOM_ASSERT(condition)                                                    \
    {                                                                               \
        if (!(condition))                                                           \
        {                                                                           \
            std::cerr << "Custom assertion has failed: " << #condition << std::endl \
                      << "in file " << __FILE__ << std::endl                        \
                      << "at function " << __FUNCTION__ << std::endl                \
                      << "at line " << __LINE__ << std::endl;                       \
            abort();                                                                \
        }                                                                           \
    }
#else
#define CUSTOM_ASSERT(condition) (condition)
#endif
