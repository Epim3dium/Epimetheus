#ifndef EPI_DEBUG_H
#define EPI_DEBUG_H
#include "log.hpp"
namespace epi {

#if EPI_DEBUG == 1

#   define EPI_ASSERT(x) (assert(x))
#   define EPI_DEBUGCALL(x) x

#else

#   define EPI_ASSERT(x) ((void)sizeof(x))
#   define EPI_DEBUGCALL(x) ;

#endif

}
#endif //EPI_DEBUG_H
