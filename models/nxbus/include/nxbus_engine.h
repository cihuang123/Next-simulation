#ifndef MODELS_NXBUS_INCLUDE_NXBUS_ENGINE_H_
#define MODELS_NXBUS_INCLUDE_NXBUS_ENGINE_H_
/********************************* TRICK HEADER *******************************
PURPOSE:
      Next Simulation Software Bus
LIBRARY DEPENDENCY:
      (
          (../src/nxbus_engine.c)
      )
PROGRAMMERS:
      (((Dung-Ru Tsai) () () () ))
*******************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
typedef enum _ENUM_NXBUS_DATA_T {
    NXBUS_DOUBLE,
    NXBUS_UINT32,
    NXBUS_UINT16,
    NXBUS_UINT8,
    NXBUS_INT32,
    NXBUS_INT16,
    NXBUS_INT8,
    NXBUS_NUM_OF_DATA_TYPE
}ENUM_NXBUS_DATA_T;

#ifdef __cplusplus
extern "C" {
#endif

void nxbus_init(void);
int nxbus_mset(ENUM_NXBUS_DATA_T data_type, const char *key_name, size_t dimension, ...);
int nxbus_mget(ENUM_NXBUS_DATA_T data_type, const char *key_name, size_t dimension, ...);
void nxbus_deinit(void);

#ifdef __cplusplus
}
#endif

#endif  // MODELS_NXBUS_INCLUDE_NXBUS_ENGINE_H_
