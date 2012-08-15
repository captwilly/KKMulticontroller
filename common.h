#ifndef COMMON_H_
#define COMMON_H_

#include <avr/io.h>
#include <avr/eeprom.h>
#include "typedefs.h"
#include "io_cfg.h"
#include "config.h"


// Safe MAX and MIN macro implementation got from:
//  http://gcc.gnu.org/onlinedocs/gcc-3.4.6/gcc/Typeof.html
#define MAX(a,b)    ({                                                      \
        __typeof__ (a) _a = (a);                                            \
        __typeof__ (b) _b = (b);                                            \
        _a > _b ? _a : _b;                                                  \
    })
#define MIN(a,b)    ({                                                      \
        __typeof__ (a) _a = (a);                                            \
        __typeof__ (b) _b = (b);                                            \
        _a < _b ? _a : _b;                                                  \
    })

#define FOREVER     while(true)

#define DEBUG(x)	{														\
        eeprom_write_block(&x, (void*)512, sizeof(x));                      \
        FOREVER{}                                                           \
    }

#endif /* COMMON_H_ */
