#ifndef TRANSVEH_MULTIGAIN_H
#define TRANSVEH_MULTIGAIN_H

#include "std.h"

extern uint8_t activeGainSet;
extern void transveh_multigain_init(void);
extern void transveh_multigain_periodic(void);
extern void SetGainSetA(void);
extern void SetGainSetB(void);
extern void SetGainSetHandler(void);

#endif  /* TRANSVEH_MULTIGAIN_H */
