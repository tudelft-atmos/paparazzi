#ifndef TRANSVEH_SET_SP_BY_MODE_H
#define TRANSVEH_SET_SP_BY_MODE_H

#include "std.h"

#if !TRANSITION_DEG_PER_SEC
#define TRANSITION_DEG_PER_SEC 45
#endif

#if !TRANSITION_DEG_PER_TICK
#define TRANSITION_DEG_PER_TICK 0.45
#endif

extern void transveh_set_sp_by_mode_init(void);
extern void transveh_set_sp_by_mode_periodic(void);

#endif  /* TRANSVEH_SET_SP_BY_MODE_H */
