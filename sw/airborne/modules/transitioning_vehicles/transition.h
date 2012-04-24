#ifndef TRANSVEH_SET_SP_BY_MODE_H
#define TRANSVEH_SET_SP_BY_MODE_H

#include "std.h"

#if !TRANSITION_DEG_PER_TICK
#define TRANSITION_DEG_PER_TICK 0.45f
#endif


extern void transveh_set_sp_by_mode_init(void);
extern void transveh_set_sp_by_mode_periodic(void);
extern void transveh_smooth_transition(int32_t desired, int32_t *actual);


#endif  /* TRANSVEH_SET_SP_BY_MODE_H */
