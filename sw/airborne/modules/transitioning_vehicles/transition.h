#ifndef TRANSVEH_TRANSITION_H
#define TRANSVEH_TRANSITION_H

#include "std.h"

#if !TRANSITION_DEG_PER_TICK
#define TRANSITION_DEG_PER_TICK 0.45f
#endif

enum TransitionState { HOVER,
                       PREP_FOR_TRANSITION_ROTATE_TO_FORWARD,
                       PREP_FOR_TRANSITION_ROTATE_TO_HOVER,
                       PREP_FOR_TRANSITION_CHANGE_THRUST_RATIO_TO_FORWARD,
                       PREP_FOR_TRANSITION_CHANGE_THRUST_RATIO_TO_HOVER,
                       PREP_FOR_TRANSITION_HOVER_PROPS_OFF,
                       PREP_FOR_TRANSITION_HOVER_PROPS_ON,
                       FORWARD};

extern enum TransitionState transition_state;
extern enum TransitionState required_transition_state;

extern void transveh_set_sp_by_mode_init(void);
extern void transveh_set_sp_by_mode_periodic(void);
extern void transveh_smooth_transition(int32_t desired, int32_t *actual);

void PrepForTransitionToForward(void);
void PrepForTransitionToHover(void);


#endif
