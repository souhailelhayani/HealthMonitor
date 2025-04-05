#include "kalman.h"

//takes one current measurement, and returns its approximation
float update_kalman_c(struct kalman_state* state, float measurement)
{
	state->p = state->p + state->q;

	if(state->p + state->r == 0) return 2;

	state->k = state->p / (state->p + state->r);

	state->x = state->x + (state->k * (measurement - state->x));

	state->p = (1 - state->k) * state->p;

	//TODO check overflow
	int flags = __get_FPSCR();
	if((flags & 268435456) != 0) {
		return 1;
	}

	return state->x;
}
