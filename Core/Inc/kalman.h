struct kalman_state {
	float x, p, q, r, k;
};

float update_kalman_c(struct kalman_state* state, float measurement);
