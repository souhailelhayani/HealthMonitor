#include "circular_q.h"

void queue_add(struct queue* q, float val) {
	q->array[q->start] = val;
	q->start = (q->start + 1) % q->size;
	q->end = (q->end + 1) % q->size;
}

float queue_average(struct queue *q) {
	float sum = 0;

	int index = q->start;
	while(index != q->end) {
		sum += q->array[index];

		index = (index + 1) % q->size;
	}
	sum += q->array[index]; //for the last value, since while loop stops there

	return sum / (float)q->size;
}
