struct queue {
	int start, end, size;
	float *array;
};

void queue_add(struct queue* q, float val);

float queue_average(struct queue *q);


