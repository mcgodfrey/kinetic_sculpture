#define NUM_SERVOS 1
#define MAN_N 16

typedef struct {
	servo all_servos[NUM_SERVOS];
} program;


typedef struct {
	int n;
	long times[MAX_N];
	unsigned char positions[MAX_N];
} servo;