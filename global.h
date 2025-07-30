typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	float prev_error;
	float I;
	float I_max;
	float I_min;
	float max;
	float min;
} PID;