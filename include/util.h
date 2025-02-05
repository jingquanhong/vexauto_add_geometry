extern float reduce_0_to_360(float angle);

extern float reduce_negative_180_to_180(float angle);

extern float reduce_negative_90_to_90(float angle);

extern float to_rad(float angle_deg);

extern float to_deg(float angle_rad);

extern float clamp(float input, float min, float max);

extern bool is_reversed(double input);

extern float to_volt(float percent);

extern int to_port(int port);

extern float deadband(float input, float width);