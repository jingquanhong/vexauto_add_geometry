#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
typedef struct{
    float estimate=0.0;
    float error=0.0;
    float total=0.0;
    float q_angle;//过程噪声
    float r_measure;//测量噪声
    
}Kalman_Filter_t;
void predict(Kalman_Filter_t *kf);
void update(Kalman_Filter_t *kf,float measurement);
float get_estimate(const Kalman_Filter_t *kf);
#endif 