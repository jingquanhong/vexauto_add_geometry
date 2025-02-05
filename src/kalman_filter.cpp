#include "Kalman_Filter.h"

void predict(Kalman_Filter_t *kf) {
    // 预测阶段通常不需要做任何操作，因为您没有控制输入
    // 如果有控制输入，可以在这里更新kf->estimate
}

void update(Kalman_Filter_t *kf, float measurement) {
    // 计算卡尔曼增益
    float K = kf->error / (kf->error + kf->r_measure);
    kf->estimate += K * (measurement - kf->estimate);
    kf->error = (1 - K) * kf->error;
}

float get_estimate(const Kalman_Filter_t *kf) {
    return kf->estimate;
}
