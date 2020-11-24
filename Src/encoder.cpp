#include "encoder.h"
#include "string.h" // for memset()


// 12 CPR Quadrature Encoder ??
// Motor Gear Ratio 1:9
// Quadrature encoder mode 3 (x4 mode)
const double CountsPerTurn = 12.0*9.0*4.0;


Encoder::Encoder(TIM_HandleTypeDef *hEncodertimer)
    : htimer(hEncodertimer)
    , total(0)
{
}


void
Encoder::start() {
    HAL_TIM_Encoder_Start(htimer, TIM_CHANNEL_ALL);
}


double
Encoder::read() { // in Giri Motore
    int16_t counts = int16_t(htimer->Instance->CNT);
    total += counts;
    return double(counts)/CountsPerTurn;
}


void
Encoder::reset() {
    htimer->Instance->CNT = 0;
}


int32_t
Encoder::readTotal() {
    return total;
}


void
Encoder::resetTotal() {
    total = 0;
}


int32_t
Encoder::readAndResetTotal() {
    int32_t saveTotal = total;
    total = 0;
    return saveTotal;
}


double
Encoder::readAndReset() { // in Giri Motore
    int16_t counts = int16_t(htimer->Instance->CNT);
    total += counts;
    htimer->Instance->CNT = 0;
    return double(counts)/CountsPerTurn;
}
