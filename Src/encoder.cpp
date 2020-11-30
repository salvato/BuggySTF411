#include "encoder.h"
#include "utility.h"
#include "string.h"


// 12 CPR Quadrature Encoder ??
// Motor Gear Ratio 1:9
// Quadrature encoder mode 3 (x4 mode)
const double CountsPerTurn = 12.0*9.0*4.0;


Encoder::Encoder(TIM_HandleTypeDef *hEncodertimer)
    : htimer(hEncodertimer)
    , total(0)
{
    currElement = 0;
    memset(countHistory, 0, sizeof(countHistory));
    start();
}


void
Encoder::start() {
    if(HAL_TIM_Encoder_Start(htimer, TIM_CHANNEL_ALL))
        Error_Handler();
}


double
Encoder::read() { // in Giri Motore
    int16_t counts = int16_t(htimer->Instance->CNT);
    total += counts;
    countHistory[currElement] = counts;
    currElement++;
    currElement = currElement % N_AVG;
    double avgCounts = 0;
    for(int i=0; i<N_AVG; i++) {
        avgCounts += countHistory[i];
    }
    return avgCounts/(CountsPerTurn*N_AVG);
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
    countHistory[currElement] = counts;
    currElement++;
    currElement = currElement % N_AVG;
    double avgCounts = 0;
    for(int i=0; i<N_AVG; i++) {
        avgCounts += countHistory[i];
    }
    return avgCounts/(CountsPerTurn*N_AVG);
}
