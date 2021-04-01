#ifndef __FILTER_H
#define __FILTER_H

#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "utils.h"

/** Second Order IIR Lowpass Butterworth Filter
 *
 * using biquad filter with bilinear z transform
 *
 * http://en.wikipedia.org/wiki/Digital_biquad_filter
 * http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/utils/interface/filter.h
 *
 * Laplace continious form:
 *
 *                 1
 * H(s) = -------------------
 *        s^2 + s/Q + 1
 *
 *
 * Polynomial discrete form:
 *
 *        b0 + b1 z^-1 + b2 z^-2
 * H(z) = ----------------------
 *        a0 + a1 z^-1 + a2 z^-2
 *
 * with:
 *  a0 = 1
 *  a1 = 2*(K^2 - 1) / (K^2 + K/Q + 1)
 *  a2 = (K^2 - K/Q + 1) / (K^2 + K/Q + 1)
 *  b0 = K^2 / (K^2 + K/Q + 1)
 *  b1 = 2*b0
 *  b2 = b0
 *  K = tan(pi*Fc/Fs) ~ pi*Fc/Fs = Ts/(2*tau), K=tan(ω_cut * Ts / 2) = tan((2pi * Fc) / (2 * Fs))
 *  Fc: cutting frequency, ω_cut = 2pi * Fc
 *  Fs: sampling frequency， Ts = 1 / Fs
 *  Ts: sampling period
 *  tau: time constant (tau = 1/(2*pi*Fc))
 *  Q: gain at cutoff frequency, 1 / sqrt(2)
 *
 * Note that b[0]=b[2], so we don't need to save b[2]
 * w[n] = x[n] - a1 * w[n-1] - a2 * w[n-2]
 * y[n] = b0 * w[n] + b1 * w[n-1] + b2 * w[n-2]
 */

typedef struct {
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float delay_element_1;
    float delay_element_2;
} Lowpass2Data;

void Lowpass2Init(Lowpass2Data* lowpass2_data, float sample_freq, float cutoff_freq);
float Lowpass2Apply(Lowpass2Data* lowpass2_data, float sample);
float Lowpass2Reset(Lowpass2Data* lowpass2_data, float sample);

void Lowpass2Apply3Axis(Lowpass2Data* lowpass2_data, float *data);
#endif /* __FILTER_H */
