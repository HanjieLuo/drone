#include "filter.h"

void Lowpass2Init(Lowpass2Data* lowpass2_data, float sample_freq, float cutoff_freq) {
    if (lowpass2_data == NULL || cutoff_freq <= 0.0f) {
        return;
    }

    // ω_cut = 2pi * cutoff_freq
    // Ts = 1 / sample_freq
    // K=tan(ω_cut * Ts / 2) = tan((2pi * cutoff_freq) / (2 * sample_freq)) = tan((pi * cutoff_freq) / sample_freq)
    float K = tanf(PI * cutoff_freq / sample_freq);
    // c = (K^2 + K/Q + 1), Q = 1 / sqrt(2)
    float c                        = 1.0f + 1.41421356237f * K + K * K;
    lowpass2_data->b0              = K * K / c;
    lowpass2_data->b1              = 2.0f * lowpass2_data->b0;
    lowpass2_data->b2              = lowpass2_data->b0;
    lowpass2_data->a1              = 2.0f * (K * K - 1.0f) / c;
    lowpass2_data->a2              = (1.0f - 1.41421356237f * K + K * K) / c;
    lowpass2_data->delay_element_1 = 0.0f;
    lowpass2_data->delay_element_2 = 0.0f;
}

float Lowpass2Apply(Lowpass2Data* lowpass2_data, float sample) {
    // w[n] = x[n] - a1 * w[n-1] - a2 * w[n-2]
    float delay_element_0 = sample - lowpass2_data->delay_element_1 * lowpass2_data->a1 - lowpass2_data->delay_element_2 * lowpass2_data->a2;

    if (!isfinite(delay_element_0)) {
        // don't allow bad values to propigate via the filter
        delay_element_0 = sample;
    }

    // y[n] = b0 * w[n] + b1 * w[n-1] + b2 * w[n-2]
    float output = delay_element_0 * lowpass2_data->b0 + lowpass2_data->delay_element_1 * lowpass2_data->b1 + lowpass2_data->delay_element_2 * lowpass2_data->b2;

    lowpass2_data->delay_element_2 = lowpass2_data->delay_element_1;
    lowpass2_data->delay_element_1 = delay_element_0;

    return output;
}

float Lowpass2Reset(Lowpass2Data* lowpass2_data, float sample) {
    float dval                     = sample / (lowpass2_data->b0 + lowpass2_data->b1 + lowpass2_data->b2);
    lowpass2_data->delay_element_1 = dval;
    lowpass2_data->delay_element_2 = dval;
    return Lowpass2Apply(lowpass2_data, sample);
}

void Lowpass2Apply3Axis(Lowpass2Data* lowpass2_data, float *data) {
    data[0] = Lowpass2Apply(&lowpass2_data[0], data[0]);
    data[1] = Lowpass2Apply(&lowpass2_data[1], data[1]);
    data[2] = Lowpass2Apply(&lowpass2_data[2], data[2]);
}