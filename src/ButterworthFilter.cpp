#include "test_pkg/ButterworthFilter.hpp"
#include <cmath>

ButterworthFilter::ButterworthFilter(double cutoff_freq, double sampling_time)
    : cutoff_freq_(cutoff_freq), sampling_time_(sampling_time),
      prev_input_(0.0), prev_output_(0.0) {}

ButterworthFilter::ButterworthFilter() 
    : ButterworthFilter(1.0, 0.01) {} // 기본 생성자에서 디폴트 값 설정

double ButterworthFilter::apply(double input) {
    // Butterworth 필터 간단 구현 (1차 필터)
    double alpha = sampling_time_ / (sampling_time_ + (1.0 / (2.0 * M_PI * cutoff_freq_)));
    double output = alpha * input + (1.0 - alpha) * prev_output_;
    prev_input_ = input;
    prev_output_ = output;
    return output;
}

