#ifndef BUTTERWORTHFILTER_HPP
#define BUTTERWORTHFILTER_HPP

class ButterworthFilter {
public:
    ButterworthFilter(double cutoff_freq, double sampling_time); // 파라미터가 있는 생성자
    ButterworthFilter(); // 기본 생성자
    double apply(double input); // 입력 값을 필터링하는 메서드

private:
    double cutoff_freq_;       // 차단 주파수
    double sampling_time_;     // 샘플링 시간
    double prev_input_;        // 이전 입력 값 (상태 저장용)
    double prev_output_;       // 이전 출력 값 (상태 저장용)
};

#endif // BUTTERWORTHFILTER_HPP

