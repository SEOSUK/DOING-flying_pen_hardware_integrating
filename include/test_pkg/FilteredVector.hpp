#ifndef FILTEREDVECTOR_HPP
#define FILTEREDVECTOR_HPP

#include <Eigen/Dense>
#include <vector>
#include "ButterworthFilter.hpp"

class FilteredVector {
private:
    std::vector<ButterworthFilter> filters; // 각 벡터 요소에 대한 필터
    size_t size;

public:
    FilteredVector(size_t vectorSize, double cutoff_freq, double sampling_time)
        : size(vectorSize) {
        for (size_t i = 0; i < size; ++i) {
            filters.emplace_back(cutoff_freq, sampling_time);
        }
    }

    Eigen::VectorXd apply(const Eigen::VectorXd& input) {
        Eigen::VectorXd filtered(input.size());
        for (size_t i = 0; i < size; ++i) {
            filtered[i] = filters[i].apply(input[i]); // 각 요소에 필터 적용
        }
        return filtered;
    }
};

#endif // FILTEREDVECTOR_HPP

