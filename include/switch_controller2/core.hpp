#pragma once


#include <mutex>
#include <thread>
#include <queue>
#include <chrono>
#include <functional>
#include <iostream>
#include <iterator>
#include <condition_variable>
#include <Eigen/Dense>
#include <cmath>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

template <class T, size_t N> std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) 
    {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}