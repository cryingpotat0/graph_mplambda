#include <Eigen/Dense>
#include <jilog.hpp>

int main() {
    Eigen::Matrix<double, 2, 1> a, b;
    a << 10, 10;
    b << 11, 11;
    JI_LOG(INFO) << "a: " << a << ", b: " << b << ", isApprox: " << (a-b).isMuchSmallerThan(1.415, 1) << " " << (a-b).norm();
    return 0;
}

