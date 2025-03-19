#ifndef SU_ROT_HPP
#define SU_ROT_HPP

#include <Eigen/Dense>
#include <cmath>

// Function to compute the rotation matrix from roll, pitch, and yaw
inline Eigen::Matrix3d get_rotation_matrix(double roll, double pitch, double yaw) {
    // Z-axis (Yaw) rotation
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw),  std::cos(yaw), 0,
          0, 0, 1;

    // Y-axis (Pitch) rotation
    Eigen::Matrix3d Ry;
    Ry << std::cos(pitch), 0, std::sin(pitch),
          0, 1, 0,
         -std::sin(pitch), 0, std::cos(pitch);

    // X-axis (Roll) rotation
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll),  std::cos(roll);

    // Combined rotation: R = Rx * Ry * Rz
    return Rz * Ry * Rx;
}


#endif // SU_ROT_HPP
