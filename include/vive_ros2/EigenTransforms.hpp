#ifndef EIGEN_TRANSFORMS_HPP
#define EIGEN_TRANSFORMS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <openvr.h>
#include <cmath>

namespace VRTransforms {

/**
 * @brief Converts OpenVR HmdMatrix34_t to Eigen::Matrix4d
 * @param vrMatrix OpenVR matrix
 * @return Eigen transformation matrix
 */
inline Eigen::Matrix4d vrMatrixToEigen(const vr::HmdMatrix34_t& vrMatrix) {
    Eigen::Matrix4d eigenMatrix = Eigen::Matrix4d::Identity();
    
    // Copy the 3x4 matrix from OpenVR format
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            eigenMatrix(row, col) = vrMatrix.m[row][col];
        }
    }
    
    return eigenMatrix;
}

/**
 * @brief Extracts position from transformation matrix
 * @param transform Eigen transformation matrix
 * @return Position vector
 */
inline Eigen::Vector3d getPosition(const Eigen::Matrix4d& transform) {
    return transform.block<3, 1>(0, 3);
}

/**
 * @brief Extracts rotation matrix from transformation matrix
 * @param transform Eigen transformation matrix
 * @return 3x3 rotation matrix
 */
inline Eigen::Matrix3d getRotationMatrix(const Eigen::Matrix4d& transform) {
    return transform.block<3, 3>(0, 0);
}

/**
 * @brief Extracts quaternion from transformation matrix
 * @param transform Eigen transformation matrix
 * @return Quaternion (w, x, y, z)
 */
inline Eigen::Quaterniond getQuaternion(const Eigen::Matrix4d& transform) {
    Eigen::Matrix3d rotationMatrix = getRotationMatrix(transform);
    return Eigen::Quaterniond(rotationMatrix);
}

/**
 * @brief Directly extracts quaternion from OpenVR matrix
 * @param vrMatrix OpenVR matrix
 * @return Quaternion (w, x, y, z)
 */
inline Eigen::Quaterniond getQuaternionFromVRMatrix(const vr::HmdMatrix34_t& vrMatrix) {
    Eigen::Matrix4d eigenMatrix = vrMatrixToEigen(vrMatrix);
    return getQuaternion(eigenMatrix);
}

/**
 * @brief Directly extracts position from OpenVR matrix
 * @param vrMatrix OpenVR matrix
 * @return Position vector
 */
inline Eigen::Vector3d getPositionFromVRMatrix(const vr::HmdMatrix34_t& vrMatrix) {
    return Eigen::Vector3d(vrMatrix.m[0][3], vrMatrix.m[1][3], vrMatrix.m[2][3]);
}

/**
 * @brief Converts quaternion to Euler angles (XYZ order)
 * @param q Quaternion
 * @return Euler angles (roll, pitch, yaw) in radians
 */
inline Eigen::Vector3d quaternionToEulerXYZ(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix().eulerAngles(0, 1, 2); // XYZ order
}

/**
 * @brief Converts Euler angles to quaternion
 * @param euler Euler angles (roll, pitch, yaw) in radians
 * @return Quaternion
 */
inline Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& euler) {
    return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}

/**
 * @brief Creates a transformation matrix from position and quaternion
 * @param position Position vector
 * @param quaternion Quaternion
 * @return 4x4 transformation matrix
 */
inline Eigen::Matrix4d createTransform(const Eigen::Vector3d& position, 
                                       const Eigen::Quaterniond& quaternion) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position;
    return transform;
}

/**
 * @brief Computes relative transformation between two poses
 * @param from Initial pose
 * @param to Target pose
 * @return Relative transformation from 'from' to 'to'
 */
inline Eigen::Matrix4d computeRelativeTransform(const Eigen::Matrix4d& from, 
                                                 const Eigen::Matrix4d& to) {
    return from.inverse() * to;
}

/**
 * @brief Applies low-pass filter to position
 * @param current Current position
 * @param previous Previous filtered position
 * @param alpha Filter coefficient (0-1, higher = less filtering)
 * @return Filtered position
 */
inline Eigen::Vector3d filterPosition(const Eigen::Vector3d& current,
                                      const Eigen::Vector3d& previous,
                                      double alpha) {
    return alpha * current + (1.0 - alpha) * previous;
}

/**
 * @brief Applies SLERP to quaternions for filtering
 * @param current Current quaternion
 * @param previous Previous filtered quaternion
 * @param alpha Filter coefficient (0-1, higher = less filtering)
 * @return Filtered quaternion
 */
inline Eigen::Quaterniond filterQuaternion(const Eigen::Quaterniond& current,
                                           const Eigen::Quaterniond& previous,
                                           double alpha) {
    return previous.slerp(alpha, current);
}

/**
 * @brief Checks if position change is reasonable (for outlier detection)
 * @param currentPos Current position
 * @param previousPos Previous position
 * @param maxDistance Maximum allowed distance change
 * @return True if change is reasonable
 */
inline bool isPositionChangeReasonable(const Eigen::Vector3d& currentPos,
                                       const Eigen::Vector3d& previousPos,
                                       double maxDistance) {
    return (currentPos - previousPos).norm() <= maxDistance;
}

/**
 * @brief Computes velocity from position change
 * @param currentPos Current position
 * @param previousPos Previous position
 * @param deltaTime Time difference in seconds
 * @return Velocity vector
 */
inline Eigen::Vector3d computeVelocity(const Eigen::Vector3d& currentPos,
                                       const Eigen::Vector3d& previousPos,
                                       double deltaTime) {
    if (deltaTime <= 0.0) return Eigen::Vector3d::Zero();
    return (currentPos - previousPos) / deltaTime;
}

/**
 * @brief Rotates a vector by a quaternion
 * @param vector Vector to rotate
 * @param quaternion Rotation quaternion
 * @return Rotated vector
 */
inline Eigen::Vector3d rotateVector(const Eigen::Vector3d& vector,
                                    const Eigen::Quaterniond& quaternion) {
    return quaternion * vector;
}

} // namespace VRTransforms

#endif // EIGEN_TRANSFORMS_HPP
