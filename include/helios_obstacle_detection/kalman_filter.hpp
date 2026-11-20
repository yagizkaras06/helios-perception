// Linear Kalman filter, 2D constant-velocity model, state x = [px, py, vx, vy].
// Process noise: white-noise acceleration (discrete Wiener process acceleration model).
#pragma once

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

namespace helios_obstacle_detection
{

class ConstantVelocityKF
{
public:
  using Vec2 = Eigen::Vector2d;
  using Vec4 = Eigen::Matrix<double, 4, 1>;
  using Mat2 = Eigen::Matrix2d;
  using Mat4 = Eigen::Matrix<double, 4, 4>;

  void init(const Vec2 & position, const Mat2 & position_cov, double velocity_sigma)
  {
    x_.setZero();
    x_.head<2>() = position;
    P_.setZero();
    P_.topLeftCorner<2, 2>() = position_cov;
    P_(2, 2) = P_(3, 3) = velocity_sigma * velocity_sigma;
  }

  void predict(double dt, double accel_sigma)
  {
    if (!(dt > 0.0)) {
      return;
    }
    Mat4 F = Mat4::Identity();
    F(0, 2) = F(1, 3) = dt;
    const double q = accel_sigma * accel_sigma;
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    Mat4 Q = Mat4::Zero();
    Q(0, 0) = Q(1, 1) = 0.25 * dt4 * q;
    Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = 0.5 * dt3 * q;
    Q(2, 2) = Q(3, 3) = dt2 * q;
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
    symmetrize();
  }

  /// Innovation y = z - Hx and its covariance S = H P H^T + R  (H = [I 0]).
  void innovation(const Vec2 & z, const Mat2 & R, Vec2 & y, Mat2 & S) const
  {
    y = z - x_.head<2>();
    S = P_.topLeftCorner<2, 2>() + R;
  }

  /// Squared Mahalanobis distance of a measurement (NIS).
  double mahalanobis2(const Vec2 & z, const Mat2 & R) const
  {
    Vec2 y;
    Mat2 S;
    innovation(z, R, y, S);
    return y.dot(S.ldlt().solve(y));
  }

  /// Joseph-form update (numerically stable, keeps P symmetric positive semi-definite).
  void update(const Vec2 & z, const Mat2 & R)
  {
    Vec2 y;
    Mat2 S;
    innovation(z, R, y, S);
    const Eigen::Matrix<double, 4, 2> PHt = P_.leftCols<2>();
    const Eigen::Matrix<double, 4, 2> K = PHt * S.inverse();
    x_ += K * y;
    Mat4 IKH = Mat4::Identity();
    IKH.leftCols<2>() -= K;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    symmetrize();
  }

  /// Replaces the velocity by v with an isotropic sigma and no position/velocity correlation
  /// (static motion model while an object is not observed).
  void resetVelocity(const Vec2 & v, double sigma)
  {
    x_.tail<2>() = v;
    P_.topRightCorner<2, 2>().setZero();
    P_.bottomLeftCorner<2, 2>().setZero();
    P_.bottomRightCorner<2, 2>() = sigma * sigma * Mat2::Identity();
  }

  const Vec4 & x() const noexcept {return x_;}
  const Mat4 & P() const noexcept {return P_;}
  Vec2 position() const {return x_.head<2>();}
  Vec2 velocity() const {return x_.tail<2>();}
  Mat2 positionCov() const {return P_.topLeftCorner<2, 2>();}
  Mat2 velocityCov() const {return P_.bottomRightCorner<2, 2>();}

private:
  void symmetrize() {P_ = 0.5 * (P_ + P_.transpose());}

  Vec4 x_{Vec4::Zero()};
  Mat4 P_{Mat4::Identity()};
};

}  // namespace helios_obstacle_detection
