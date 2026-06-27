#include "custom_lio_ekf/covariance_propagation.hpp"

#include <cmath>

namespace custom_lio_ekf
{

namespace
{

bool isValidDt(double dt)
{
  return std::isfinite(dt) && dt > 0.0;
}

}  // namespace

Matrix18d buildContinuousErrorDynamics(
  const EkfState& state,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  bool use_accel_translation)
{
  Matrix18d F_c = Matrix18d::Zero();

  const Eigen::Matrix3d R_WI = state.q_WI.normalized().toRotationMatrix();

  // Attitude error dynamics are active in both rotation-only and full IMU modes.
  F_c.block<3, 3>(kThetaOffset, kThetaOffset) =
    -skewSymmetric(gyro_unbiased);

  F_c.block<3, 3>(kThetaOffset, kGyroBiasOffset) =
    -Eigen::Matrix3d::Identity();

  if (!use_accel_translation) {
    return F_c;
  }

  // These blocks are only valid when velocity/position are propagated from accel.
  F_c.block<3, 3>(kPositionOffset, kVelocityOffset) =
    Eigen::Matrix3d::Identity();

  F_c.block<3, 3>(kVelocityOffset, kThetaOffset) =
    -R_WI * skewSymmetric(accel_unbiased);

  F_c.block<3, 3>(kVelocityOffset, kAccelBiasOffset) =
    -R_WI;

  F_c.block<3, 3>(kVelocityOffset, kGravityOffset) =
    Eigen::Matrix3d::Identity();

  return F_c;
}

Matrix18x12d buildContinuousNoiseInputMatrix(
  const EkfState& state,
  bool use_accel_translation)
{
  Matrix18x12d G_c = Matrix18x12d::Zero();

  const Eigen::Matrix3d R_WI = state.q_WI.normalized().toRotationMatrix();

  G_c.block<3, 3>(kThetaOffset, 0) =
    -Eigen::Matrix3d::Identity();

  if (use_accel_translation) {
    G_c.block<3, 3>(kVelocityOffset, 3) =
      -R_WI;
  }

  G_c.block<3, 3>(kGyroBiasOffset, 6) =
    Eigen::Matrix3d::Identity();

  G_c.block<3, 3>(kAccelBiasOffset, 9) =
    Eigen::Matrix3d::Identity();

  return G_c;
}

Matrix18d discretizeErrorDynamics(
  const Matrix18d& F_c,
  double dt)
{
  // First-order discretization is adequate for the short Livox IMU intervals.
  return Matrix18d::Identity() + F_c * dt;
}

Matrix18d discretizeProcessNoise(
  const Matrix18x12d& G_c,
  const Matrix12d& Q_c,
  double dt)
{
  return G_c * Q_c * G_c.transpose() * dt;
}

void propagateCovariance(
  EkfState& state,
  const ImuNoiseStdDevs& imu_noise,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  double dt,
  bool use_accel_translation)
{
  const Matrix18d F_c =
    buildContinuousErrorDynamics(state, gyro_unbiased, accel_unbiased, use_accel_translation);
  const Matrix18x12d G_c =
    buildContinuousNoiseInputMatrix(state, use_accel_translation);
  const Matrix12d Q_c =
    makeContinuousImuNoiseCovariance(imu_noise);

  const Matrix18d F_d = discretizeErrorDynamics(F_c, dt);
  const Matrix18d Q_d = discretizeProcessNoise(G_c, Q_c, dt);

  state.P = F_d * state.P * F_d.transpose() + Q_d;
  symmetrizeCovariance(state);
}

void propagateStateAndCovariance(
  EkfState& state,
  const ImuNoiseStdDevs& imu_noise,
  const custom_imu_propagator::ImuSample& sample,
  double dt,
  bool use_accel_translation)
{
  const Eigen::Vector3d omega_hat = biasCorrectedGyro(sample, state);
  const Eigen::Vector3d accel_hat = biasCorrectedAccel(sample, state);

  // Linearize covariance propagation at the start of the IMU interval.
  // The nominal state is advanced afterward so F_c/G_c remain consistent with x_j.
  propagateCovariance(state, imu_noise, omega_hat, accel_hat, dt, use_accel_translation);
  propagateNominalState(state, omega_hat, accel_hat, dt, use_accel_translation);
}

bool predictStateAndCovariance(
  EkfState& state,
  const EkfParameters& parameters,
  const std::vector<custom_imu_propagator::ImuSample>& samples,
  EkfPredictionStats& stats)
{
  stats = EkfPredictionStats{};

  if (!hasValidNoiseParameters(parameters)) {
    stats.status = "invalid_ekf_parameters";
    return false;
  }

  if (samples.size() < 2) {
    stats.status = "not_enough_imu_samples";
    return false;
  }

  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    const auto& current = samples[i];
    const auto& next = samples[i + 1];

    const double dt = (next.stamp - current.stamp).seconds();

    if (!isValidDt(dt)) {
      stats.status = "invalid_imu_dt";
      return false;
    }

    propagateStateAndCovariance(
      state,
      parameters.imu_noise,
      current,
      dt,
      parameters.use_accel_translation_prediction);

    if (!isFinite(state)) {
      stats.status = "nonfinite_predicted_state";
      return false;
    }

    ++stats.intervals_integrated;
    stats.dt_total += dt;
  }

  stats.success = true;
  stats.status = "success";
  return true;
}

}  // namespace custom_lio_ekf
