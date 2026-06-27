#include "custom_lio_ekf/imu_prediction.hpp"

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

Eigen::Vector3d biasCorrectedGyro(
  const custom_imu_propagator::ImuSample& sample,
  const EkfState& state)
{
  return sample.gyro - state.b_g;
}

Eigen::Vector3d biasCorrectedAccel(
  const custom_imu_propagator::ImuSample& sample,
  const EkfState& state)
{
  return sample.accel - state.b_a;
}

Eigen::Vector3d worldAcceleration(
  const EkfState& state,
  const Eigen::Vector3d& accel_unbiased)
{
  return state.q_WI.normalized() * accel_unbiased + state.g_W;
}

void propagateNominalState(
  EkfState& state,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  double dt,
  bool use_accel_translation)
{
  if (use_accel_translation) {
    const Eigen::Vector3d a_W = worldAcceleration(state, accel_unbiased);

    state.p_I_W = state.p_I_W +
      state.v_I_W * dt +
      0.5 * a_W * dt * dt;

    state.v_I_W = state.v_I_W + a_W * dt;
  }

  state.q_WI = state.q_WI * so3Exp(gyro_unbiased * dt);
  state.q_WI.normalize();
}

bool predictNominalState(
  EkfState& state,
  const std::vector<custom_imu_propagator::ImuSample>& samples,
  ImuPredictionStats& stats,
  bool use_accel_translation)
{
  stats = ImuPredictionStats{};

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

    const Eigen::Vector3d omega_hat = biasCorrectedGyro(current, state);
    const Eigen::Vector3d accel_hat = biasCorrectedAccel(current, state);

    if (!omega_hat.allFinite() || !accel_hat.allFinite()) {
      stats.status = "nonfinite_imu_measurement";
      return false;
    }

    propagateNominalState(state, omega_hat, accel_hat, dt, use_accel_translation);

    ++stats.intervals_integrated;
    stats.dt_total += dt;
  }

  if (!isFinite(state)) {
    stats.status = "nonfinite_predicted_state";
    return false;
  }

  stats.success = true;
  stats.status = "success";
  return true;
}

}  // namespace custom_lio_ekf
