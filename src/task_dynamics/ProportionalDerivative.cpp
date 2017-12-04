#include <tvm/task_dynamics/ProportionalDerivative.h>

#include <tvm/function/abstract/Function.h>

namespace tvm
{

namespace task_dynamics
{

  ProportionalDerivative::ProportionalDerivative(double kp, double kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(double kp)
    : ProportionalDerivative(kp, 2 * std::sqrt(kp))
  {
  }

  std::unique_ptr<abstract::TaskDynamicsImpl> ProportionalDerivative::impl_(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs) const
  {
    return std::unique_ptr<abstract::TaskDynamicsImpl>(new Impl(f, t, rhs, kp_, kv_));
  }

  ProportionalDerivative::Impl::Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs, double kp, double kv)
    : TaskDynamicsImpl(Order::Two, f, t, rhs)
    , kp_(kp)
    , kv_(kv)
  {
  }

  void ProportionalDerivative::Impl::updateValue()
  {
    value_ = -kv_ * function().velocity() - kp_ * (function().value() - rhs());
  }

  std::pair<double, double> ProportionalDerivative::Impl::gains() const
  {
    return {kp_, kv_};
  }

  void ProportionalDerivative::Impl::gains(double kp, double kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(double kp)
  {
    kp_ = kp;
    kv_ = 2 * std::sqrt(kp);
  }

}  // namespace task_dynamics

}  // namespace tvm