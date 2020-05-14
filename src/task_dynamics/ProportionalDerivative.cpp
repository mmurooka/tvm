/* Copyright 2017-2018 CNRS-AIST JRL and CNRS-UM LIRMM
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <Eigen/Eigenvalues>

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

  ProportionalDerivative::ProportionalDerivative(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::MatrixXd& kp, const Eigen::MatrixXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(double kp, const Eigen::VectorXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(double kp, const Eigen::MatrixXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::VectorXd& kp, double kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::VectorXd& kp, const Eigen::MatrixXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::MatrixXd& kp, double kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::MatrixXd& kp, const Eigen::VectorXd& kv)
    : kp_(kp)
    , kv_(kv)
  {
  }

  ProportionalDerivative::ProportionalDerivative(double kp)
    : ProportionalDerivative(kp, 2 * std::sqrt(kp))
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::VectorXd& kp)
    : kp_(kp)
    , kv_(std::in_place_index<1>, 2*kp.cwiseSqrt())
  {
  }

  ProportionalDerivative::ProportionalDerivative(const Eigen::MatrixXd& kp)
    : kp_(kp)
    , kv_(Eigen::MatrixXd(kp.rows(), kp.cols()))
  {
    Eigen::RealSchur<Eigen::MatrixXd> dec(kp);
    assert(dec.matrixT().isDiagonal(1e-8) && "kp is not symmetric.");
    assert((dec.matrixT().diagonal().array() >= 0).all() && "kp is undefinite.");
    std::get<2>(kv_).noalias() = 2*dec.matrixU() * dec.matrixT().diagonal().asDiagonal() * dec.matrixU().transpose();
  }

  std::unique_ptr<abstract::TaskDynamicsImpl> ProportionalDerivative::impl_(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs) const
  {
    return std::unique_ptr<abstract::TaskDynamicsImpl>(new Impl(f, t, rhs, kp_, kv_));
  }

  ProportionalDerivative::Impl::Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs, const Gain& kp, const Gain& kv)
    : TaskDynamicsImpl(Order::Two, f, t, rhs)
    , kp_(kp)
    , kv_(kv)
  {
    assert((kp.index() == 0                                           // Scalar gain
        || (kp.index() == 1 && std::get<1>(kp).size() == f->size())   // Diagonal gain
        || (kp.index() == 2 && std::get<2>(kp).cols() == f->size()    // Matrix gain
                            && std::get<2>(kp).rows() == f->size()))  
      && "Gain kp and function have incompatible sizes");

    assert((kv.index() == 0                                           // Scalar gain
        || (kv.index() == 1 && std::get<1>(kv).size() == f->size())   // Diagonal gain
        || (kv.index() == 2 && std::get<2>(kv).cols() == f->size()    // Matrix gain
                            && std::get<2>(kv).rows() == f->size()))  
      && "Gain kv and function have incompatible sizes");
  }

  void ProportionalDerivative::Impl::updateValue()
  {
    switch (kv_.index())
    {
    case 0: value_ = -std::get<double>(kv_) * function().velocity(); break;
    case 1: value_.noalias() = -(std::get<Eigen::VectorXd>(kv_).asDiagonal() * function().velocity()); break;
    case 2: value_.noalias() = -std::get<Eigen::MatrixXd>(kv_) * function().velocity(); break;
    default: assert(false);
    }
    switch (kp_.index())
    {
    case 0: value_ -= std::get<double>(kp_) * (function().value() - rhs()); break;
    case 1: value_.noalias() -= std::get<Eigen::VectorXd>(kp_).asDiagonal() * (function().value() - rhs()); break;
    case 2: value_.noalias() -= std::get<Eigen::MatrixXd>(kp_) * (function().value() - rhs()); break;
    default: assert(false);
    }
  }

  std::pair<const PD::Gain&, const PD::Gain&> ProportionalDerivative::Impl::gains() const
  {
    return {kp_, kv_};
  }

  void ProportionalDerivative::Impl::gains(double kp, double kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::VectorXd& kp, const Eigen::VectorXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::MatrixXd& kp, const Eigen::MatrixXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(double kp, const Eigen::VectorXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(double kp, const Eigen::MatrixXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::VectorXd& kp, double kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::VectorXd& kp, const Eigen::MatrixXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::MatrixXd& kp, double kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(const Eigen::MatrixXd& kp, const Eigen::VectorXd& kv)
  {
    kp_ = kp;
    kv_ = kv;
  }

  void ProportionalDerivative::Impl::gains(double kp)
  {
    kp_ = kp;
    kv_ = 2 * std::sqrt(kp);
  }

  void ProportionalDerivative::Impl::gains(const Eigen::VectorXd& kp)
  {
    kp_ = kp;
    kv_.emplace<1>(2 * kp.cwiseSqrt());
  }

  void ProportionalDerivative::Impl::gains(const Eigen::MatrixXd& kp)
  {
    Eigen::RealSchur<Eigen::MatrixXd> dec(kp);
    assert(dec.matrixT().isDiagonal(1e-8) && "kp is not symmetric.");
    assert((dec.matrixT().diagonal().array() >= 0).all() && "kp is undefinite.");
    kv_.emplace<2>(2*dec.matrixU() * dec.matrixT().diagonal().asDiagonal() * dec.matrixU().transpose());
  }

}  // namespace task_dynamics

}  // namespace tvm
