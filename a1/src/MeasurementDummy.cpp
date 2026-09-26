#include <cmath>
#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "GaussianInfo.hpp"
#include "Measurement.h"
#include "MeasurementDummy.h"

MeasurementDummy::MeasurementDummy(double time, const Eigen::VectorXd & y)
    : Measurement(time)
    , y_(y)
{
    // updateMethod_ = UpdateMethod::BFGSLMSQRT;
    updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
    // updateMethod_ = UpdateMethod::SR1TRUSTEIG;
    // updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;

    // updateMethod_ = UpdateMethod::AFFINE;
    // updateMethod_ = UpdateMethod::GAUSSNEWTON;
    // updateMethod_ = UpdateMethod::LEVELBERGMARQUARDT;
}


MeasurementDummy::MeasurementDummy(double time, const Eigen::VectorXd & y, int verbosity)
    : Measurement(time, verbosity)
    , y_(y)
{
    // updateMethod_ = UpdateMethod::BFGSLMSQRT;
    updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
    // updateMethod_ = UpdateMethod::SR1TRUSTEIG;
    // updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;

    // updateMethod_ = UpdateMethod::AFFINE;
    // updateMethod_ = UpdateMethod::GAUSSNEWTON;
    // updateMethod_ = UpdateMethod::LEVELBERGMARQUARDT;
}

MeasurementDummy::~MeasurementDummy() = default;

std::string MeasurementDummy::getProcessString() const
{
    return "Dummy measurement update:";
}

// Simulate measurement
Eigen::VectorXd MeasurementDummy::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    return y_;
}

// Log-likelihood (dummy - returns 0)
double MeasurementDummy::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    return 0.0;
}

// Log-likelihood with gradient (dummy - returns 0 and zero gradient)
double MeasurementDummy::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    g.resize(x.size());
    g.setZero();
    return 0.0;
}

// Log-likelihood with gradient and Hessian (dummy - returns 0 with zero gradient and Hessian)
double MeasurementDummy::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    g.resize(x.size());
    g.setZero();
    H.resize(x.size(), x.size());
    H.setZero();
    return 0.0;
}
