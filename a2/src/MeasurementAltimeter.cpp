#include <cmath>
#include <numbers>
#include <Eigen/Core>
#include "SystemVisualNav.h"
#include "MeasurementAltimeter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

MeasurementAltimeter::MeasurementAltimeter(double time, double altitude, double sigma)
    : Measurement(time)
    , altitude_(altitude)
    , sigma_(sigma)
{
}

Eigen::VectorXd MeasurementAltimeter::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    // Measurement model: y = -x[8] (altitude = -D, where D is down in NED)
    Eigen::VectorXd y(1);
    y(0) = -x(8);
    return y;
}

double MeasurementAltimeter::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    // Predicted measurement
    Eigen::VectorXd y_pred = simulate(x, system);
    
    // Residual
    double residual = altitude_ - y_pred(0);
    
    // Log-likelihood: -0.5 * (residual^2 / sigma^2) - 0.5 * log(2*pi*sigma^2)
    double sigma_sq = sigma_ * sigma_;
    double logLik = -0.5 * (residual * residual / sigma_sq) - 0.5 * std::log(2.0 * M_PI * sigma_sq);
    
    return logLik;
}

double MeasurementAltimeter::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    // Predicted measurement
    Eigen::VectorXd y_pred = simulate(x, system);
    
    // Residual
    double residual = altitude_ - y_pred(0);
    
    // Gradient: dL/dx = (residual / sigma^2) * dh/dx
    // where dh/dx has only one non-zero entry: dh/dx[8] = -1
    g.resize(x.size());
    g.setZero();
    g(8) = residual / (sigma_ * sigma_);  // dL/dx[8] = residual/sigma^2 * (-1) * (-1)
    
    // Log-likelihood
    double sigma_sq = sigma_ * sigma_;
    double logLik = -0.5 * (residual * residual / sigma_sq) - 0.5 * std::log(2.0 * M_PI * sigma_sq);
    
    return logLik;
}

double MeasurementAltimeter::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // Predicted measurement
    Eigen::VectorXd y_pred = simulate(x, system);
    
    // Residual
    double residual = altitude_ - y_pred(0);
    
    // Gradient
    g.resize(x.size());
    g.setZero();
    g(8) = residual / (sigma_ * sigma_);
    
    // Hessian: d²L/dx² = -1/sigma^2 * (dh/dx)^T * (dh/dx)
    // Only H(8,8) is non-zero
    H.resize(x.size(), x.size());
    H.setZero();
    H(8, 8) = -1.0 / (sigma_ * sigma_);
    
    // Log-likelihood
    double sigma_sq = sigma_ * sigma_;
    double logLik = -0.5 * (residual * residual / sigma_sq) - 0.5 * std::log(2.0 * M_PI * sigma_sq);
    
    return logLik;
}
