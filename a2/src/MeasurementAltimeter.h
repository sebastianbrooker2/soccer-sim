/**
 * @file MeasurementAltimeter.h
 * @brief Altimeter measurement class for Scenario 4
 */

#ifndef MEASUREMENTALTIMETER_H
#define MEASUREMENTALTIMETER_H

#include <Eigen/Core>
#include "Measurement.h"

class SystemVisualNav;

/**
 * @class MeasurementAltimeter
 * @brief Measurement class for altitude sensor (from DJI subtitle data)
 * 
 * Provides a Gaussian likelihood on the z-position component of the state.
 * Used in Scenario 4 to constrain the altitude drift in visual odometry.
 */
class MeasurementAltimeter : public Measurement
{
public:
    /**
     * @brief Construct altimeter measurement
     * @param time Measurement timestamp
     * @param altitude Measured altitude (AGL in meters)
     * @param sigma Measurement standard deviation (meters)
     */
    MeasurementAltimeter(double time, double altitude, double sigma = 1.0);

    /**
     * @brief Simulate a measurement given a state
     * @param x The state vector
     * @param system The system estimator
     * @return The simulated measurement (altitude)
     */
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;

    /**
     * @brief Calculate log-likelihood
     * @param x The state vector
     * @param system The system estimator
     * @return The log-likelihood value
     */
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;

    /**
     * @brief Calculate log-likelihood and gradient
     * @param x The state vector
     * @param system The system estimator
     * @param g Output gradient
     * @return The log-likelihood value
     */
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;

    /**
     * @brief Calculate log-likelihood, gradient, and Hessian
     * @param x The state vector
     * @param system The system estimator
     * @param g Output gradient
     * @param H Output Hessian
     * @return The log-likelihood value
     */
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

private:
    double altitude_;  ///< Measured altitude above ground level (m)
    double sigma_;     ///< Measurement standard deviation (m)
};

#endif // MEASUREMENTALTIMETER_H
