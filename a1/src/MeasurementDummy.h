/**
 * @file MeasurementDummy.h
 * @brief Defines the MeasurementDummy class for a blind/no-op measurement.
 */
#ifndef MEASUREMENTDUMMY_H
#define MEASUREMENTDUMMY_H

#include <Eigen/Core>
#include <unsupported/Eigen/CXX11/Tensor>
#include "GaussianInfo.hpp"
#include "SystemEstimator.h"
#include "Measurement.h"

/**
 * @class MeasurementDummy
 * @brief A dummy measurement that provides no information.
 *
 * This class is useful for testing propagation behaviour of the filter
 * without actually updating with informative measurements.
 */
class MeasurementDummy : public Measurement
{
public:
    /**
     * @brief Construct a new MeasurementDummy object.
     * @param time The time of the measurement.
     * @param y    The dummy measurement vector (values ignored).
     */
    MeasurementDummy(double time, const Eigen::VectorXd & y);

    /**
     * @brief Construct a new MeasurementDummy object with verbosity.
     * @param time The time of the measurement.
     * @param y    The dummy measurement vector (values ignored).
     * @param verbosity The verbosity level for logging and output.
     */
    MeasurementDummy(double time, const Eigen::VectorXd & y, int verbosity);

    virtual ~MeasurementDummy() override;

    // Inherited virtual functions
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

protected:
    virtual std::string getProcessString() const override;
    Eigen::VectorXd y_;  // Dummy measurement vector
};

#endif
