#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include "Measurement.h"
#include "Camera.h"
#include "SystemVisualNav.h"

/**
 * @brief Measurement class for point landmarks (Scenario 6)
 * 
 * Detects and tracks point features (corners) in indoor environments.
 * Features tracked using goodFeaturesToTrack + Lucas-Kanade optical flow.
 */
class MeasurementPointBundle : public Measurement
{
public:
    /**
     * @brief Construct measurement from detected point features
     * @param time Measurement timestamp
     * @param image Current camera frame
     * @param camera Camera calibration
     */
    MeasurementPointBundle(double time, const cv::Mat & image, const Camera & camera);
    
    /**
     * @brief Clone this measurement
     */
    Measurement * clone() const;
    
    /**
     * @brief Override update to handle map management (init + delete)
     */
    void update(SystemBase & system) override;
    
    /**
     * @brief Get detected feature locations
     */
    const std::vector<cv::Point2f> & detectedFeatures() const { return detectedFeatures_; }
    
    /**
     * @brief Get associated landmark indices (-1 = no association)
     */
    const std::vector<int> & associations() const { return associations_; }
    
    /**
     * @brief Predict feature density for a single landmark
     */
    DensityBase<double> * predictFeatureDensity(const SystemVisualNav & system, std::size_t idxLandmark) const;
    
    /**
     * @brief Predict feature bundle density for multiple landmarks
     */
    DensityBase<double> * predictFeatureBundleDensity(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks) const;
    
    /**
     * @brief Data association: match features to landmarks
     */
    std::vector<int> associate(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks);
    
    /**
     * @brief Simulate measurements (required by Measurement base class)
     */
    Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    
    /**
     * @brief Compute log-likelihood (required overloads)
     */
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;
    
protected:
    /**
     * @brief Compute log-likelihood
     */
    template <typename Scalar>
    Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const;
    
private:
    const Camera & camera_;
    std::vector<cv::Point2f> detectedFeatures_;  ///< Detected feature locations (pixel coords)
    std::vector<int> associations_;             ///< associations_[lmIdx] = featureIdx (or -1)
    double sigma_;                              ///< Measurement noise std dev (pixels)
};
