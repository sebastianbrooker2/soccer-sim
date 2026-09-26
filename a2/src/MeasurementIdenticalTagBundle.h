#ifndef MEASUREMENTIDENTICALTAGBUNDLE_H
#define MEASUREMENTIDENTICALTAGBUNDLE_H

#include <vector>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include "Camera.h"
#include "SystemVisualNav.h"
#include "MeasurementSLAM.h"
#include "GaussianInfo.hpp"

class MeasurementIdenticalTagBundle : public MeasurementSLAM
{
public:
    MeasurementIdenticalTagBundle(double time, const cv::Mat & image, const Camera & camera, double tagSize);
    
    MeasurementSLAM * clone() const override;
    
    // Accessor for detected corners (needed for initialization in main loop)
    const std::vector<std::vector<cv::Point2f>> & detectedCorners() const { return corners_; }
    
    Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;
    
    // Base class pure virtuals (SystemSLAM interface)
    GaussianInfo<double> predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const override;
    GaussianInfo<double> predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const override;
    const std::vector<int> & associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) override;
    
    // Overloaded versions for SystemVisualNav
    GaussianInfo<double> predictFeatureDensity(const SystemVisualNav & system, std::size_t idxLandmark) const;
    GaussianInfo<double> predictFeatureBundleDensity(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks) const;
    const std::vector<int> & associate(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks);
    
    void update(SystemBase & system) override;
    
private:
    std::vector<std::vector<cv::Point2f>> corners_;  // Detected tag corners (4 per tag)
    std::vector<int> idxFeatures_;                    // Association: idxFeatures_[landmark] = detection
    double tagSize_;                                  // Physical tag size in meters
    double sigma_;                                    // Corner measurement uncertainty (pixels)
};

#endif
