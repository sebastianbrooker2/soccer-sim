#ifndef MEASUREMENTSLAMDUCKBUNDLE_H
#define MEASUREMENTSLAMDUCKBUNDLE_H

#include <Eigen/Core>
#include <vector>
#include <map>
#include <opencv2/core/mat.hpp>
#include "SystemBase.h"
#include "SystemEstimator.h"
#include "Camera.h"
#include "Pose.hpp"
#include "MeasurementSLAM.h"

class MeasurementDuckBundle : public MeasurementSLAM
{
public:
    MeasurementDuckBundle(double time, const std::vector<cv::Point2f> & centroids, 
                         const std::vector<int> & areas, const Camera & camera, double markerSize = 0.166);
    
    MeasurementSLAM * clone() const override;
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

    virtual GaussianInfo<double> predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const override;
    virtual GaussianInfo<double> predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const override;
    virtual const std::vector<int> & associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) override;

    // Getters for detected data
    const std::vector<cv::Point2f> & getCentroids() const { return centroids_; }
    const std::vector<int> & getAreas() const { return areas_; }
    const std::vector<int> & getAssociationStatus() const { return idxFeatures_; }
    
    // Set pre-associations for newly initialized landmarks
    void setPreAssociations(const std::map<std::size_t, std::size_t> & preAssoc) { preAssociations_ = preAssoc; }

protected:
    virtual void update(SystemBase & system) override;
    
    std::vector<cv::Point2f> centroids_;  // Detected duck centroids (pixel coordinates)
    std::vector<int> areas_;             // Detected duck areas (pixel count)
    double sigma_centroid_;             // Centroid measurement error std dev (pixels)
    double sigma_area_;                 // Area measurement error std dev
    std::vector<int> idxFeatures_;      // Features associated with visible landmarks
    std::map<std::size_t, std::size_t> preAssociations_;  // landmarkIdx -> detectionIdx for newly initialized landmarks
};

// Helper function to predict centroid and area for a landmark (templated for autodiff)
template <typename Scalar>
std::pair<Eigen::Vector2<Scalar>, Scalar> predictCentroidAndArea(
    const Eigen::VectorX<Scalar> & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera);

// Non-template wrapper for double with Jacobian computation
std::pair<Eigen::Vector2d, double> predictCentroidAndArea(
    const Eigen::VectorXd & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera, 
    Eigen::MatrixXd & J);

#endif
