#ifndef MEASUREMENTSLAMUNIQUETAGBUNDLE_H
#define MEASUREMENTSLAMUNIQUETAGBUNDLE_H

#include <Eigen/Core>
#include <vector>
#include "SystemBase.h"
#include "SystemEstimator.h"
#include "Camera.h"
#include "Pose.hpp"
#include "MeasurementSLAM.h"

class MeasurementUniqueTagBundle : public MeasurementSLAM
{
public:
    MeasurementUniqueTagBundle(double time, const std::vector<int> & markerIds, 
                          const std::vector<std::vector<cv::Point2f>> & markerCorners, 
                          const Camera & camera, double markerSize = 0.166);
    
    MeasurementSLAM * clone() const override;
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

    template <typename Scalar> Eigen::Vector2<Scalar> predictCorner(const Eigen::VectorX<Scalar> & x, const SystemSLAM & system, std::size_t idxLandmark, int cornerIdx) const;
    Eigen::Vector2d predictCorner(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const SystemSLAM & system, std::size_t idxLandmark, int cornerIdx) const;
    virtual GaussianInfo<double> predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const override;

    template <typename Scalar> Eigen::VectorX<Scalar> predictCornerBundle(const Eigen::VectorX<Scalar> & x, const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const;
    Eigen::VectorXd predictCornerBundle(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const;
    virtual GaussianInfo<double> predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const override;

    virtual const std::vector<int> & associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) override;

    // Public getters for visualization
    const std::vector<std::vector<cv::Point2f>> & getMarkerCorners() const { return markerCorners_; }
    const std::vector<int> & getMarkerIds() const { return markerIds_; }
    const std::vector<int> & getAssociationStatus() const { return idxFeatures_; }

protected:
    virtual void update(SystemBase & system) override;
    
    std::vector<int> markerIds_;                            // Detected marker IDs
    std::vector<std::vector<cv::Point2f>> markerCorners_;   // 4 corners per marker
    double markerSize_;                                      // Edge length in meters (default 0.166)
    double sigma_;                                          // Corner measurement error std dev (pixels)
    std::vector<int> idxFeatures_;                          // Features associated with visible landmarks
};

// Predict image location of a single corner
template <typename Scalar>
Eigen::Vector2<Scalar> MeasurementUniqueTagBundle::predictCorner(const Eigen::VectorX<Scalar> & x, const SystemSLAM & system, std::size_t idxLandmark, int cornerIdx) const
{
    // Obtain camera pose from state using STATIC template functions
    Pose<Scalar> Tnc;
    Tnc.translationVector = SystemSLAM::cameraPosition<Scalar>(camera_, x);      // rCNn
    Tnc.rotationMatrix = SystemSLAM::cameraOrientation<Scalar>(camera_, x);      // Rnc

    // Obtain landmark pose from state (position + orientation)
    std::size_t idx = system.landmarkPositionIndex(idxLandmark);
    Eigen::Vector3<Scalar> rPNn = x.template segment<3>(idx);       // Landmark position
    Eigen::Vector3<Scalar> Theta = x.template segment<3>(idx + 3);  // Landmark orientation (RPY)

    // Compute landmark rotation matrix from RPY angles
    Eigen::Matrix3<Scalar> Rnj = rpy2rot(Theta);

    // Corner position in marker frame (4 corners arranged in square)
    Scalar half = static_cast<Scalar>(markerSize_) / Scalar(2);
    Eigen::Vector3<Scalar> rjc;
    if (cornerIdx == 0)      rjc << -half,  half, Scalar(0);  // Top-left
    else if (cornerIdx == 1) rjc <<  half,  half, Scalar(0);  // Top-right
    else if (cornerIdx == 2) rjc <<  half, -half, Scalar(0);  // Bottom-right
    else                     rjc << -half, -half, Scalar(0);  // Bottom-left

    // Transform corner to world frame: r^n_{jc/N} = R^n_j * r^j_{jc/j} + r^n_{j/N}
    Eigen::Vector3<Scalar> rCornerNn = Rnj * rjc + rPNn;

    // Transform to camera frame
    Eigen::Vector3<Scalar> rCornerCc = Tnc.rotationMatrix.transpose() * (rCornerNn - Tnc.translationVector);

    // Project to pixel coordinates
    Eigen::Vector2<Scalar> rQOi = camera_.vectorToPixel(rCornerCc);
    
    return rQOi;
}

// Predict all corners for a bundle of landmarks
template <typename Scalar>
Eigen::VectorX<Scalar> MeasurementUniqueTagBundle::predictCornerBundle(const Eigen::VectorX<Scalar> & x, const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t nL = idxLandmarks.size();
    Eigen::VectorX<Scalar> h(8 * nL);  // 4 corners × 2 coordinates per landmark
    
    for (std::size_t i = 0; i < nL; ++i)
    {
        for (int c = 0; c < 4; ++c)
        {
            Eigen::Vector2<Scalar> corner = predictCorner(x, system, idxLandmarks[i], c);
            h.template segment<2>(8 * i + 2 * c) = corner;
        }
    }
    return h;
}

#endif
