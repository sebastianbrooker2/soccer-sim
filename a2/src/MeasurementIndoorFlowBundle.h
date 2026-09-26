#ifndef MEASUREMENTINDOORFLOWBUNDLE_H
#define MEASUREMENTINDOORFLOWBUNDLE_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "Camera.h"
#include "Measurement.h"
#include "SystemEstimator.h"

/**
 * @brief Optical flow bundle measurement for indoor unstructured environment
 * 
 * Uses epipolar constraint violation (1D measurement per flow vector).
 * No assumptions about environment structure (no ground plane or sky dome).
 */
class MeasurementIndoorFlowBundle : public Measurement
{
public:
    /**
     * @brief Constructor
     * @param time Measurement time
     * @param camera Camera object
     * @param imgk_raw Current frame (raw)
     * @param imgkm1_raw Previous frame (raw)
     * @param rQOikm1 Previously tracked features (2xN matrix of pixel coordinates)
     */
    MeasurementIndoorFlowBundle(double time, const Camera & camera, const cv::Mat & imgk_raw, 
                               const cv::Mat & imgkm1_raw, const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOikm1);

    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

    /**
     * @brief Get tracked features from previous frame
     */
    const Eigen::Matrix<double, 2, Eigen::Dynamic> & trackedPreviousFeatures() const;

    /**
     * @brief Get tracked features from current frame
     */
    const Eigen::Matrix<double, 2, Eigen::Dynamic> & trackedCurrentFeatures() const;

    /**
     * @brief Get inlier mask (from RANSAC fundamental matrix estimation)
     */
    const std::vector<unsigned char> & inlierMask() const;

    /**
     * @brief Predict features in current frame given state
     * @param x State vector
     * @param system System estimator
     * @return 2xN matrix of predicted pixel coordinates
     */
    Eigen::Matrix<double, 2, Eigen::Dynamic> predictedFeatures(const Eigen::VectorXd & x, const SystemEstimator & system) const;

private:
    template <typename Scalar>
    Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const;

    template <typename Scalar>
    Eigen::Matrix<double, 3, Eigen::Dynamic> predictFlowImpl(const Eigen::VectorX<Scalar> & x,
                                                              const Eigen::Matrix<double, 3, Eigen::Dynamic> & pkm1,
                                                              const Eigen::Matrix<double, 3, Eigen::Dynamic> & pk) const;

    Camera camera_;                                           // Camera object
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1_;      // Previous frame features (pixel coords)
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOik_;        // Current frame features (pixel coords)
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOikm1_;   // Previous frame features (undistorted)
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOik_;     // Current frame features (undistorted)
    std::vector<unsigned char> mask_;                         // Inlier mask from RANSAC
    Eigen::Matrix<double, 3, Eigen::Dynamic> pkm1_;         // Inlier homogeneous coords (previous)
    Eigen::Matrix<double, 3, Eigen::Dynamic> pk_;           // Inlier homogeneous coords (current)
    double sigma_;                                            // Measurement noise std dev (pixels)
};

#endif // MEASUREMENTINDOORFLOWBUNDLE_H
