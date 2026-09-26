#ifndef MEASUREMENTOUTDOORFLOWBUNDLE_H
#define MEASUREMENTOUTDOORFLOWBUNDLE_H

#include <vector>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "SystemEstimator.h"
#include "Pose.hpp"
#include "Camera.h"
#include "Measurement.h"

class MeasurementOutdoorFlowBundle : public Measurement
{
public:
    MeasurementOutdoorFlowBundle(double time, const Camera & camera, const cv::Mat & imgk_raw, const cv::Mat & imgkm1_raw, const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOikm1);
    virtual Eigen::VectorXd simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const override;
    virtual double logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const override;

    // Helper functions for log likelihood and visualisation
    template <typename Scalar> Eigen::Matrix<Scalar, 3, Eigen::Dynamic> predictFlowImpl(const Eigen::VectorX<Scalar> & x, const Eigen::Matrix<double, 3, Eigen::Dynamic> & pkm1, const Eigen::Matrix<double, 3, Eigen::Dynamic> & pk) const;
    template <typename Scalar> Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const;
    Eigen::Matrix<double, 2, Eigen::Dynamic> predictedFeatures(const Eigen::VectorXd & x, const SystemEstimator & system) const;

    // Note: costOdometry is used only in Lab 11.
    //       Assignment 2 uses costJointDensity instead.
    template <typename Scalar> Scalar costOdometryImpl(const Eigen::VectorX<Scalar> & etak, const Eigen::VectorXd & etakm1) const;
    double costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1) const;
    double costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1, Eigen::VectorXd & g) const;
    double costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1, Eigen::VectorXd & g, Eigen::MatrixXd & H) const;

    const Eigen::Matrix<double, 2, Eigen::Dynamic> & trackedPreviousFeatures() const;
    const Eigen::Matrix<double, 2, Eigen::Dynamic> & trackedCurrentFeatures() const;
    const std::vector<unsigned char> & inlierMask() const;
protected:
    const Camera & camera_;

    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1_;      // Measured features for previous frame
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOik_;        // Measured features for current frame

    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOikm1_;   // Undistorted features for previous frame
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOik_;     // Undistorted features for current frame

    std::vector<unsigned char> mask_;                       // Inlier mask

    Eigen::Matrix<double, 3, Eigen::Dynamic> pkm1_;         // Inlier undistorted homogeneous points in previous frame
    Eigen::Matrix<double, 3, Eigen::Dynamic> pk_;           // Inlier undistorted homogeneous points in current frame

    double sigma_;                                          // Feature error standard deviation (in pixels)
};

template <typename Scalar>
Eigen::Matrix<Scalar, 3, Eigen::Dynamic> MeasurementOutdoorFlowBundle::predictFlowImpl(const Eigen::VectorX<Scalar> & x, const Eigen::Matrix<double, 3, Eigen::Dynamic> & pkm1, const Eigen::Matrix<double, 3, Eigen::Dynamic> & pk) const
{
    assert(x.rows() >= 18);
    assert(x.cols() == 1);
    assert(pkm1.cols() == pk.cols());
    
    // Extract body poses at k and k-1
    Eigen::Vector3<Scalar> rBNn_k = x.template segment<3>(6);
    Eigen::Vector3<Scalar> rpy_k = x.template segment<3>(9);
    Eigen::Vector3<Scalar> rBNn_km1 = x.template segment<3>(12);
    Eigen::Vector3<Scalar> rpy_km1 = x.template segment<3>(15);
    
    // Body to navigation frame rotation matrices
    Eigen::Matrix3<Scalar> Rnb_k = rpy2rot(rpy_k);
    Eigen::Matrix3<Scalar> Rnb_km1 = rpy2rot(rpy_km1);
    
    // Camera to body rotation (from camera.Tbc)
    Eigen::Matrix3d Rbc = camera_.Tbc.rotationMatrix;
    Eigen::Matrix3<Scalar> Rbc_s = Rbc.cast<Scalar>();
    
    // Camera to navigation rotation: Rnc = Rnb * Rbc
    Eigen::Matrix3<Scalar> Rnc_k = Rnb_k * Rbc_s;
    Eigen::Matrix3<Scalar> Rnc_km1 = Rnb_km1 * Rbc_s;
    
    // Camera positions (assuming camera and body origins coincide)
    Eigen::Vector3<Scalar> rCNn_k = rBNn_k;
    Eigen::Vector3<Scalar> rCNn_km1 = rBNn_km1;
    
    // Camera intrinsics
    double fx = camera_.cameraMatrix.at<double>(0, 0);
    double fy = camera_.cameraMatrix.at<double>(1, 1);
    double cx = camera_.cameraMatrix.at<double>(0, 2);
    double cy = camera_.cameraMatrix.at<double>(1, 2);
    
    Eigen::Matrix3<Scalar> K;
    K << fx, 0, cx,
         0, fy, cy,
         0, 0, 1;
    
    Eigen::Matrix3<Scalar> Kinv = K.inverse();
    
    // Compute homographies (Equations 3 and 4 from PDF)
    Eigen::Matrix3<Scalar> Rnc_k_T = Rnc_k.transpose();
    
    // H_infinity (Equation 4): for sky dome
    Eigen::Matrix3<Scalar> H_inf = K * Rnc_k_T * Rnc_km1 * Kinv;
    
    // H_z (Equation 3): for ground plane
    Eigen::Vector3<Scalar> e3(0, 0, 1);
    Eigen::Vector3<Scalar> delta_r = rCNn_km1 - rCNn_k;
    Scalar denominator = e3.transpose() * rCNn_km1;
    
    // Add safety check for denominator to prevent division by zero
    using std::abs;  // Enable ADL for autodiff
    if (abs(denominator) < Scalar(0.1)) {
        // If too close to ground, use only infinity homography
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> pk_hat(3, pkm1.cols());
        for (int j = 0; j < pkm1.cols(); ++j) {
            pk_hat.col(j) = H_inf * pkm1.col(j).template cast<Scalar>();
        }
        return pk_hat;
    }
    
    Eigen::Matrix3<Scalar> H_z = K * Rnc_k_T * (Eigen::Matrix3<Scalar>::Identity() - (delta_r * e3.transpose()) / denominator) * Rnc_km1 * Kinv;
    
    // Apply homographies to each point
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> pk_hat(3, pkm1.cols());
    
    int ground_count = 0;
    int sky_count = 0;
    
    for (int j = 0; j < pkm1.cols(); ++j)
    {
        // Check if point is above or below horizon (Equation 2)
        // e3^T * Rnc[k] * K^-1 * p[k] > 0 means ground plane
        Eigen::Vector3<Scalar> ray_k = Rnc_k * Kinv * pk.col(j).template cast<Scalar>();
        
        if (ray_k(2) > 0)
        {
            // e3^T * Rnc[k] * K^-1 * p[k] > 0 - use ground plane (Equation 2)
            pk_hat.col(j) = H_z * pkm1.col(j).template cast<Scalar>();
            ground_count++;
        }
        else
        {
            // Otherwise - use sky dome at infinity
            pk_hat.col(j) = H_inf * pkm1.col(j).template cast<Scalar>();
            sky_count++;
        }
    }
    
    // COMPREHENSIVE DEBUG: Print detailed predictFlowImpl info (only for double precision)
    if constexpr (std::is_same_v<Scalar, double>) {
        static int call_count = 0;
        bool print_debug = (call_count % 10 == 0);
        
        if (print_debug) {
            std::println("  [predictFlowImpl DEBUG]");
            std::println("    Poses extracted from state:");
            std::println("      Current (k): pos=({:.2f},{:.2f},{:.2f}), rpy=({:.3f},{:.3f},{:.3f})", 
                       rBNn_k(0), rBNn_k(1), rBNn_k(2), rpy_k(0), rpy_k(1), rpy_k(2));
            std::println("      Delayed (k-1): pos=({:.2f},{:.2f},{:.2f}), rpy=({:.3f},{:.3f},{:.3f})",
                       rBNn_km1(0), rBNn_km1(1), rBNn_km1(2), rpy_km1(0), rpy_km1(1), rpy_km1(2));
            std::println("    delta_r = rCNn_km1 - rCNn_k = ({:.2f},{:.2f},{:.2f})",
                       delta_r(0), delta_r(1), delta_r(2));
            std::println("    denominator = e3^T * rCNn_km1 = {:.2f}", denominator);
            std::println("    Homography split: ground={}, sky={}", ground_count, sky_count);
            std::println("    Current altitude = {:.2f}m", -rCNn_k(2));
        }
        call_count++;
    }
    
    return pk_hat;
}

template <typename Scalar>
Scalar MeasurementOutdoorFlowBundle::logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const
{
    assert(pkm1_.cols() == pk_.cols());

    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> pk_hat = predictFlowImpl(x, pkm1_, pk_);
    Eigen::Matrix<Scalar, 2, Eigen::Dynamic> rQbarOik_hat = pk_hat.template topRows<2>().array().rowwise()/pk_hat.row(2).array();

    Scalar logLik = 0;
    // TODO: Lab 11
    int n = pk_.cols();
    Scalar sigma_sq = Scalar(sigma_ * sigma_);
    
    // Compute residuals for all inlier points
    Eigen::Matrix<Scalar, 2, Eigen::Dynamic> residuals = pk_.template topRows<2>().template cast<Scalar>() - rQbarOik_hat;
    
    // Sum of squared residuals
    Scalar sum_sq_residuals = residuals.array().square().sum();
    
    // Log likelihood: -1/(2*sigma^2) * sum_sq_residuals - n*log(2*pi*sigma^2)
    using std::log;  // Enable ADL for autodiff
    logLik = -sum_sq_residuals / (2.0 * sigma_sq) - Scalar(n) * log(Scalar(2.0 * M_PI) * sigma_sq);
    
    return logLik;
}

// Note: costOdometry is used only in Lab 11, not Assignment 2.
template <typename Scalar>
Scalar MeasurementOutdoorFlowBundle::costOdometryImpl(const Eigen::VectorX<Scalar> & etak, const Eigen::VectorXd & etakm1) const
{
    Eigen::VectorX<Scalar> x(18);
    x.template segment<6>(6) = etak;
    x.template segment<6>(12) = etakm1;
    return -logLikelihoodImpl(x);
}

#endif
