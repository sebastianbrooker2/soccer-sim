#include <cstddef>
#include <print>
#include <numeric>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "GaussianInfo.hpp"
#include "rotation.hpp"
#include "SystemEstimator.h"
#include "MeasurementIndoorFlowBundle.h"

MeasurementIndoorFlowBundle::MeasurementIndoorFlowBundle(double time, const Camera & camera, const cv::Mat & imgk_raw, const cv::Mat & imgkm1_raw, const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOikm1)
    : Measurement(time)
    , camera_(camera)
    , rQOikm1_(rQOikm1)
    , rQOik_()
    , rQbarOikm1_()
    , rQbarOik_()
    , mask_()
    , pkm1_()
    , pk_()
    , sigma_(8.0)  // 8 pixel std dev - same as outdoor, weak enough to prevent orientation lock
{
    // Feature detection and tracking parameters (same as outdoor)
    const int divisor               = 2;
    const int maxNumFeatures        = 100;
    const int minNumFeatures        = 90;
    const double featureQuality     = 0.0007;             
    const double minFeatureDistance = 15.0;

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01);
    cv::Size subPixWinSize(11, 11);
    cv::Size winSize(21, 21);

    // Convert to grayscale and scale
    cv::Mat imgk_gray, imgkm1_gray;
    cv::cvtColor(imgk_raw, imgk_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgkm1_raw, imgkm1_gray, cv::COLOR_BGR2GRAY);

    cv::Mat imgk_scaled, imgkm1_scaled;
    cv::resize(imgk_gray, imgk_scaled, cv::Size(), 1.0/divisor, 1.0/divisor);
    cv::resize(imgkm1_gray, imgkm1_scaled, cv::Size(), 1.0/divisor, 1.0/divisor);

    // Initialize or reuse features from previous frame
    std::vector<cv::Point2f> rQOikm1_scaled;
    if (rQOikm1_.cols() < minNumFeatures)
    {
        cv::goodFeaturesToTrack(imgkm1_scaled,rQOikm1_scaled, maxNumFeatures, featureQuality, minFeatureDistance, cv::Mat(), 3, false, 0.04);
        if (!rQOikm1_scaled.empty())
        {
            cv::cornerSubPix(imgkm1_scaled, rQOikm1_scaled, subPixWinSize, cv::Size(-1, -1), termcrit);
        }
    }
    else
    {
        rQOikm1_scaled.resize(rQOikm1_.cols());
        for (int j = 0; j < rQOikm1_.cols(); ++j)
        {
            rQOikm1_scaled[j].x = rQOikm1_(0, j)/divisor;
            rQOikm1_scaled[j].y = rQOikm1_(1, j)/divisor;
        }
    }

    // Track features using Lucas-Kanade optical flow
    std::vector<cv::Point2f> rQOik_scaled;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imgkm1_scaled, imgk_scaled, rQOikm1_scaled, rQOik_scaled, status, err, winSize, 3, termcrit);

    // Filter by tracking status
    int np = 0;
    std::vector<cv::Point2f> rQOik_scaled_filtered, rQOikm1_scaled_filtered;
    for (size_t j = 0; j < status.size(); ++j)
    {
        if (status[j])
        {
            rQOik_scaled_filtered.push_back(rQOik_scaled[j]);
            rQOikm1_scaled_filtered.push_back(rQOikm1_scaled[j]);
            np++;
        }
    }
    rQOik_scaled = rQOik_scaled_filtered;
    rQOikm1_scaled = rQOikm1_scaled_filtered;

    // Scale back to original resolution
    rQOik_.resize(2, np);
    rQOikm1_.resize(2, np);
    for (int j = 0; j < np; ++j)
    {
        rQOik_(0, j) = rQOik_scaled[j].x * divisor;
        rQOik_(1, j) = rQOik_scaled[j].y * divisor;
        rQOikm1_(0, j) = rQOikm1_scaled[j].x * divisor;
        rQOikm1_(1, j) = rQOikm1_scaled[j].y * divisor;
    }

    // Undistort features
    rQbarOik_.resize(2, np);
    rQbarOikm1_.resize(2, np);
    rQbarOikm1_ = camera_.undistort(rQOikm1_);
    rQbarOik_ = camera_.undistort(rQOik_);

    // RANSAC fundamental matrix to find inliers
    std::vector<cv::Point2f> rQbarOikm1_cv(np), rQbarOik_cv(np);
    for (int j = 0; j < np; ++j)
    {
        rQbarOikm1_cv[j].x = rQbarOikm1_(0, j);
        rQbarOikm1_cv[j].y = rQbarOikm1_(1, j);
        rQbarOik_cv[j].x = rQbarOik_(0, j);
        rQbarOik_cv[j].y = rQbarOik_(1, j);
    }
    cv::findFundamentalMat(rQbarOikm1_cv, rQbarOik_cv, cv::FM_RANSAC, 0.5, 0.99, mask_);
    
    int nInliers = std::count(mask_.begin(), mask_.end(), true);
    std::println("Indoor flow: {} inliers, {} outliers", nInliers, mask_.size() - nInliers);

    // Store inlier homogeneous coordinates
    pk_   = Eigen::MatrixXd::Ones(3, nInliers);
    pkm1_ = Eigen::MatrixXd::Ones(3, nInliers);
    int inlierIdx = 0;
    for (int j = 0; j < np; ++j)
    {
        if (mask_[j])
        {
            pkm1_(0, inlierIdx) = rQbarOikm1_(0, j);
            pkm1_(1, inlierIdx) = rQbarOikm1_(1, j);
            pk_(0, inlierIdx) = rQbarOik_(0, j);
            pk_(1, inlierIdx) = rQbarOik_(1, j);
            inlierIdx++;
        }
    }
}

Eigen::VectorXd MeasurementIndoorFlowBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    Eigen::VectorXd y;
    throw std::runtime_error("Not implemented");
    return y;
}

const Eigen::Matrix<double, 2, Eigen::Dynamic> & MeasurementIndoorFlowBundle::trackedPreviousFeatures() const
{
    return rQOikm1_;
}

const Eigen::Matrix<double, 2, Eigen::Dynamic> & MeasurementIndoorFlowBundle::trackedCurrentFeatures() const
{
    return rQOik_;
}

const std::vector<unsigned char> & MeasurementIndoorFlowBundle::inlierMask() const
{
    return mask_;
}

double MeasurementIndoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    return logLikelihoodImpl(x);
}

double MeasurementIndoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual>();
    autodiff::dual fdual;
    g = gradient(&MeasurementIndoorFlowBundle::logLikelihoodImpl<autodiff::dual>, wrt(xdual), at(this, xdual), fdual);
    
    // DEBUG: Print gradient for orientation BEFORE zeroing
    if (g.size() >= 18) {
        std::println("[FLOW GRAD DEBUG] Orientation grad BEFORE zero:");
        std::println("  g(9)={:.3e} g(10)={:.3e} g(11)={:.3e} (roll/pitch/yaw)", g(9), g(10), g(11));
        std::println("  {} inliers, logL={:.2f}", pk_.cols(), val(fdual));
    }
    
    // Zero out gradient for delayed pose (zeta) states (12-17)
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    
    // DEBUG: Print AFTER
    if (g.size() >= 18) {
        std::println("[FLOW GRAD DEBUG] Orientation grad AFTER zero:");
        std::println("  g(9)={:.3e} g(10)={:.3e} g(11)={:.3e}", g(9), g(10), g(11));
    }
    
    return val(fdual);
}

double MeasurementIndoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    Eigen::Matrix<autodiff::dual2nd, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual2nd>();
    autodiff::dual2nd fdual;
    H = hessian(&MeasurementIndoorFlowBundle::logLikelihoodImpl<autodiff::dual2nd>, wrt(xdual), at(this, xdual), fdual, g);
    
    // DEBUG: Print Hessian diagonal for orientation BEFORE zeroing zeta
    if (H.rows() >= 18) {
        std::println("[FLOW HESSIAN DEBUG] Orientation diag BEFORE zero:");
        std::println("  H(9,9)={:.3e} H(10,10)={:.3e} H(11,11)={:.3e} (roll/pitch/yaw)",
                   H(9,9), H(10,10), H(11,11));
        std::println("  {} inliers contributed", pk_.cols());
    }
    
    // Zero out delayed pose gradient and Hessian
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    if (H.rows() >= 18 && H.cols() >= 18) {
        H.block<6, Eigen::Dynamic>(12, 0, 6, H.cols()).setZero();
        H.block<Eigen::Dynamic, 6>(0, 12, H.rows(), 6).setZero();
    }
    
    // DEBUG: Print AFTER zeroing zeta
    if (H.rows() >= 18) {
        std::println("[FLOW HESSIAN DEBUG] Orientation diag AFTER zero:");
        std::println("  H(9,9)={:.3e} H(10,10)={:.3e} H(11,11)={:.3e} (roll/pitch/yaw)",
                   H(9,9), H(10,10), H(11,11));
    }
    
    return val(fdual);
}

Eigen::Matrix<double, 2, Eigen::Dynamic> MeasurementIndoorFlowBundle::predictedFeatures(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    // This is for visualization only - not used in likelihood
    // For indoor (epipolar), we can't predict 2D features, only 1D constraint
    // Return empty for now
    return Eigen::Matrix<double, 2, Eigen::Dynamic>();
}

template <typename Scalar>
Scalar MeasurementIndoorFlowBundle::logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const
{
    // Extract states - FULL 6DOF (position + orientation)
    Eigen::Vector3<Scalar> rBNn_k = x.template segment<3>(6);    // Current position (eta)
    Eigen::Vector3<Scalar> rpy_k = x.template segment<3>(9);     // Current orientation (eta)
    Eigen::Vector3<Scalar> rBNn_km1 = x.template segment<3>(12); // Delayed position (zeta)
    Eigen::Vector3<Scalar> rpy_km1 = x.template segment<3>(15);  // Delayed orientation (zeta)
    
    // Rotation matrices
    Eigen::Matrix3<Scalar> Rnb_k = rpy2rot(rpy_k);
    Eigen::Matrix3<Scalar> Rnb_km1 = rpy2rot(rpy_km1);
    
    // Camera rotation matrices
    Eigen::Matrix3d Rbc = camera_.Tbc.rotationMatrix;
    Eigen::Matrix3<Scalar> Rbc_scalar = Rbc.cast<Scalar>();
    
    Eigen::Matrix3<Scalar> Rnc_k = Rnb_k * Rbc_scalar;
    Eigen::Matrix3<Scalar> Rnc_km1 = Rnb_km1 * Rbc_scalar;
    
    // Camera positions in world frame
    Eigen::Vector3d rBCb = camera_.Tbc.translationVector;
    Eigen::Vector3<Scalar> rBCb_scalar = rBCb.cast<Scalar>();
    
    Eigen::Vector3<Scalar> rCNn_k = rBNn_k + Rnb_k * rBCb_scalar;
    Eigen::Vector3<Scalar> rCNn_km1 = rBNn_km1 + Rnb_km1 * rBCb_scalar;
    
    // Translation from frame k-1 to frame k (in world frame)
    Eigen::Vector3<Scalar> t_n = rCNn_k - rCNn_km1;
    
    // Transform translation to camera frame k
    Eigen::Vector3<Scalar> t_c = Rnc_k.transpose() * t_n;
    
    // Rotation from frame k-1 to frame k
    Eigen::Matrix3<Scalar> R_c = Rnc_k.transpose() * Rnc_km1;
    
    // Build skew-symmetric matrix [t_c]_x
    Eigen::Matrix3<Scalar> t_c_x;
    t_c_x << Scalar(0), -t_c(2), t_c(1),
             t_c(2), Scalar(0), -t_c(0),
             -t_c(1), t_c(0), Scalar(0);
    
    // Fundamental matrix: F = K^{-T} * [t_c]_x * R_c * K^{-1}
    // For normalized coordinates, K = I, so F = [t_c]_x * R_c
    Eigen::Matrix3<Scalar> F = t_c_x * R_c;
    
    Scalar logL = Scalar(0.0);
    
    // For each inlier flow vector
    for (int j = 0; j < pk_.cols(); ++j)
    {
        // Get homogeneous coordinates (already normalized/undistorted)
        Eigen::Vector3<Scalar> p_km1 = pkm1_.col(j).cast<Scalar>();
        Eigen::Vector3<Scalar> p_k = pk_.col(j).cast<Scalar>();
        
        // SAMPSON DISTANCE: First-order geometric error approximation
        // More numerically stable than algebraic error
        
        // Algebraic error (numerator)
        Scalar algebraic_error = p_k.transpose() * F * p_km1;
        
        // Jacobians for normalization (denominator)
        Eigen::Vector3<Scalar> J_k = F.transpose() * p_k;      // ∂(p_k^T F p_{k-1})/∂p_k
        Eigen::Vector3<Scalar> J_km1 = F * p_km1;              // ∂(p_k^T F p_{k-1})/∂p_{k-1}
        
        // Only use first two components (homogeneous coord has w=1, so dw/dx = 0)
        Scalar denom = J_k.template head<2>().squaredNorm() + J_km1.template head<2>().squaredNorm();
        
        // Sampson distance squared
        Scalar sampson_error_sq = (algebraic_error * algebraic_error) / (denom + Scalar(1e-10));
        
        // Gaussian likelihood: N(0, sigma^2) on Sampson distance
        Scalar sigma_scalar = Scalar(sigma_);
        logL += Scalar(-0.5) * sampson_error_sq / (sigma_scalar * sigma_scalar);
        logL += Scalar(-0.5) * log(Scalar(2.0) * Scalar(M_PI) * sigma_scalar * sigma_scalar);
    }
    
    return logL;
}

template <typename Scalar>
Eigen::Matrix<double, 3, Eigen::Dynamic> MeasurementIndoorFlowBundle::predictFlowImpl(const Eigen::VectorX<Scalar> & x,
                                                                                        const Eigen::Matrix<double, 3, Eigen::Dynamic> & pkm1,
                                                                                        const Eigen::Matrix<double, 3, Eigen::Dynamic> & pk) const
{
    // Not used for indoor - return empty
    return Eigen::Matrix<double, 3, Eigen::Dynamic>();
}

// Explicit template instantiations
template double MeasurementIndoorFlowBundle::logLikelihoodImpl<double>(const Eigen::VectorXd &) const;
template autodiff::dual MeasurementIndoorFlowBundle::logLikelihoodImpl<autodiff::dual>(const Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> &) const;
template autodiff::dual2nd MeasurementIndoorFlowBundle::logLikelihoodImpl<autodiff::dual2nd>(const Eigen::Matrix<autodiff::dual2nd, Eigen::Dynamic, 1> &) const;
