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
#include "MeasurementOutdoorFlowBundle.h"

MeasurementOutdoorFlowBundle::MeasurementOutdoorFlowBundle(double time, const Camera & camera, const cv::Mat & imgk_raw, const cv::Mat & imgkm1_raw, const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOikm1)
    : Measurement(time)
    , camera_(camera)
    , rQOikm1_(rQOikm1)
    , rQOik_()
    , rQbarOikm1_()
    , rQbarOik_()
    , mask_()
    , pkm1_()
    , pk_()
    , sigma_(10) // Reduced trust: more skeptical of measurements (was 6)
{
    // Update method can be left as default or set explicitly
    // updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;

    // TODO: Lab 11
    const int divisor               = 2;                // Image scaling factor
    const int maxNumFeatures        = 2000;             // Maximum number of features per frame
    const int minNumFeatures        = 1800;             // Minimum number of features per frame
    const double featureQuality     = 0.01;             // Quality threshold for corner detection (0.01-0.05)
    const double minFeatureDistance = 30.0;             // Minimum distance between features in pixels (5-30)

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01);
    cv::Size subPixWinSize(11, 11);     // Window size for subpixel refinement
    cv::Size winSize(21, 21);           // Window size for optical flow

    // Convert images to grayscale
    cv::Mat imgk_gray;
    cv::Mat imgkm1_gray;
    cv::cvtColor(imgk_raw, imgk_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgkm1_raw, imgkm1_gray, cv::COLOR_BGR2GRAY);

    // Scale images
    cv::Mat imgk_scaled;
    cv::Mat imgkm1_scaled;
    cv::resize(imgk_gray, imgk_scaled, cv::Size(), 1.0/divisor, 1.0/divisor);
    cv::resize(imgkm1_gray, imgkm1_scaled, cv::Size(), 1.0/divisor, 1.0/divisor);

    std::vector<cv::Point2f> rQOikm1_scaled;
    if (rQOikm1_.cols() < minNumFeatures)
    {
        // Initialise new features
        // TODO: Lab 11
        cv::goodFeaturesToTrack(imgkm1_scaled, rQOikm1_scaled, maxNumFeatures, featureQuality, minFeatureDistance, cv::Mat(), 3, false, 0.04);
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

    // Track features from frame k-1 to frame k using the scaled images and previous points
    std::vector<cv::Point2f> rQOik_scaled;
    // TODO: Lab 11
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(imgkm1_scaled, imgk_scaled, rQOikm1_scaled, rQOik_scaled, status, err, winSize, 3, termcrit);

    // Keep points that have been matched between both frames
    int np = 0;
    // TODO: Lab 11
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
    std::println("After filtering by status, there are {} associations.", np);

    // Calculate where flow points would be in the unscaled image
    rQOik_.resize(2, np);
    rQOikm1_.resize(2, np);
    for (int j = 0; j < np; ++j)
    {
        rQOik_(0, j) = rQOik_scaled[j].x*divisor;
        rQOik_(1, j) = rQOik_scaled[j].y*divisor;

        rQOikm1_(0, j) = rQOikm1_scaled[j].x*divisor;
        rQOikm1_(1, j) = rQOikm1_scaled[j].y*divisor;
    }

    // Calculate the undistorted location of features
    rQbarOik_.resize(2, np);
    rQbarOikm1_.resize(2, np);
    // TODO: Lab 11
    rQbarOikm1_ = camera_.undistort(rQOikm1_);
    rQbarOik_ = camera_.undistort(rQOik_);

    // Use RANSAC to find fundamental matrix and determine inliers (mask_)
    //
    // Note: We don't actually use the fundamental matrix computed here, it is just used to
    //       determine which undistorted flow vectors are consistent with the epipolar constraint.

    // TODO: Lab 11
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
    std::println("No. inliers = {}, No. outliers  = {}", nInliers, mask_.size() - nInliers);

    // Inlier undistorted homogeneous points (pkm1_ and pk_)
    pk_     = Eigen::MatrixXd::Ones(3, nInliers);
    pkm1_   = Eigen::MatrixXd::Ones(3, nInliers);
    // TODO: Lab 11
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

Eigen::VectorXd MeasurementOutdoorFlowBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    Eigen::VectorXd y;
    throw std::runtime_error("Not implemented");
    return y;
}

const Eigen::Matrix<double, 2, Eigen::Dynamic> & MeasurementOutdoorFlowBundle::trackedPreviousFeatures() const
{
    return rQOikm1_;
}

const Eigen::Matrix<double, 2, Eigen::Dynamic> & MeasurementOutdoorFlowBundle::trackedCurrentFeatures() const
{
    return rQOik_;
}

const std::vector<unsigned char> & MeasurementOutdoorFlowBundle::inlierMask() const
{
    return mask_;
}

double MeasurementOutdoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    return logLikelihoodImpl(x);
}

double MeasurementOutdoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    // Assignment 2: Compute gradients using forward-mode autodiff
    Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual>();
    autodiff::dual fdual;
    g = gradient(&MeasurementOutdoorFlowBundle::logLikelihoodImpl<autodiff::dual>, wrt(xdual), at(this, xdual), fdual);
    
    // CRITICAL FIX: Zero out gradient for delayed pose (zeta) states (indices 12-17)
    // Zeta should only change during time update (stochastic cloning), NOT during measurement update
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    
    return val(fdual);
}

double MeasurementOutdoorFlowBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // Assignment 2: Compute Hessian using forward-mode autodiff
    Eigen::Matrix<autodiff::dual2nd, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual2nd>();
    autodiff::dual2nd fdual;
    H = hessian(&MeasurementOutdoorFlowBundle::logLikelihoodImpl<autodiff::dual2nd>, wrt(xdual), at(this, xdual), fdual, g);
    
    // CRITICAL FIX: Zero out gradient and Hessian rows/cols for delayed pose (zeta) states (12-17)
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    if (H.rows() >= 18 && H.cols() >= 18) {
        H.block<6, Eigen::Dynamic>(12, 0, 6, H.cols()).setZero();  // Zero rows 12-17
        H.block<Eigen::Dynamic, 6>(0, 12, H.rows(), 6).setZero();  // Zero cols 12-17
    }
    
    return val(fdual);
}

Eigen::Matrix<double, 2, Eigen::Dynamic> MeasurementOutdoorFlowBundle::predictedFeatures(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    std::size_t np = rQOik_.cols();

    // Predict undistorted homogeneous image points in current frame
    Eigen::Matrix<double, 3, Eigen::Dynamic> pk(3, np);
    Eigen::Matrix<double, 3, Eigen::Dynamic> pkm1(3, np);
    pkm1.topRows<2>() = rQbarOikm1_;
    pkm1.row(2).setOnes();
    pk.topRows<2>() = rQbarOik_;
    pk.row(2).setOnes();

    Eigen::Matrix<double, 3, Eigen::Dynamic> pk_hat = predictFlowImpl(x, pkm1, pk);
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOik_hat = pk_hat.topRows<2>().array().rowwise()/pk_hat.row(2).array();
    assert(rQbarOik_hat.cols() == np);
    
    // Compute image coordinates (with lens distortion)
    double fx = camera_.cameraMatrix.at<double>( 0,  0);
    double fy = camera_.cameraMatrix.at<double>( 1,  1);
    double cx = camera_.cameraMatrix.at<double>( 0,  2);
    double cy = camera_.cameraMatrix.at<double>( 1,  2);
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOik_hat;
    // TODO: Lab 11
    rQOik_hat = camera_.distort(rQbarOik_hat);
    return rQOik_hat;
}

// Note: costOdometry is used only in Lab 11, not Assignment 2.
double MeasurementOutdoorFlowBundle::costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1) const
{
    return costOdometryImpl(etak, etakm1);
}

double MeasurementOutdoorFlowBundle::costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1, Eigen::VectorXd & g) const
{
    // Forward-mode autodifferentiation
    Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> etakdual = etak.cast<autodiff::dual>();
    autodiff::dual fdual;
    g = gradient(&MeasurementOutdoorFlowBundle::costOdometryImpl<autodiff::dual>, wrt(etakdual), at(this, etakdual, etakm1), fdual);
    return val(fdual);
}

double MeasurementOutdoorFlowBundle::costOdometry(const Eigen::VectorXd & etak, const Eigen::VectorXd & etakm1, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // Forward-mode autodifferentiation
    Eigen::Matrix<autodiff::dual2nd, Eigen::Dynamic, 1> etakdual = etak.cast<autodiff::dual2nd>();
    autodiff::dual2nd fdual;
    H = hessian(&MeasurementOutdoorFlowBundle::costOdometryImpl<autodiff::dual2nd>, wrt(etakdual), at(this, etakdual, etakm1), fdual, g);
    return val(fdual);
}
