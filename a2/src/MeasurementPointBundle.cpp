#include "MeasurementPointBundle.h"
#include "GaussianBase.hpp"
#include "Pose.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <iostream>
#include <limits>

MeasurementPointBundle::MeasurementPointBundle(double time, const cv::Mat & image, const Camera & camera)
    : Measurement(time)
    , camera_(camera)
    , sigma_(2.0)  // 2 pixel measurement noise
{
    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    
    // Detect good features to track (Shi-Tomasi corner detector)
    std::vector<cv::Point2f> corners;
    
    // Parameters for goodFeaturesToTrack
    int maxCorners = 40;            // Max features to detect (reduced for speed)
    double qualityLevel = 0.01;     // Quality threshold (relaxed for indoor)
    double minDistance = 15.0;      // Min distance between features (pixels) - increased for less clutter
    int blockSize = 7;              // Corner detection window
    bool useHarrisDetector = false; // Use Shi-Tomasi instead of Harris
    double k = 0.04;                // Free parameter for Harris (unused)
    
    // Detect corners
    cv::goodFeaturesToTrack(gray, corners, maxCorners, qualityLevel, minDistance,
                           cv::Mat(), blockSize, useHarrisDetector, k);
    
    detectedFeatures_ = corners;
    
    std::cout << "  [POINT FEATURES] Detected " << detectedFeatures_.size() << " corners" << std::endl;
}

Measurement * MeasurementPointBundle::clone() const
{
    return new MeasurementPointBundle(*this);
}

void MeasurementPointBundle::update(SystemBase & system_)
{
    SystemVisualNav & system = dynamic_cast<SystemVisualNav &>(system_);
    
    std::cout << "  [POINT BUNDLE UPDATE] Starting map management..." << std::endl;
    std::cout << "    Current: " << system.numberLandmarks() << " landmarks, " 
              << detectedFeatures_.size() << " detected features" << std::endl;
    
    // 1. ASSOCIATE features with existing landmarks
    std::vector<std::size_t> existingLandmarks;
    for (std::size_t i = 0; i < system.numberLandmarks(); ++i) {
        existingLandmarks.push_back(i);
    }
    
    std::vector<int> feature_to_lm;  // feature_to_lm[featIdx] = lmIdx (or -1)
    if (!existingLandmarks.empty()) {
        associations_ = associate(system, existingLandmarks);
        
        // Invert: associations_[lmIdx] = featIdx → feature_to_lm[featIdx] = lmIdx
        feature_to_lm.resize(detectedFeatures_.size(), -1);
        for (std::size_t lmIdx = 0; lmIdx < associations_.size(); ++lmIdx) {
            int featIdx = associations_[lmIdx];
            if (featIdx >= 0 && static_cast<std::size_t>(featIdx) < detectedFeatures_.size()) {
                feature_to_lm[featIdx] = static_cast<int>(lmIdx);
            }
        }
    } else {
        feature_to_lm.resize(detectedFeatures_.size(), -1);
        associations_.clear();
    }
    
    // 2. DELETE landmarks with consecutive failures (convex environment!)
    // Track failure counts (simplified: just check current frame)
    std::vector<bool> landmarkVisible(system.numberLandmarks(), false);
    std::vector<bool> landmarkMatched(system.numberLandmarks(), false);
    
    for (std::size_t lmIdx = 0; lmIdx < system.numberLandmarks(); ++lmIdx) {
        // Check if landmark projects into FOV
        Eigen::VectorXd x = system.density.mean();
        Pose<double> Tnc;
        Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
        Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
        
        std::size_t idx = system.landmarkPositionIndex(lmIdx);
        Eigen::Vector3d rLNn = x.segment<3>(idx);
        Eigen::Vector3d rLCc = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);
        
        cv::Vec3d rLCc_cv(rLCc(0), rLCc(1), rLCc(2));
        if (camera_.isVectorWithinFOV(rLCc_cv) && rLCc(2) > 0.1) {
            landmarkVisible[lmIdx] = true;
            
            // Check if matched
            if (lmIdx < associations_.size() && associations_[lmIdx] >= 0) {
                landmarkMatched[lmIdx] = true;
            }
        }
    }
    
    // For now: NO deletion (keep it simple for first pass)
    // TODO: Track consecutive failures and delete after threshold
    
    // 3. INITIALIZE surplus (unmatched) features up to Nmax
    const std::size_t MAX_LANDMARKS = 20;  // Reduced for speed
    int numSurplus = 0;
    
    for (std::size_t featIdx = 0; featIdx < detectedFeatures_.size(); ++featIdx) {
        // Skip if already matched
        if (feature_to_lm[featIdx] >= 0) continue;
        
        // Check landmark limit
        if (system.numberLandmarks() >= MAX_LANDMARKS) {
            std::cout << "    [LIMIT] Max " << MAX_LANDMARKS << " landmarks reached" << std::endl;
            break;
        }
        
        // Initialize at 1.5m depth (indoor scale)
        const cv::Point2f & pt = detectedFeatures_[featIdx];
        cv::Vec3d rCCc_cv = camera_.pixelToVector(cv::Vec2d(pt.x, pt.y));
        Eigen::Vector3d rCCc(rCCc_cv[0], rCCc_cv[1], rCCc_cv[2]);
        double init_depth = 1.5;
        Eigen::Vector3d rLCc = init_depth * rCCc;
        
        Eigen::VectorXd x = system.density.mean();
        Pose<double> Tnc;
        Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
        Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
        Eigen::Vector3d rLNn = Tnc.translationVector + Tnc.rotationMatrix * rLCc;
        
        // Augment state
        std::size_t currentDim = system.density.dim();
        Eigen::VectorXd augmentedMean(currentDim + 3);
        augmentedMean.head(currentDim) = system.density.mean();
        augmentedMean.tail<3>() = rLNn;
        
        Eigen::MatrixXd augmentedCov(currentDim + 3, currentDim + 3);
        augmentedCov.setZero();
        augmentedCov.topLeftCorner(currentDim, currentDim) = system.density.cov();
        double init_sigma = 0.1;  // 0.1m uncertainty (very tight ellipsoids!)
        augmentedCov.block<3,3>(currentDim, currentDim) = init_sigma * init_sigma * Eigen::Matrix3d::Identity();
        
        system.density = GaussianInfo<double>::fromMoment(augmentedMean, augmentedCov);
        numSurplus++;
    }
    
    if (numSurplus > 0) {
        std::cout << "    [INIT] Initialized " << numSurplus << " new landmarks from surplus features" << std::endl;
    }
    
    std::cout << "    Result: " << system.numberLandmarks() << " total landmarks" << std::endl;
    
    // 4. NOW call base Measurement::update() for optimization
    Measurement::update(system);
}

DensityBase<double> * MeasurementPointBundle::predictFeatureDensity(const SystemVisualNav & system, std::size_t idxLandmark) const
{
    Eigen::VectorXd x = system.density.mean();
    Eigen::MatrixXd Sigma = system.density.cov();
    
    // Get landmark position
    std::size_t idx = system.landmarkPositionIndex(idxLandmark);
    Eigen::Vector3d rLNn = x.segment<3>(idx);
    
    // Get camera pose
    Pose<double> Tnc;
    Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
    Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
    
    // Transform to camera frame
    Eigen::Vector3d rLCc = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);
    
    // Project to image (worldToPixel simplified)
    cv::Vec3d rLCc_cv(rLCc(0), rLCc(1), rLCc(2));
    cv::Vec2d pixel = camera_.vectorToPixel(rLCc_cv);
    
    Eigen::Vector2d mean;
    mean << pixel[0], pixel[1];
    
    Eigen::Matrix2d cov = sigma_ * sigma_ * Eigen::Matrix2d::Identity();
    
    return new GaussianInfo<double>(GaussianInfo<double>::fromMoment(mean, cov));
}

DensityBase<double> * MeasurementPointBundle::predictFeatureBundleDensity(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks) const
{
    int M = idxLandmarks.size();
    Eigen::VectorXd mean(2 * M);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(2 * M, 2 * M);
    
    for (int i = 0; i < M; ++i) {
        auto density = predictFeatureDensity(system, idxLandmarks[i]);
        auto gaussian = dynamic_cast<GaussianInfo<double>*>(density);
        
        mean.segment<2>(2 * i) = gaussian->mean();
        cov.block<2,2>(2 * i, 2 * i) = gaussian->cov();
        
        delete density;
    }
    
    return new GaussianInfo<double>(GaussianInfo<double>::fromMoment(mean, cov));
}

std::vector<int> MeasurementPointBundle::associate(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks)
{
    std::vector<int> associations(idxLandmarks.size(), -1);
    
    if (detectedFeatures_.empty()) {
        return associations;
    }
    
    // Nearest neighbor matching with compatibility test
    const double CHI2_THRESHOLD = 9.21;  // 99% confidence for 2 DOF
    
    for (std::size_t i = 0; i < idxLandmarks.size(); ++i) {
        std::size_t lmIdx = idxLandmarks[i];
        
        // Predict where landmark should appear
        auto density = predictFeatureDensity(system, lmIdx);
        auto gaussian = dynamic_cast<GaussianBase<double>*>(density);
        
        Eigen::Vector2d predicted = gaussian->mean();
        Eigen::Matrix2d Sigma = gaussian->cov();
        Eigen::Matrix2d SigmaInv = Sigma.inverse();
        
        delete density;
        
        // Find nearest compatible feature
        double minMahalanobis = std::numeric_limits<double>::max();
        int bestMatch = -1;
        
        for (std::size_t j = 0; j < detectedFeatures_.size(); ++j) {
            const cv::Point2f & pt = detectedFeatures_[j];
            Eigen::Vector2d measured(pt.x, pt.y);
            Eigen::Vector2d innovation = measured - predicted;
            
            double mahalanobis = innovation.transpose() * SigmaInv * innovation;
            
            if (mahalanobis < CHI2_THRESHOLD && mahalanobis < minMahalanobis) {
                minMahalanobis = mahalanobis;
                bestMatch = static_cast<int>(j);
            }
        }
        
        associations[i] = bestMatch;
    }
    
    return associations;
}

template <typename Scalar>
Scalar MeasurementPointBundle::logLikelihoodImpl(const Eigen::VectorX<Scalar> & x) const
{
    if (associations_.empty()) {
        return Scalar(0.0);
    }
    
    Pose<Scalar> Tnc;
    Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
    Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
    
    Scalar logLikelihood = Scalar(0.0);
    
    for (std::size_t lmIdx = 0; lmIdx < associations_.size(); ++lmIdx) {
        int featIdx = associations_[lmIdx];
        if (featIdx < 0) continue;  // No association
        
        // Get landmark position
        std::size_t idx = 18 + 3 * lmIdx;  // SystemVisualNav offset
        Eigen::Vector3<Scalar> rLNn = x.template segment<3>(idx);
        
        // Transform to camera frame
        Eigen::Vector3<Scalar> rLCc = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);
        
        // Project (simplified, assumes camera_.vectorToPixel works with Scalar)
        // For autodiff, need to manually compute projection
        Scalar fx = Scalar(camera_.cameraMatrix.at<double>(0, 0));
        Scalar fy = Scalar(camera_.cameraMatrix.at<double>(1, 1));
        Scalar cx = Scalar(camera_.cameraMatrix.at<double>(0, 2));
        Scalar cy = Scalar(camera_.cameraMatrix.at<double>(1, 2));
        
        Scalar u = fx * rLCc(0) / rLCc(2) + cx;
        Scalar v = fy * rLCc(1) / rLCc(2) + cy;
        
        // Measurement
        const cv::Point2f & pt = detectedFeatures_[featIdx];
        Scalar uMeas = Scalar(pt.x);
        Scalar vMeas = Scalar(pt.y);
        
        // Gaussian log-likelihood
        Scalar du = u - uMeas;
        Scalar dv = v - vMeas;
        Scalar sigma2 = Scalar(sigma_ * sigma_);
        
        logLikelihood += -0.5 * (du * du + dv * dv) / sigma2;
    }
    
    return logLikelihood;
}

Eigen::VectorXd MeasurementPointBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system_) const
{
    const SystemVisualNav & system = dynamic_cast<const SystemVisualNav &>(system_);
    
    // Return simulated measurements (pixel coordinates for all associated landmarks)
    int numAssoc = 0;
    for (int assoc : associations_) {
        if (assoc >= 0) numAssoc++;
    }
    
    Eigen::VectorXd y(2 * numAssoc);
    int idx = 0;
    
    for (std::size_t lmIdx = 0; lmIdx < associations_.size(); ++lmIdx) {
        if (associations_[lmIdx] < 0) continue;
        
        // Get landmark position
        std::size_t stateIdx = system.landmarkPositionIndex(lmIdx);
        Eigen::Vector3d rLNn = x.segment<3>(stateIdx);
        
        // Get camera pose
        Pose<double> Tnc;
        Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
        Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
        
        // Transform to camera frame
        Eigen::Vector3d rLCc = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);
        
        // Project
        cv::Vec3d rLCc_cv(rLCc(0), rLCc(1), rLCc(2));
        cv::Vec2d pixel = camera_.vectorToPixel(rLCc_cv);
        
        y(2*idx) = pixel[0];
        y(2*idx+1) = pixel[1];
        idx++;
    }
    
    return y;
}

double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    return logLikelihoodImpl<double>(x);
}

double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    // Use autodiff gradient (same pattern as working S4)
    Eigen::Matrix<autodiff::dual, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual>();
    autodiff::dual fdual;
    g = gradient(&MeasurementPointBundle::logLikelihoodImpl<autodiff::dual>, wrt(xdual), at(this, xdual), fdual);
    
    // Zero out delayed pose gradient (states 12-17)
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    
    return val(fdual);
}

double MeasurementPointBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // Use autodiff hessian (same pattern as working S4)
    Eigen::Matrix<autodiff::dual2nd, Eigen::Dynamic, 1> xdual = x.cast<autodiff::dual2nd>();
    autodiff::dual2nd fdual;
    H = hessian(&MeasurementPointBundle::logLikelihoodImpl<autodiff::dual2nd>, wrt(xdual), at(this, xdual), fdual, g);
    
    // Zero out delayed pose gradient and Hessian (states 12-17)
    if (g.size() >= 18) {
        g.segment<6>(12).setZero();
    }
    if (H.rows() >= 18 && H.cols() >= 18) {
        H.block<6, Eigen::Dynamic>(12, 0, 6, H.cols()).setZero();
        H.block<Eigen::Dynamic, 6>(0, 12, H.rows(), 6).setZero();
    }
    
    return val(fdual);
}

// Explicit template instantiation (same as working S4)
template double MeasurementPointBundle::logLikelihoodImpl<double>(const Eigen::VectorXd &) const;
template autodiff::dual MeasurementPointBundle::logLikelihoodImpl<autodiff::dual>(const Eigen::VectorX<autodiff::dual> &) const;
template autodiff::dual2nd MeasurementPointBundle::logLikelihoodImpl<autodiff::dual2nd>(const Eigen::VectorX<autodiff::dual2nd> &) const;
