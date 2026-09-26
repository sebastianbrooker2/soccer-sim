#include <cmath>
#include <iostream>
#include <map>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include "MeasurementIdenticalTagBundle.h"
#include "GaussianInfo.hpp"
#include "SystemVisualNav.h"
#include "association_util.h"

// Helper to predict tag corners from 3D landmark position
template <typename Scalar>
Eigen::Matrix<Scalar, 8, 1> predictTagCorners(
    const Eigen::VectorX<Scalar> & x,
    const SystemVisualNav & system,
    std::size_t idxLandmark,
    const Camera & camera,
    double tagSize);

Eigen::Matrix<double, 8, 1> predictTagCorners(
    const Eigen::VectorXd & x,
    const SystemVisualNav & system,
    std::size_t idxLandmark,
    const Camera & camera,
    double tagSize,
    Eigen::MatrixXd & J);

MeasurementIdenticalTagBundle::MeasurementIdenticalTagBundle(double time, const cv::Mat & image, const Camera & camera, double tagSize)
    : MeasurementSLAM(time, camera)
    , tagSize_(tagSize)
    , sigma_(10.0)  // 10 pixels corner uncertainty (MATCHED TO A1 SCENARIO 1)
{
    // Detect AprilTags
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }
    
    // EXACT A1 SETTINGS: DICT_6X6_250 with default parameters
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    
    std::vector<int> ids;
    detector.detectMarkers(gray, corners_, ids);
    
    std::cout << "[APRILTAG] Detected " << corners_.size() << " tags";
    if (!ids.empty()) {
        std::cout << " (IDs: ";
        for (size_t i = 0; i < ids.size(); ++i) {
            std::cout << ids[i];
            if (i < ids.size()-1) std::cout << ",";
        }
        std::cout << ")";
    }
    std::cout << std::endl;
    
    idxFeatures_.clear();
}

MeasurementSLAM * MeasurementIdenticalTagBundle::clone() const
{
    return new MeasurementIdenticalTagBundle(*this);
}

Eigen::VectorXd MeasurementIdenticalTagBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    Eigen::VectorXd z(8 * corners_.size());
    for (size_t i = 0; i < corners_.size(); ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            z(8*i + 2*j) = corners_[i][j].x;
            z(8*i + 2*j + 1) = corners_[i][j].y;
        }
    }
    return z;
}

double MeasurementIdenticalTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    const SystemVisualNav & systemVisualNav = dynamic_cast<const SystemVisualNav &>(system);
    
    double logLik = 0.0;
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    double sigma2 = sigma_ * sigma_;
    
    for (std::size_t landmarkIdx = 0; landmarkIdx < idxFeatures_.size(); ++landmarkIdx)
    {
        int detectionIdx = idxFeatures_[landmarkIdx];
        
        if (detectionIdx < 0 || static_cast<std::size_t>(detectionIdx) >= corners_.size())
        {
            logLik -= std::log(imageArea);
            continue;
        }
        
        std::size_t i = static_cast<std::size_t>(detectionIdx);
        
        // Predict 4 corners
        Eigen::MatrixXd J;
        Eigen::Matrix<double, 8, 1> predicted = predictTagCorners(x, systemVisualNav, landmarkIdx, camera_, tagSize_, J);
        
        // Measure 4 corners
        Eigen::Matrix<double, 8, 1> measured;
        for (int j = 0; j < 4; ++j)
        {
            measured(2*j) = corners_[i][j].x;
            measured(2*j + 1) = corners_[i][j].y;
        }
        
        // Gaussian likelihood for all 4 corners
        Eigen::Matrix<double, 8, 1> error = measured - predicted;
        double mahalanobis = error.squaredNorm() / sigma2;
        logLik += -0.5 * mahalanobis - 4.0 * std::log(2.0 * M_PI * sigma2);
    }
    
    return logLik;
}

double MeasurementIdenticalTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    const SystemVisualNav & systemVisualNav = dynamic_cast<const SystemVisualNav &>(system);
    
    g.resize(x.size());
    g.setZero();
    
    double logLik = 0.0;
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    double inv_sigma2 = 1.0 / (sigma_ * sigma_);
    
    for (std::size_t landmarkIdx = 0; landmarkIdx < idxFeatures_.size(); ++landmarkIdx)
    {
        int detectionIdx = idxFeatures_[landmarkIdx];
        
        if (detectionIdx < 0 || static_cast<std::size_t>(detectionIdx) >= corners_.size())
        {
            logLik -= std::log(imageArea);
            continue;
        }
        
        std::size_t i = static_cast<std::size_t>(detectionIdx);
        
        // Predict with Jacobian
        using autodiff::dual;
        Eigen::VectorX<dual> xdual = x.cast<dual>();
        
        auto corner_func = [&](const Eigen::VectorX<dual>& xin) -> Eigen::Matrix<dual, 8, 1> {
            return predictTagCorners<dual>(xin, systemVisualNav, landmarkIdx, camera_, tagSize_);
        };
        
        Eigen::Matrix<dual, 8, 1> predicted_dual;
        Eigen::MatrixXd J = autodiff::jacobian(corner_func, autodiff::wrt(xdual), autodiff::at(xdual), predicted_dual);
        
        Eigen::Matrix<double, 8, 1> predicted;
        for (int k = 0; k < 8; ++k) {
            predicted(k) = autodiff::val(predicted_dual(k));
        }
        
        // Measured corners
        Eigen::Matrix<double, 8, 1> measured;
        for (int j = 0; j < 4; ++j)
        {
            measured(2*j) = corners_[i][j].x;
            measured(2*j + 1) = corners_[i][j].y;
        }
        
        Eigen::Matrix<double, 8, 1> error = measured - predicted;
        double mahalanobis = error.squaredNorm() * inv_sigma2;
        logLik += -0.5 * mahalanobis - 4.0 * std::log(2.0 * M_PI * sigma_ * sigma_);
        
        g += J.transpose() * (inv_sigma2 * error);
    }
    
    return logLik;
}

double MeasurementIdenticalTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    H = Eigen::MatrixXd::Zero(x.size(), x.size());
    return logLikelihood(x, system, g);
}

// SystemSLAM interface wrappers (cast to SystemVisualNav)
GaussianInfo<double> MeasurementIdenticalTagBundle::predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const
{
    const SystemVisualNav & systemVisualNav = dynamic_cast<const SystemVisualNav &>(system);
    return predictFeatureDensity(systemVisualNav, idxLandmark);
}

GaussianInfo<double> MeasurementIdenticalTagBundle::predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const SystemVisualNav & systemVisualNav = dynamic_cast<const SystemVisualNav &>(system);
    return predictFeatureBundleDensity(systemVisualNav, idxLandmarks);
}

const std::vector<int> & MeasurementIdenticalTagBundle::associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks)
{
    const SystemVisualNav & systemVisualNav = dynamic_cast<const SystemVisualNav &>(system);
    return associate(const_cast<SystemVisualNav &>(systemVisualNav), idxLandmarks);
}

// SystemVisualNav implementations
GaussianInfo<double> MeasurementIdenticalTagBundle::predictFeatureDensity(const SystemVisualNav & system, std::size_t idxLandmark) const
{
    Eigen::MatrixXd J;
    Eigen::Matrix<double, 8, 1> predicted = predictTagCorners(system.density.mean(), system, idxLandmark, camera_, tagSize_, J);
    
    const std::size_t nx = system.density.dim();
    const std::size_t ny = 8;
    
    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        Eigen::MatrixXd Jtemp;
        Eigen::Matrix<double, 8, 1> corners = predictTagCorners(x, system, idxLandmark, camera_, tagSize_, Jtemp);
        
        Eigen::VectorXd ya = corners + v;
        Ja.resize(ny, nx + ny);
        Ja << Jtemp, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };
    
    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

GaussianInfo<double> MeasurementIdenticalTagBundle::predictFeatureBundleDensity(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t nx = system.density.dim();
    const std::size_t n = idxLandmarks.size();
    const std::size_t ny = 8 * n;  // 8 values per tag (4 corners × 2 coords)
    
    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        Eigen::VectorXd predictions(8 * n);
        Eigen::MatrixXd J(8 * n, nx);
        
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::MatrixXd Ji;
            Eigen::Matrix<double, 8, 1> corners = predictTagCorners(x, system, idxLandmarks[i], camera_, tagSize_, Ji);
            predictions.segment<8>(8*i) = corners;
            J.block(8*i, 0, 8, nx) = Ji;
        }
        
        Eigen::VectorXd ya = predictions + v;
        Ja.resize(ny, nx + ny);
        Ja << J, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };
    
    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

const std::vector<int> & MeasurementIdenticalTagBundle::associate(const SystemVisualNav & system, const std::vector<std::size_t> & idxLandmarks)
{
    // 1. Create 2D measurement matrix Y with tag centroids
    Eigen::Matrix<double, 2, Eigen::Dynamic> Y(2, corners_.size());
    for (size_t i = 0; i < corners_.size(); ++i)
    {
        // Compute centroid of 4 corners
        float cx = 0.0f, cy = 0.0f;
        for (int j = 0; j < 4; ++j)
        {
            cx += corners_[i][j].x;
            cy += corners_[i][j].y;
        }
        Y.col(i) << cx / 4.0, cy / 4.0;
    }
    
    // 2. Build 2D centroid-based feature bundle density
    const std::size_t nx = system.density.dim();
    const std::size_t n = idxLandmarks.size();
    const std::size_t ny = 2 * n;  // 2D centroid per landmark
    
    const auto centroid_func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        Eigen::VectorXd centroids(2 * n);
        Eigen::MatrixXd J(2 * n, nx);
        
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::MatrixXd Ji_corners;
            Eigen::Matrix<double, 8, 1> corners = predictTagCorners(x, system, idxLandmarks[i], camera_, tagSize_, Ji_corners);
            
            // Average to get centroid
            double cx = 0.0, cy = 0.0;
            for (int j = 0; j < 4; ++j)
            {
                cx += corners(2*j);
                cy += corners(2*j + 1);
            }
            centroids(2*i) = cx / 4.0;
            centroids(2*i + 1) = cy / 4.0;
            
            // Jacobian: average of corner Jacobians
            J.row(2*i) = (Ji_corners.row(0) + Ji_corners.row(2) + Ji_corners.row(4) + Ji_corners.row(6)) / 4.0;
            J.row(2*i + 1) = (Ji_corners.row(1) + Ji_corners.row(3) + Ji_corners.row(5) + Ji_corners.row(7)) / 4.0;
        }
        
        Eigen::VectorXd ya = centroids + v;
        Ja.resize(ny, nx + ny);
        Ja << J, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };
    
    // Simple nearest-neighbor association (SNN disabled - compatibility issue  with SystemVisualNav)
    double sigma_centroid = sigma_ / 2.0;
    
    idxFeatures_.resize(idxLandmarks.size(), -1);
    
    // Simple nearest-neighbor matching WITH DEBUG
    Eigen::VectorXd stateMean = system.density.mean();
    const double THRESHOLD = 100.0;  // 100px threshold
    
    
    for (size_t lm = 0; lm < idxLandmarks.size(); ++lm)
    {
        // Predict centroid for this landmark
        Eigen::MatrixXd J_corners;
        Eigen::Matrix<double, 8, 1> corners = predictTagCorners(stateMean, system, idxLandmarks[lm], camera_, tagSize_, J_corners);
        
        double pred_cx = (corners(0) + corners(2) + corners(4) + corners(6)) / 4.0;
        double pred_cy = (corners(1) + corners(3) + corners(5) + corners(7)) / 4.0;
        
        // Get world position of landmark
        std::size_t idx = system.landmarkPositionIndex(idxLandmarks[lm]);
        Eigen::Vector3d worldPos = stateMean.segment<3>(idx);
        
        double minDist = std::numeric_limits<double>::infinity();
        int bestDet = -1;
        
        for (size_t det = 0; det < corners_.size(); ++det)
        {
            double dx = Y(0, det) - pred_cx;
            double dy = Y(1, det) - pred_cy;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < minDist) {
                minDist = dist;
                bestDet = det;
            }
        }
        
        if (minDist < THRESHOLD) {
            idxFeatures_[lm] = bestDet;
        }
    }
    
    return idxFeatures_;
}

void MeasurementIdenticalTagBundle::update(SystemBase & system)
{
    SystemVisualNav & systemVisualNav = dynamic_cast<SystemVisualNav &>(system);
    
    // Track unmatched frames per LANDMARK only (for deletion - no detection cooldown!)
    static std::vector<int> landmarkUnmatchedFrames;
    
    // Resize landmark tracking if needed
    while (landmarkUnmatchedFrames.size() < systemVisualNav.numberLandmarks()) {
        landmarkUnmatchedFrames.push_back(0);
    }
    
    // 1. Data association for existing landmarks
    std::vector<std::size_t> landmarkIndices;
    for (std::size_t i = 0; i < systemVisualNav.numberLandmarks(); ++i) {
        landmarkIndices.push_back(i);
    }
    
    if (!landmarkIndices.empty()) {
        associate(systemVisualNav, landmarkIndices);
    } else {
        idxFeatures_.clear();
    }
    
    // 2. Delete unmatched landmarks IMMEDIATELY (can't have more landmarks than detections!)
    std::vector<std::size_t> landmarksToDelete;
    for (std::size_t i = 0; i < systemVisualNav.numberLandmarks(); ++i) {
        bool isMatched = (i < idxFeatures_.size() && idxFeatures_[i] >= 0);
        if (!isMatched) {
            landmarksToDelete.push_back(i);
        }
    }
    
    // Delete in reverse order AND clear idxFeatures_ (it's now invalid!)
    if (!landmarksToDelete.empty()) {
        std::cout << "  [DELETE] Removing " << landmarksToDelete.size() << " unmatched landmarks" << std::endl;
        
        for (auto it = landmarksToDelete.rbegin(); it != landmarksToDelete.rend(); ++it) {
            std::size_t lmIdx = *it;
            std::size_t bodyDim = 18;
            std::size_t landmarkDim = 6;
            
            std::vector<int> keepIndices;
            for (std::size_t i = 0; i < bodyDim; ++i) {
                keepIndices.push_back(i);
            }
            for (std::size_t i = 0; i < lmIdx; ++i) {
                for (std::size_t j = 0; j < landmarkDim; ++j) {
                    keepIndices.push_back(bodyDim + i * landmarkDim + j);
                }
            }
            for (std::size_t i = lmIdx + 1; i < systemVisualNav.numberLandmarks(); ++i) {
                for (std::size_t j = 0; j < landmarkDim; ++j) {
                    keepIndices.push_back(bodyDim + i * landmarkDim + j);
                }
            }
            
            Eigen::VectorXi keepIndicesEigen = Eigen::Map<Eigen::VectorXi>(keepIndices.data(), keepIndices.size());
            systemVisualNav.density = systemVisualNav.density.marginal(keepIndicesEigen);
            landmarkUnmatchedFrames.erase(landmarkUnmatchedFrames.begin() + lmIdx);
        }
        
        // CRITICAL: After deletion, ALL landmark indices have shifted!
        // Must rebuild association from scratch (but DON'T clear idxFeatures_ - associate() will resize it)
        landmarkIndices.clear();
        for (std::size_t i = 0; i < systemVisualNav.numberLandmarks(); ++i) {
            landmarkIndices.push_back(i);
        }
        if (!landmarkIndices.empty()) {
            associate(systemVisualNav, landmarkIndices);
        } else {
            idxFeatures_.clear();  // Only clear if no landmarks left
        }
    }
    
    // 3. DO MEASUREMENT UPDATE with existing landmarks
    Measurement::update(system);
    
    // 4. Initialize new landmarks from surplus (unmatched) detections
    const std::size_t MAX_LANDMARKS = 100;
    
    // Track which detections were matched
    std::vector<bool> detectionMatched(corners_.size(), false);
    for (std::size_t lm = 0; lm < idxFeatures_.size(); ++lm) {
        int detIdx = idxFeatures_[lm];
        if (detIdx >= 0 && static_cast<std::size_t>(detIdx) < corners_.size()) {
            detectionMatched[detIdx] = true;
        }
    }
    
    // Initialize surplus detections as new landmarks
    for (size_t detIdx = 0; detIdx < corners_.size(); ++detIdx) {
        // Skip if already matched
        if (detectionMatched[detIdx]) continue;
        
        // Check landmark limit
        if (systemVisualNav.numberLandmarks() >= MAX_LANDMARKS) {
            std::cout << "  [LIMIT] Max " << MAX_LANDMARKS << " landmarks reached" << std::endl;
            break;
        }
        
        // Initialize surplus detection as NEW POSE LANDMARK (6-DOF)
        const auto& curCorners = corners_[detIdx];
        float cx = 0.0f, cy = 0.0f;
        for (const auto& corner : curCorners) {
            cx += corner.x;
            cy += corner.y;
        }
        cx /= 4.0f;
        cy /= 4.0f;
        
        // Initialize at 2m depth (PDF Slide 13)
        cv::Vec3d rCCc_cv = camera_.pixelToVector(cv::Vec2d(cx, cy));
        Eigen::Vector3d rCCc(rCCc_cv[0], rCCc_cv[1], rCCc_cv[2]);
        double init_depth = 2.0;
        Eigen::Vector3d rLCc = init_depth * rCCc;
        
        Eigen::VectorXd x = systemVisualNav.density.mean();
        Pose<double> Tnc;
        Tnc.translationVector = SystemVisualNav::cameraPosition(camera_, x);
        Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera_, x);
        Eigen::Vector3d rLNn = Tnc.translationVector + Tnc.rotationMatrix * rLCc;
        
        // Initialize orientation to ZERO (unknown)
        Eigen::Vector3d rpy_tag(0.0, 0.0, 0.0);
        
        // Augment state with 6-DOF pose landmark
        std::size_t currentDim = systemVisualNav.density.dim();
        Eigen::VectorXd augmentedMean(currentDim + 6);
        augmentedMean.head(currentDim) = systemVisualNav.density.mean();
        augmentedMean.segment<3>(currentDim) = rLNn;       // Position
        augmentedMean.segment<3>(currentDim + 3) = rpy_tag; // Orientation (0,0,0)
        
        // Uncertainty (MATCH A1 SCENARIO 1!)
        Eigen::MatrixXd augmentedCov(currentDim + 6, currentDim + 6);
        augmentedCov.setZero();
        augmentedCov.topLeftCorner(currentDim, currentDim) = systemVisualNav.density.cov();
        
        // Position uncertainty: 10m std dev (PDF Slide 13)
        double pos_sigma = 10.0;
        augmentedCov.block<3,3>(currentDim, currentDim) = pos_sigma * pos_sigma * Eigen::Matrix3d::Identity();
        
        // Orientation uncertainty (match A1 S1)
        Eigen::Vector3d ori_std(0.1, 0.1, 0.5);  // rad
        augmentedCov.block<3,3>(currentDim + 3, currentDim + 3) = ori_std.array().square().matrix().asDiagonal();
        
        systemVisualNav.density = GaussianInfo<double>::fromMoment(augmentedMean, augmentedCov);
        landmarkUnmatchedFrames.push_back(0);  // Track new landmark
    }
}

// Helper implementations
template <typename Scalar>
Eigen::Matrix<Scalar, 8, 1> predictTagCorners(
    const Eigen::VectorX<Scalar> & x,
    const SystemVisualNav & system,
    std::size_t idxLandmark,
    const Camera & camera,
    double tagSize)
{
    // Get camera pose
    Pose<Scalar> Tnc;
    Tnc.translationVector = SystemVisualNav::cameraPosition(camera, x);
    Tnc.rotationMatrix = SystemVisualNav::cameraOrientation(camera, x);
    
    // Get landmark position
    std::size_t posIdx = system.landmarkPositionIndex(idxLandmark);
    Eigen::Vector3<Scalar> rLNn = x.template segment<3>(posIdx);
    
    // Check if this is 6-DOF pose landmark or 3-DOF point landmark
    // If 6-DOF, get orientation; otherwise assume tag faces camera
    Eigen::Matrix3<Scalar> Rnl;  // Rotation matrix from landmark to world frame
    
    if (system.hasPoseLandmarks()) {
        // 6-DOF: Get stored orientation (Scenario 5)
        std::size_t oriIdx = system.landmarkOrientationIndex(idxLandmark);
        Eigen::Vector3<Scalar> rpy_l = x.template segment<3>(oriIdx);
        Rnl = rpy2rot(rpy_l);
    } else {
        // 3-DOF: Assume tag faces camera (Scenario 6 - not tags)
        Eigen::Vector3<Scalar> rLCc_unnorm = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);
        Eigen::Vector3<Scalar> zAxis = rLCc_unnorm.normalized();
        
        // Manual cross product (autodiff doesn't support .cross())
        Eigen::Vector3<Scalar> up(Scalar(0), Scalar(0), Scalar(1));
        Eigen::Vector3<Scalar> xAxis;
        xAxis(0) = up(1)*zAxis(2) - up(2)*zAxis(1);
        xAxis(1) = up(2)*zAxis(0) - up(0)*zAxis(2);
        xAxis(2) = up(0)*zAxis(1) - up(1)*zAxis(0);
        xAxis = xAxis.normalized();
        
        if (xAxis.norm() < Scalar(0.1)) {
            Eigen::Vector3<Scalar> right(Scalar(1), Scalar(0), Scalar(0));
            xAxis(0) = right(1)*zAxis(2) - right(2)*zAxis(1);
            xAxis(1) = right(2)*zAxis(0) - right(0)*zAxis(2);
            xAxis(2) = right(0)*zAxis(1) - right(1)*zAxis(0);
            xAxis = xAxis.normalized();
        }
        
        // yAxis = zAxis x xAxis
        Eigen::Vector3<Scalar> yAxis;
        yAxis(0) = zAxis(1)*xAxis(2) - zAxis(2)*xAxis(1);
        yAxis(1) = zAxis(2)*xAxis(0) - zAxis(0)*xAxis(2);
        yAxis(2) = zAxis(0)*xAxis(1) - zAxis(1)*xAxis(0);
        
        Eigen::Matrix3<Scalar> Rcl;
        Rcl.col(0) = xAxis;
        Rcl.col(1) = yAxis;
        Rcl.col(2) = zAxis;
        Rnl = Tnc.rotationMatrix * Rcl;
    }
    
    // Tag corners in tag's local frame (z-axis out of tag)
    Scalar half = Scalar(tagSize / 2.0);
    std::vector<Eigen::Vector3<Scalar>> tagCorners_local = {
        {-half,  half, Scalar(0)},  // Top-left
        { half,  half, Scalar(0)},  // Top-right  
        { half, -half, Scalar(0)},  // Bottom-right
        {-half, -half, Scalar(0)}   // Bottom-left
    };
    
    // Transform corners to world frame, then to camera frame
    Eigen::Matrix<Scalar, 8, 1> corners2D;
    for (int i = 0; i < 4; ++i)
    {
        // Corner in world frame
        Eigen::Vector3<Scalar> rCornerNn = rLNn + Rnl * tagCorners_local[i];
        
        // Corner in camera frame
        Eigen::Vector3<Scalar> rCornerCc = Tnc.rotationMatrix.transpose() * (rCornerNn - Tnc.translationVector);
        
        // Project to image
        Eigen::Vector2<Scalar> pixel = camera.vectorToPixel(rCornerCc);
        corners2D(2*i) = pixel(0);
        corners2D(2*i + 1) = pixel(1);
    }
    
    return corners2D;
}

Eigen::Matrix<double, 8, 1> predictTagCorners(
    const Eigen::VectorXd & x,
    const SystemVisualNav & system,
    std::size_t idxLandmark,
    const Camera & camera,
    double tagSize,
    Eigen::MatrixXd & J)
{
    using autodiff::dual;
    Eigen::VectorX<dual> xdual = x.cast<dual>();
    
    auto f = [&](const Eigen::VectorX<dual>& xin) -> Eigen::Matrix<dual, 8, 1> {
        return predictTagCorners<dual>(xin, system, idxLandmark, camera, tagSize);
    };
    
    Eigen::Matrix<dual, 8, 1> ydual;
    J = autodiff::jacobian(f, autodiff::wrt(xdual), autodiff::at(xdual), ydual);
    
    Eigen::Matrix<double, 8, 1> y;
    for (int i = 0; i < 8; ++i) {
        y(i) = autodiff::val(ydual(i));
    }
    
    return y;
}
