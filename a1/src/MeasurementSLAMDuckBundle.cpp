#include <cmath>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include "MeasurementSLAMDuckBundle.h"
#include "GaussianInfo.hpp"
#include "SystemSLAM.h"
#include "association_util.h"

// Helper function declarations at top
template <typename Scalar>
std::pair<Eigen::Vector2<Scalar>, Scalar> predictCentroidAndArea(
    const Eigen::VectorX<Scalar> & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera);

std::pair<Eigen::Vector2d, double> predictCentroidAndArea(
    const Eigen::VectorXd & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera, 
    Eigen::MatrixXd & J);

MeasurementDuckBundle::MeasurementDuckBundle(double time, const std::vector<cv::Point2f> & centroids,
                                           const std::vector<int> & areas, const Camera & camera, double markerSize)
    : MeasurementSLAM(time, camera), centroids_(centroids), areas_(areas)
{
    sigma_centroid_ = 5.0;     // pixels - centroid measurement error
    sigma_area_ = 1000.0;       // pixel area - stronger depth constraint to prevent landmark drift
    idxFeatures_.resize(centroids.size(), -1);
    
    // Use same update method as scenario 1 for consistency
    updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
}

MeasurementSLAM * MeasurementDuckBundle::clone() const
{
    return new MeasurementDuckBundle(*this);
}

Eigen::VectorXd MeasurementDuckBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    // This would return simulated measurement given true state x
    // For simplicity, return actual measurements for now
    Eigen::VectorXd z(3 * centroids_.size());
    for (size_t i = 0; i < centroids_.size(); ++i)
    {
        z.segment<2>(3*i) << centroids_[i].x, centroids_[i].y;
        z(3*i + 2) = areas_[i];
    }
    return z;
}

double MeasurementDuckBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    const SystemSLAM & systemSLAM = dynamic_cast<const SystemSLAM &>(system);
    
    double logLik = 0.0;
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    
    // idxFeatures_[landmark] = detection, so we need to iterate over landmarks
    for (std::size_t landmarkIdx = 0; landmarkIdx < idxFeatures_.size(); ++landmarkIdx)
    {
        int detectionIdx = idxFeatures_[landmarkIdx];
        
        if (detectionIdx < 0 || static_cast<std::size_t>(detectionIdx) >= centroids_.size())
        {
            logLik -= std::log(imageArea);  // Penalty for unassociated
            continue;
        }
        
        std::size_t i = static_cast<std::size_t>(detectionIdx);
        
        // Predict centroid and area for this landmark
        Eigen::MatrixXd J;
        auto [predicted_centroid, predicted_area] = predictCentroidAndArea(x, systemSLAM, static_cast<std::size_t>(landmarkIdx), camera_, J);
        
        // Measured centroid
        Eigen::Vector2d measured_centroid(centroids_[i].x, centroids_[i].y);
        
        // Centroid likelihood: log N(y_centroid; predicted, σ²_c I)
        Eigen::Vector2d centroid_error = measured_centroid - predicted_centroid;
        double mahalanobis_centroid = centroid_error.squaredNorm() / (sigma_centroid_ * sigma_centroid_);
        double centroidLogLik = -0.5 * mahalanobis_centroid - std::log(2.0 * M_PI * sigma_centroid_ * sigma_centroid_);
        
        // Area likelihood: log N(A_measured; A_predicted, σ²_a)
        double measured_area = static_cast<double>(areas_[i]);
        double area_error = measured_area - predicted_area;
        double mahalanobis_area = (area_error * area_error) / (sigma_area_ * sigma_area_);
        double areaLogLik = -0.5 * mahalanobis_area - std::log(std::sqrt(2.0 * M_PI) * sigma_area_);
        
        logLik += centroidLogLik + areaLogLik;
    }
    
    return logLik;
}

double MeasurementDuckBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    const SystemSLAM & systemSLAM = dynamic_cast<const SystemSLAM &>(system);
    
    // Initialize gradient
    g.resize(x.size());
    g.setZero();
    
    double logLik = 0.0;
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    
    // Measurement covariance (scalar for 2D centroid)
    double sigma2 = sigma_centroid_ * sigma_centroid_;
    double inv_sigma2 = 1.0 / sigma2;
    
    // idxFeatures_[landmark] = detection, so we need to iterate over landmarks
    for (std::size_t landmarkIdx = 0; landmarkIdx < idxFeatures_.size(); ++landmarkIdx)
    {
        int detectionIdx = idxFeatures_[landmarkIdx];
        
        if (detectionIdx < 0 || static_cast<std::size_t>(detectionIdx) >= centroids_.size())
        {
            logLik -= std::log(imageArea);  // Penalty for unassociated
            continue;
        }
        
        std::size_t i = static_cast<std::size_t>(detectionIdx);
        
        // Predict centroid and area with BOTH Jacobians using autodiff
        using autodiff::dual;
        Eigen::VectorX<dual> xdual = x.cast<dual>();
        
        // Centroid Jacobian
        auto centroid_func = [&](const Eigen::VectorX<dual>& xin) -> Eigen::Vector2<dual> {
            auto [c, a] = predictCentroidAndArea<dual>(xin, systemSLAM, static_cast<std::size_t>(landmarkIdx), camera_);
            return c;
        };
        Eigen::Vector2<dual> centroid_dual;
        Eigen::MatrixXd J_centroid = autodiff::jacobian(centroid_func, autodiff::wrt(xdual), autodiff::at(xdual), centroid_dual);
        
        // Area Jacobian
        auto area_func = [&](const Eigen::VectorX<dual>& xin) -> dual {
            auto [c, a] = predictCentroidAndArea<dual>(xin, systemSLAM, static_cast<std::size_t>(landmarkIdx), camera_);
            return a;
        };
        dual area_dual;
        Eigen::VectorXd J_area = autodiff::gradient(area_func, autodiff::wrt(xdual), autodiff::at(xdual), area_dual);
        
        // Get predicted values
        Eigen::Vector2d predicted_centroid(autodiff::val(centroid_dual[0]), autodiff::val(centroid_dual[1]));
        double predicted_area = autodiff::val(area_dual);
        
        // Measured centroid and area
        Eigen::Vector2d measured_centroid(centroids_[i].x, centroids_[i].y);
        double measured_area = static_cast<double>(areas_[i]);
        
        // Centroid innovation (measurement residual)
        Eigen::Vector2d centroid_innov = measured_centroid - predicted_centroid;
        
        // Centroid log-likelihood: log N(y; h(x), σ²_c I)
        double mahalanobis_centroid = centroid_innov.squaredNorm() * inv_sigma2;
        double centroidLogLik = -0.5 * mahalanobis_centroid - std::log(2.0 * M_PI * sigma2);
        
        // Area log-likelihood: log N(A; A_pred, σ²_a)
        double area_error = measured_area - predicted_area;
        double sigma_a2 = sigma_area_ * sigma_area_;
        double inv_sigma_a2 = 1.0 / sigma_a2;
        double mahalanobis_area = (area_error * area_error) * inv_sigma_a2;
        double areaLogLik = -0.5 * mahalanobis_area - std::log(std::sqrt(2.0 * M_PI) * sigma_area_);
        
        logLik += centroidLogLik + areaLogLik;
        
        // COMPLETE gradient: both centroid AND area terms!
        // ∂/∂x logp = J_c^T * (1/σ²_c) * error_c + J_a^T * (1/σ²_a) * error_a
        g += J_centroid.transpose() * (inv_sigma2 * centroid_innov);
        g += J_area * (inv_sigma_a2 * area_error);
    }
    
    return logLik;
}

double MeasurementDuckBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    // Hessian version - simplified for now
    H = Eigen::MatrixXd::Zero(x.size(), x.size());
    return logLikelihood(x, system, g);
}

GaussianInfo<double> MeasurementDuckBundle::predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const
{
    // Predict centroid using Jacobian version
    Eigen::MatrixXd J;
    auto [predicted_centroid, predicted_area] = predictCentroidAndArea(system.density.mean(), system, idxLandmark, camera_, J);

    // Create Gaussian density using affine transform
    const std::size_t nx = system.density.dim();
    const std::size_t ny = 2;

    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        assert(xv.size() == nx + ny);
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        Eigen::MatrixXd Jtemp;
        auto [centroid, area] = predictCentroidAndArea(x, system, idxLandmark, camera_, Jtemp);
        
        Eigen::VectorXd ya = centroid + v;
        Ja.resize(ny, nx + ny);
        Ja << Jtemp, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };
    
    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_centroid_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

GaussianInfo<double> MeasurementDuckBundle::predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t nx = system.density.dim();
    const std::size_t n = idxLandmarks.size();
    const std::size_t ny = 2 * n;

    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        assert(xv.size() == nx + ny);
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        Eigen::VectorXd predictions(2 * n);
        Eigen::MatrixXd J(2 * n, nx);
        
        for (size_t i = 0; i < n; ++i)
        {
            Eigen::MatrixXd Ji;
            auto [centroid, area] = predictCentroidAndArea(x, system, idxLandmarks[i], camera_, Ji);
            predictions.segment<2>(2*i) = centroid;
            J.block(2*i, 0, 2, nx) = Ji;
        }
        
        Eigen::VectorXd ya = predictions + v;
        Ja.resize(ny, nx + ny);
        Ja << J, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };

    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_centroid_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

const std::vector<int> & MeasurementDuckBundle::associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks)
{
    // Use SNN data association from Lab 9
    // Create measurement matrix Y (2 x m) where m is number of detected ducks
    Eigen::Matrix<double, 2, Eigen::Dynamic> Y(2, centroids_.size());
    for (size_t i = 0; i < centroids_.size(); ++i)
    {
        Y.col(i) << centroids_[i].x, centroids_[i].y;
    }

    // Get predicted feature bundle density
    GaussianInfo<double> featureBundleDensity = predictFeatureBundleDensity(system, idxLandmarks);

    // Run SNN association
    double surprisal = snn(system, featureBundleDensity, idxLandmarks, Y, camera_, idxFeatures_);
    
    // Apply pre-associations (overwrite SNN results for newly initialized landmarks)
    // But check for conflicts - don't steal detections already claimed by other landmarks
    for (const auto& pair : preAssociations_) {
        std::size_t landmarkIdx = pair.first;
        std::size_t detectionIdx = pair.second;
        
        // Check if this detection is already claimed by another landmark
        bool alreadyClaimed = false;
        std::size_t claimingLandmarkIdx = 0;
        for (size_t i = 0; i < idxLandmarks.size(); ++i) {
            if (static_cast<std::size_t>(idxFeatures_[i]) == detectionIdx && idxLandmarks[i] != landmarkIdx) {
                alreadyClaimed = true;
                claimingLandmarkIdx = idxLandmarks[i];
                break;
            }
        }
        
        if (alreadyClaimed) {
            std::cout << "  [PRE-ASSOC CONFLICT] LM" << landmarkIdx << " wants Detection #" << detectionIdx 
                      << " but it's already claimed by LM" << claimingLandmarkIdx << " - SKIPPING" << std::endl;
            continue;
        }
        
        // Find this landmark in idxLandmarks and apply pre-association
        for (size_t i = 0; i < idxLandmarks.size(); ++i) {
            if (idxLandmarks[i] == landmarkIdx) {
                if (detectionIdx < centroids_.size()) {
                    idxFeatures_[i] = static_cast<int>(detectionIdx);
                    std::cout << "  [PRE-ASSOC] LM" << landmarkIdx << " -> Detection #" << detectionIdx << std::endl;
                }
                break;
            }
        }
    }

    return idxFeatures_;
}

void MeasurementDuckBundle::update(SystemBase & system)
{
    SystemSLAM & systemSLAM = dynamic_cast<SystemSLAM &>(system);
    
    // Perform data association
    std::vector<std::size_t> landmarkIndices;
    for (std::size_t i = 0; i < systemSLAM.numberLandmarks(); ++i) {
        landmarkIndices.push_back(i);
    }
    
    // Run association to populate idxFeatures_
    if (!landmarkIndices.empty()) {
        associate(systemSLAM, landmarkIndices);
    }
    
    // Call base class update to refine state estimate
    Measurement::update(system);
}

// Templated helper function implementation
template <typename Scalar>
std::pair<Eigen::Vector2<Scalar>, Scalar> predictCentroidAndArea(
    const Eigen::VectorX<Scalar> & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera)
{
    // Get landmark position from state
    std::size_t idx = system.landmarkPositionIndex(idxLandmark);
    
    // Bounds check
    if (idx + 3 > static_cast<std::size_t>(x.size())) {
        std::cout << "  [ERROR] Out of bounds! idxLandmark=" << idxLandmark 
                  << ", idx=" << idx 
                  << ", need " << (idx+3) << ", but x.size()=" << x.size() 
                  << ", numberLandmarks=" << system.numberLandmarks() << std::endl;
        throw std::runtime_error("Landmark index out of bounds in predictCentroidAndArea");
    }
    
    Eigen::Vector3<Scalar> rLNn = x.template segment<3>(idx);  // Landmark position

    // Get camera pose
    Pose<Scalar> Tnc;
    Tnc.translationVector = SystemSLAM::cameraPosition(camera, x);
    Tnc.rotationMatrix = SystemSLAM::cameraOrientation(camera, x);

    // Transform landmark to camera frame
    Eigen::Vector3<Scalar> rLCc = Tnc.rotationMatrix.transpose() * (rLNn - Tnc.translationVector);

    // Project to pixel coordinates (centroid prediction)
    Eigen::Vector2<Scalar> predicted_centroid = camera.vectorToPixel(rLCc);

    // Area prediction per PDF Equation 11: A = (f_x * f_y * π * r²) / distance²
    Scalar distance = rLCc.norm();
    Scalar f_x = static_cast<Scalar>(camera.cameraMatrix.at<double>(0, 0));
    Scalar f_y = static_cast<Scalar>(camera.cameraMatrix.at<double>(1, 1));
    Scalar r = static_cast<Scalar>(0.018);  // Duck characteristic radius in meters (18mm)
    Scalar pi = static_cast<Scalar>(M_PI);
    Scalar predicted_area = (f_x * f_y * pi * r * r) / (distance * distance);

    return {predicted_centroid, predicted_area};
}

// Explicit instantiation for double
template std::pair<Eigen::Vector2d, double> predictCentroidAndArea<double>(
    const Eigen::VectorXd &, const SystemSLAM &, std::size_t, const Camera &);

// Jacobian wrapper using autodiff
std::pair<Eigen::Vector2d, double> predictCentroidAndArea(
    const Eigen::VectorXd & x, 
    const SystemSLAM & system, 
    std::size_t idxLandmark, 
    const Camera & camera, 
    Eigen::MatrixXd & J)
{
    using autodiff::dual;
    Eigen::VectorX<dual> xdual = x.cast<dual>();
    
    auto f = [&](const Eigen::VectorX<dual>& xin) -> Eigen::Vector2<dual> {
        auto [centroid, area] = predictCentroidAndArea<dual>(xin, system, idxLandmark, camera);
        return centroid;
    };

    Eigen::Vector2<dual> ydual;
    J = autodiff::jacobian(f, autodiff::wrt(xdual), autodiff::at(xdual), ydual);

    Eigen::Vector2d y;
    y[0] = autodiff::val(ydual[0]);
    y[1] = autodiff::val(ydual[1]);
    
    // Get area without autodiff (simpler)
    auto [centroid, area] = predictCentroidAndArea<double>(x, system, idxLandmark, camera);
    
    return {y, area};
}
