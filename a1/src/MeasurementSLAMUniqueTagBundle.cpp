#include <cstddef>
#include <numeric>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <Eigen/Core>
#include "GaussianInfo.hpp"
#include "SystemBase.h"
#include "SystemEstimator.h"
#include "SystemSLAM.h"
#include "Camera.h"
#include "rotation.hpp"
#include "Measurement.h"
#include "MeasurementSLAM.h"
#include "MeasurementSLAMUniqueTagBundle.h"

// File-scope static lookup table: maps marker ID -> landmark index
// Shared between update() and associate() methods
static std::vector<int> knownIds;

MeasurementUniqueTagBundle::MeasurementUniqueTagBundle(double time,
                                             const std::vector<int> & markerIds,
                                             const std::vector<std::vector<cv::Point2f>> & markerCorners,
                                             const Camera & camera,
                                             double markerSize)
    : MeasurementSLAM(time, camera)
    , markerIds_(markerIds)
    , markerCorners_(markerCorners)
    , markerSize_(markerSize)
    , sigma_(10)
{
    // updateMethod_ = UpdateMethod::BFGSLMSQRT;
    updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
    // updateMethod_ = UpdateMethod::SR1TRUSTEIG;
    // updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
}

MeasurementSLAM * MeasurementUniqueTagBundle::clone() const
{
    return new MeasurementUniqueTagBundle(*this);
}

Eigen::VectorXd MeasurementUniqueTagBundle::simulate(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    Eigen::VectorXd y(markerIds_.size() * 8);
    throw std::runtime_error("Not implemented");
    return y;
}

double MeasurementUniqueTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system) const
{
    const SystemSLAM & systemSLAM = dynamic_cast<const SystemSLAM &>(system);
    
    double logLik = 0.0;
    
    // Image area in pixels
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    
    // Use the stored feature associations from update()
    for (std::size_t i = 0; i < markerIds_.size(); ++i)
    {
        if (i >= idxFeatures_.size())
        {
            continue;
        }
        
        int landmarkIdx = idxFeatures_[i];
        
        if (landmarkIdx < 0 || static_cast<std::size_t>(landmarkIdx) >= systemSLAM.numberLandmarks())
        {
            logLik -= 4.0 * std::log(imageArea);  // Penalty for unassociated
            continue;
        }
        
        // Predict all 4 corners for this landmark
        for (int c = 0; c < 4; ++c)
        {
            Eigen::MatrixXd J;
            Eigen::Vector2d predicted = predictCorner(x, J, systemSLAM, static_cast<std::size_t>(landmarkIdx), c);
            
            // Measured corner
            Eigen::Vector2d measured(markerCorners_[i][c].x, markerCorners_[i][c].y);
            
            // Compute Gaussian log-likelihood: log N(y; predicted, σ²I)
            Eigen::Vector2d error = measured - predicted;
            double mahalanobis = error.squaredNorm() / (sigma_ * sigma_);
            double cornerLogLik = -0.5 * mahalanobis - std::log(2.0 * M_PI * sigma_ * sigma_);
            
            logLik += cornerLogLik;
        }
    }
    
    return logLik;
}

double MeasurementUniqueTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g) const
{
    const SystemSLAM & systemSLAM = dynamic_cast<const SystemSLAM &>(system);
    
    g.resize(x.size());
    g.setZero();
    
    double logLik = 0.0;
    double imageArea = camera_.imageSize.width * camera_.imageSize.height;
    
    // Compute gradient: g = ∑ J^T * R^{-1} * (measured - predicted)
    // For Gaussian with R = σ²I: g = (1/σ²) * ∑ J^T * (measured - predicted)
    
    for (std::size_t i = 0; i < markerIds_.size(); ++i)
    {
        if (i >= idxFeatures_.size())
            continue;
        
        int landmarkIdx = idxFeatures_[i];
        
        if (landmarkIdx < 0 || static_cast<std::size_t>(landmarkIdx) >= systemSLAM.numberLandmarks())
        {
            logLik -= 4.0 * std::log(imageArea);
            continue;
        }
        
        // Process all 4 corners of this marker
        for (int c = 0; c < 4; ++c)
        {
            Eigen::MatrixXd J;  // Jacobian: 2 x nx
            Eigen::Vector2d predicted = predictCorner(x, J, systemSLAM, static_cast<std::size_t>(landmarkIdx), c);
            Eigen::Vector2d measured(markerCorners_[i][c].x, markerCorners_[i][c].y);
            
            // Error: measured - predicted
            Eigen::Vector2d error = measured - predicted;
            
            // Log-likelihood contribution
            double mahalanobis = error.squaredNorm() / (sigma_ * sigma_);
            logLik += -0.5 * mahalanobis - std::log(2.0 * M_PI * sigma_ * sigma_);
            
            // Gradient contribution: g += J^T * R^{-1} * error
            //  = (1/σ²) * J^T * error
            g += (1.0 / (sigma_ * sigma_)) * J.transpose() * error;
        }
    }
    
    return logLik;
}

double MeasurementUniqueTagBundle::logLikelihood(const Eigen::VectorXd & x, const SystemEstimator & system, Eigen::VectorXd & g, Eigen::MatrixXd & H) const
{
    H.resize(x.size(), x.size());
    H.setZero();
    return logLikelihood(x, system, g);
}

void MeasurementUniqueTagBundle::update(SystemBase & system)
{
    SystemSLAM & systemSLAM = dynamic_cast<SystemSLAM &>(system);

    const std::size_t nL = systemSLAM.numberLandmarks();
    
    // Ensure knownIds has slots for all current landmarks
    if (knownIds.size() < nL)
        knownIds.resize(nL, -1);

    idxFeatures_.clear();
    idxFeatures_.reserve(markerIds_.size());

    std::cout << "  [ARUCO UPDATE] Starting association for " << markerIds_.size() << " detected markers" << std::endl;
    std::cout << "  [ARUCO UPDATE] knownIds table: [";
    for (size_t i = 0; i < knownIds.size(); ++i) {
        std::cout << knownIds[i] << (i < knownIds.size()-1 ? ", " : "");
    }
    std::cout << "]" << std::endl;

    // Data association: match detected markers to existing landmarks
    // Note: State augmentation happens in visualNavigation.cpp BEFORE process() is called
    for (std::size_t i = 0; i < markerIds_.size(); ++i)
    {
        const int id = markerIds_[i];
        std::cout << "  [ARUCO UPDATE] Detection " << i << ": Marker ID " << id << std::endl;

        // Search for this marker ID among existing landmarks
        std::size_t found = static_cast<std::size_t>(-1);
        for (std::size_t j = 0; j < nL; ++j)
        {
            if (knownIds[j] == id)
            {
                found = j;
                break;
            }
        }

        if (found != static_cast<std::size_t>(-1))
        {
            // Matched an existing landmark by ID - accept it
            std::cout << "  [ARUCO UPDATE]   -> Found existing landmark " << found << std::endl;
            idxFeatures_.push_back(static_cast<int>(found));
        }
        else
        {
            std::cout << "  [ARUCO UPDATE]   -> NEW marker, need to assign..." << std::endl;
            
            // New landmark - should have been added in visualNavigation.cpp
            // Find the first available slot in knownIds (with value -1)
            std::size_t assignedIdx = nL;  // Start search from current number of landmarks
            
            // Search for first unassigned slot
            for (std::size_t j = 0; j < knownIds.size(); ++j)
            {
                if (knownIds[j] == -1)
                {
                    assignedIdx = j;
                    break;
                }
            }
            
            // If no slot found, this landmark is at the end
            if (assignedIdx >= knownIds.size())
            {
                assignedIdx = knownIds.size();
                knownIds.push_back(id);
                std::cout << "  [ARUCO UPDATE]   -> Assigned to NEW landmark " << assignedIdx << " (expanded knownIds)" << std::endl;
            }
            else
            {
                knownIds[assignedIdx] = id;
                std::cout << "  [ARUCO UPDATE]   -> Assigned to landmark " << assignedIdx << " (reused slot)" << std::endl;
            }
            
            idxFeatures_.push_back(static_cast<int>(assignedIdx));
        }
    }
    
    std::cout << "  [ARUCO UPDATE] Final idxFeatures_ (detection->landmark): [";
    for (size_t i = 0; i < idxFeatures_.size(); ++i) {
        std::cout << idxFeatures_[i] << (i < idxFeatures_.size()-1 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    
    // Call base update to refine state estimate
    Measurement::update(system);
}

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

Eigen::Vector2d MeasurementUniqueTagBundle::predictCorner(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const SystemSLAM & system, std::size_t idxLandmark, int cornerIdx) const
{
    using autodiff::dual;
    Eigen::VectorX<dual> xdual = x.cast<dual>();
    Eigen::Vector2<dual> ydual;

    auto f = [&](const Eigen::VectorX<dual>& xin) -> Eigen::Vector2<dual> {
        return predictCorner<dual>(xin, system, idxLandmark, cornerIdx);
    };

    J = autodiff::jacobian(f, autodiff::wrt(xdual), autodiff::at(xdual), ydual);

    Eigen::Vector2d y;
    y[0] = autodiff::val(ydual[0]);
    y[1] = autodiff::val(ydual[1]);
    return y;
}

GaussianInfo<double> MeasurementUniqueTagBundle::predictFeatureDensity(const SystemSLAM & system, std::size_t idxLandmark) const
{
    const std::size_t & nx = system.density.dim();
    const std::size_t ny = 2;

    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        assert(xv.size() == nx + ny);
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        
        // Predict all 4 corners and compute their center
        Eigen::MatrixXd J0, J1, J2, J3;
        Eigen::Vector2d c0 = predictCorner(x, J0, system, idxLandmark, 0);
        Eigen::Vector2d c1 = predictCorner(x, J1, system, idxLandmark, 1);
        Eigen::Vector2d c2 = predictCorner(x, J2, system, idxLandmark, 2);
        Eigen::Vector2d c3 = predictCorner(x, J3, system, idxLandmark, 3);
        
        // Center is the average of all 4 corners
        Eigen::Vector2d center = (c0 + c1 + c2 + c3) / 4.0;
        Eigen::MatrixXd J = (J0 + J1 + J2 + J3) / 4.0;
        
        Eigen::VectorXd ya = center + v;
        Ja.resize(ny, nx + ny);
        Ja << J, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };
    
    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

Eigen::VectorXd MeasurementUniqueTagBundle::predictCornerBundle(const Eigen::VectorXd & x, Eigen::MatrixXd & J, const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t & nL = idxLandmarks.size();
    const std::size_t & nx = system.density.dim();
    assert(x.size() == nx);

    Eigen::VectorXd h(8 * nL);
    J.resize(8 * nL, nx);
    
    for (std::size_t i = 0; i < nL; ++i)
    {
        for (int c = 0; c < 4; ++c)
        {
            Eigen::MatrixXd Jcorner;
            Eigen::Vector2d corner = predictCorner(x, Jcorner, system, idxLandmarks[i], c);
            h.segment<2>(8 * i + 2 * c) = corner;
            J.block(8 * i + 2 * c, 0, 2, nx) = Jcorner;
        }
    }
    return h;
}

GaussianInfo<double> MeasurementUniqueTagBundle::predictFeatureBundleDensity(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks) const
{
    const std::size_t & nx = system.density.dim();
    const std::size_t ny = 8 * idxLandmarks.size();

    const auto func = [&](const Eigen::VectorXd & xv, Eigen::MatrixXd & Ja)
    {
        assert(xv.size() == nx + ny);
        Eigen::VectorXd x = xv.head(nx);
        Eigen::VectorXd v = xv.tail(ny);
        Eigen::MatrixXd J;
        Eigen::VectorXd ya = predictCornerBundle(x, J, system, idxLandmarks) + v;
        Ja.resize(ny, nx + ny);
        Ja << J, Eigen::MatrixXd::Identity(ny, ny);
        return ya;
    };

    auto pv = GaussianInfo<double>::fromSqrtMoment(sigma_ * Eigen::MatrixXd::Identity(ny, ny));
    auto pxv = system.density * pv;
    return pxv.affineTransform(func);
}

const std::vector<int> & MeasurementUniqueTagBundle::associate(const SystemSLAM & system, const std::vector<std::size_t> & idxLandmarks)
{
    const std::size_t nL = system.numberLandmarks();
    
    // Ensure knownIds has slots for all current landmarks
    if (knownIds.size() < nL)
        knownIds.resize(nL, -1);
    
    idxFeatures_.clear();
    idxFeatures_.resize(markerIds_.size(), -1);
    
    // Data association: match detected markers to existing landmarks by ID
    for (std::size_t i = 0; i < markerIds_.size(); ++i)
    {
        const int id = markerIds_[i];
        
        // Search for this marker ID among existing landmarks
        for (std::size_t j = 0; j < nL; ++j)
        {
            if (knownIds[j] == id)
            {
                // Found matching ID - associate feature i with landmark j
                idxFeatures_[i] = static_cast<int>(j);
                break;
            }
        }
        // If not found, idxFeatures_[i] remains -1 (unassociated)
    }
    
    return idxFeatures_;
}
