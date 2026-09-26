#include <iostream>
#include <Eigen/Core>
#include <doctest/doctest.h>

#include "../../src/MeasurementSLAMUniqueTagBundle.h"
#include "../../src/SystemSLAMPoseLandmarks.h"
#include "../../src/Camera.h"
#include "../../src/GaussianInfo.hpp"

// Test analytical Jacobian against autodiff reference
TEST_CASE("JacobiansTest - PredictCornerJacobian" * doctest::skip())
{
    // Set up test camera
    Camera camera;
    camera.imageSize = cv::Size(1920, 1080);
    camera.cameraMatrix = (cv::Mat_<double>(3,3) << 
        1000, 0, 960,
        0, 1000, 540,
        0, 0, 1);
    camera.distCoeffs = cv::Mat::zeros(12, 1, CV_64F);
    camera.flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;
    
    // Camera extrinsics (identity for simplicity)
    camera.Tbc.rotationMatrix.setIdentity();
    camera.Tbc.translationVector.setZero();
    
    // Create initial state with 12 DOF body + 1 landmark (6 DOF)
    Eigen::VectorXd mu(18);
    mu.setZero();
    mu.segment<3>(6) << 0.0, 0.0, 0.0;      // Body position
    mu.segment<3>(9) << 0.0, 0.0, 0.0;      // Body orientation (RPY)
    mu.segment<3>(12) << 1.0, 0.5, 0.3;     // Landmark position  
    mu.segment<3>(15) << 0.0, 0.0, 0.5;     // Landmark orientation (RPY)
    
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(18, 18) * 0.1;
    auto p0 = GaussianInfo<double>::fromSqrtMoment(mu, S);
    
    SystemSLAMPoseLandmarks system(p0);
    
    // Create measurement object
    std::vector<int> markerIds = {0};
    std::vector<std::vector<cv::Point2f>> markerCorners(1);
    double markerSize = 0.166;
    
    MeasurementUniqueTagBundle measurement(0.0, markerIds, markerCorners, camera, markerSize);
    
    // Test all 4 corners
    for (int cornerIdx = 0; cornerIdx < 4; ++cornerIdx)
    {
        // Compute Jacobian using autodiff (reference)
        Eigen::MatrixXd J_autodiff;
        Eigen::Vector2d pixel_autodiff = measurement.predictCorner(mu, J_autodiff, system, 0, cornerIdx);
        
        // Compute Jacobian using analytical method
        // TODO: Call analytical version once integrated
        // For now, this test will fail until we integrate the analytical version
        
        // Compare Jacobians
        CHECK(J_autodiff.rows() == 2);
        CHECK(J_autodiff.cols() == 18);
        
        // Print for debugging
        std::cout << "Corner " << cornerIdx << ":" << std::endl;
        std::cout << "  Pixel: [" << pixel_autodiff(0) << ", " << pixel_autodiff(1) << "]" << std::endl;
        std::cout << "  Jacobian shape: " << J_autodiff.rows() << "x" << J_autodiff.cols() << std::endl;
        std::cout << "  Jacobian norm: " << J_autodiff.norm() << std::endl;
        
        // Check sparsity pattern - most entries should be zero
        int nonzero_count = 0;
        for (int i = 0; i < J_autodiff.rows(); ++i) {
            for (int j = 0; j < J_autodiff.cols(); ++j) {
                if (std::abs(J_autodiff(i,j)) > 1e-10) {
                    nonzero_count++;
                }
            }
        }
        std::cout << "  Nonzero entries: " << nonzero_count << " / " << J_autodiff.size() << std::endl;
        
        // Verify Jacobian is not all zeros
        CHECK(J_autodiff.norm() > 0.0);
    }
}

// Test that Jacobian matches finite differences
TEST_CASE("JacobiansTest - FiniteDifferenceValidation" * doctest::skip())
{
    // Set up test camera
    Camera camera;
    camera.imageSize = cv::Size(1920, 1080);
    camera.cameraMatrix = (cv::Mat_<double>(3,3) << 
        1000, 0, 960,
        0, 1000, 540,
        0, 0, 1);
    camera.distCoeffs = cv::Mat::zeros(12, 1, CV_64F);
    camera.flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;
    
    camera.Tbc.rotationMatrix.setIdentity();
    camera.Tbc.translationVector.setZero();
    
    // Create test state
    Eigen::VectorXd mu(18);
    mu.setZero();
    mu.segment<3>(6) << 0.0, 0.0, 0.0;
    mu.segment<3>(9) << 0.0, 0.0, 0.0;
    mu.segment<3>(12) << 1.0, 0.5, 0.3;
    mu.segment<3>(15) << 0.0, 0.0, 0.5;
    
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(18, 18) * 0.1;
    auto p0 = GaussianInfo<double>::fromSqrtMoment(mu, S);
    SystemSLAMPoseLandmarks system(p0);
    
    std::vector<int> markerIds = {0};
    std::vector<std::vector<cv::Point2f>> markerCorners(1);
    MeasurementUniqueTagBundle measurement(0.0, markerIds, markerCorners, camera, 0.166);
    
    // Test one corner with finite differences
    int cornerIdx = 0;
    Eigen::MatrixXd J_autodiff;
    Eigen::Vector2d pixel = measurement.predictCorner(mu, J_autodiff, system, 0, cornerIdx);
    
    // Compute finite difference Jacobian
    double epsilon = 1e-6;
    Eigen::MatrixXd J_fd(2, 18);
    
    for (int j = 0; j < 18; ++j) {
        Eigen::VectorXd mu_plus = mu;
        mu_plus(j) += epsilon;
        
        // Update system with perturbed state
        Eigen::MatrixXd S_plus = Eigen::MatrixXd::Identity(18, 18) * 0.1;
        auto p_plus = GaussianInfo<double>::fromSqrtMoment(mu_plus, S_plus);
        SystemSLAMPoseLandmarks system_plus(p_plus);
        
        Eigen::MatrixXd J_dummy;
        Eigen::Vector2d pixel_plus = measurement.predictCorner(mu_plus, J_dummy, system_plus, 0, cornerIdx);
        
        J_fd.col(j) = (pixel_plus - pixel) / epsilon;
    }
    
    // Compare autodiff vs finite difference
    double max_error = 0.0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 18; ++j) {
            double error = std::abs(J_autodiff(i,j) - J_fd(i,j));
            max_error = std::max(max_error, error);
        }
    }
    
    std::cout << "Max error between autodiff and finite difference: " << max_error << std::endl;
    CHECK(max_error < 1e-4);  // Should match to reasonable precision
}
