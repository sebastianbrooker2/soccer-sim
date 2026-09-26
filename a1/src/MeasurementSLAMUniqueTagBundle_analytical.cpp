// Analytical Jacobian implementation for predictCorner
// This file contains the analytical Jacobian computation to replace autodiff

#include "MeasurementSLAMUniqueTagBundle.h"
#include "rotation.hpp"
#include "SystemSLAM.h"
#include "Camera.h"
#include <Eigen/Core>

// Helper function to compute Jacobian of RPY rotation matrix w.r.t. RPY angles
// Returns dR/droll, dR/dpitch, dR/dyaw as 3x3 matrices
static void computeRPYJacobian(const Eigen::Vector3d& rpy, 
                               Eigen::Matrix3d& dR_droll,
                               Eigen::Matrix3d& dR_dpitch, 
                               Eigen::Matrix3d& dR_dyaw)
{
    double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);
    
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // Compute partial derivatives
    
    // dR/droll
    dR_droll << 
        0, -cp*sy*sr - sp*cy, -cp*sy*cr + sp*sy,
        0,  cp*cy*sr - sp*sy,  cp*cy*cr + sp*cy,
        0, -cp*sr,            -cp*cr;
    
    // dR/dpitch
    dR_dpitch << 
        -sp*cy, cp*sy*cr - sp*cy*sr, -cp*sy*sr - sp*cy*cr,
        -sp*sy, -cp*cy*cr - sp*sy*sr, cp*cy*sr - sp*sy*cr,
        -cp,    sp*sr,                sp*cr;
    
    // dR/dyaw
    dR_dyaw << 
        -sy*cp, -sy*sp*cr - cy*sr, -sy*sp*sr + cy*cr,
        cy*cp,  cy*sp*cr - sy*sr,  cy*sp*sr + sy*cr,
        0,      0,                 0;
}

// Analytical Jacobian for predictCorner
// This replaces the autodiff version for speed
Eigen::Vector2d predictCornerAnalytical(
    const Eigen::VectorXd& x,
    Eigen::MatrixXd& J,
    const SystemSLAM& system,
    const Camera& camera,
    std::size_t idxLandmark,
    int cornerIdx,
    double markerSize)
{
    const std::size_t nx = x.size();
    
    // Extract camera pose from state
    Eigen::Vector3d Thetanb = x.segment<3>(9);  // Body orientation (RPY)
    Eigen::Vector3d rBNn = x.segment<3>(6);     // Body position
    
    // Compute camera pose
    Eigen::Matrix3d Rnb = rpy2rot(Thetanb);
    Pose<double> Tnb;
    Tnb.rotationMatrix = Rnb;
    Tnb.translationVector = rBNn;
    Pose<double> Tnc = camera.bodyToCamera(Tnb);
    Eigen::Matrix3d Rnc = Tnc.rotationMatrix;
    Eigen::Vector3d rCNn = Tnc.translationVector;
    
    // Extract landmark pose from state
    std::size_t idx = system.landmarkPositionIndex(idxLandmark);
    Eigen::Vector3d rPNn = x.segment<3>(idx);      // Landmark position
    Eigen::Vector3d Theta_j = x.segment<3>(idx+3); // Landmark orientation (RPY)
    Eigen::Matrix3d Rnj = rpy2rot(Theta_j);
    
    // Corner position in marker frame
    double half = markerSize / 2.0;
    Eigen::Vector3d rjc;
    if (cornerIdx == 0)      rjc << -half,  half, 0.0;  // Top-left
    else if (cornerIdx == 1) rjc <<  half,  half, 0.0;  // Top-right
    else if (cornerIdx == 2) rjc <<  half, -half, 0.0;  // Bottom-right
    else                     rjc << -half, -half, 0.0;  // Bottom-left
    
    // Transform corner to world frame: r^n_{jc/N} = R^n_j * r^j_{jc/j} + r^n_{j/N}
    Eigen::Vector3d rCornerNn = Rnj * rjc + rPNn;
    
    // Transform to camera frame: r^c_{jc/C} = (R^n_c)^T * (r^n_{jc/N} - r^n_{C/N})
    Eigen::Vector3d rCornerCc = Rnc.transpose() * (rCornerNn - rCNn);
    
    // Project to pixel coordinates
    Eigen::Matrix23d J_pixel_rCc;  // 2x3 Jacobian of pixel projection
    Eigen::Vector2d pixel = camera.vectorToPixel(rCornerCc, J_pixel_rCc);
    
    // Now compute full Jacobian using chain rule
    J.resize(2, nx);
    J.setZero();
    
    // Chain rule: ∂pixel/∂x = ∂pixel/∂rCc * ∂rCc/∂x
    
    // 1. Jacobian w.r.t. camera position (x[6:8])
    // ∂rCc/∂rCNn = -(R^n_c)^T
    Eigen::Matrix3d J_rCc_rCNn = -Rnc.transpose();
    
    // But rCNn depends on rBNn and Rnb through camera extrinsics
    // rCNn = Rnb * rCBb + rBNn
    // So ∂rCNn/∂rBNn = I
    J.block<2,3>(0, 6) = J_pixel_rCc * J_rCc_rCNn;  // Direct contribution from body position
    
    // 2. Jacobian w.r.t. camera orientation (x[9:11])  
    // rCc = (R^n_c)^T * (rCornerNn - rCNn)
    // Need ∂rCc/∂Thetanb through ∂R^n_c/∂Thetanb
    
    //  This is complex - R^n_c depends on Thetanb through the camera extrinsics
    // For now, compute numerically or use finite differences for these terms
    // TODO: Implement full analytical derivative of camera orientation
    
    // 3. Jacobian w.r.t. landmark position (x[idx:idx+2])
    // ∂rCc/∂rPNn = (R^n_c)^T
    Eigen::Matrix3d J_rCc_rPNn = Rnc.transpose();
    J.block<2,3>(0, idx) = J_pixel_rCc * J_rCc_rPNn;
    
    // 4. Jacobian w.r.t. landmark orientation (x[idx+3:idx+5])
    // rCornerNn = R^n_j * rjc + rPNn
    // ∂rCornerNn/∂Theta_j requires ∂R^n_j/∂Theta_j
    Eigen::Matrix3d dRnj_droll, dRnj_dpitch, dRnj_dyaw;
    computeRPYJacobian(Theta_j, dRnj_droll, dRnj_dpitch, dRnj_dyaw);
    
    Eigen::Vector3d drCornerNn_droll = dRnj_droll * rjc;
    Eigen::Vector3d drCornerNn_dpitch = dRnj_dpitch * rjc;
    Eigen::Vector3d drCornerNn_dyaw = dRnj_dyaw * rjc;
    
    Eigen::Matrix3d J_rCornerNn_Theta;
    J_rCornerNn_Theta.col(0) = drCornerNn_droll;
    J_rCornerNn_Theta.col(1) = drCornerNn_dpitch;
    J_rCornerNn_Theta.col(2) = drCornerNn_dyaw;
    
    // Chain through to camera frame
    Eigen::Matrix3d J_rCc_Theta = Rnc.transpose() * J_rCornerNn_Theta;
    J.block<2,3>(0, idx+3) = J_pixel_rCc * J_rCc_Theta;
    
    return pixel;
}
