#include <cstddef>
#include <cmath>
#include <vector>
#include <print>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "GaussianInfo.hpp"
#include "SystemEstimator.h"
#include "SystemVisualNav.h"

SystemVisualNav::SystemVisualNav(const GaussianInfo<double> & density)
    : SystemEstimator(density)
    , lastEventWasFlow_(true)  // CRITICAL: Initialize to true so first flow event clones η→ζ
{

}

SystemVisualNav * SystemVisualNav::clone() const
{
    return new SystemVisualNav(*this);
}

void SystemVisualNav::predict(double time)
{
    double dt = time - time_;
    assert(dt >= 0);
    if (dt == 0.0) return;

    // DIAGNOSTIC: Check if predict is being called
    static int predict_call_count = 0;
    if (predict_call_count % 10 == 0) {
        std::println("  [PREDICT] dt={:.6f}s, lastEventWasFlow_={}, dim={}", 
                     dt, lastEventWasFlow_, density.dim());
    }
    predict_call_count++;

    // Augment state density with independent noise increment dw ~ N^{-1}(0, LambdaQ/dt)
    // [ x] ~ N^{-1}([ eta ], [ Lambda,          0 ])
    // [dw]         ([   0 ]  [      0, LambdaQ/dt ])

    auto pdw = processNoiseDensity(dt); // p(dw(idxQ)[k])
    auto pxdw = density*pdw;            // p(x[k], dw(idxQ)[k]) = p(x[k])*p(dw(idxQ)[k])

    // Phi maps [ x[k]; dw(idxQ)[k] ] to x[k+1]
    auto Phi = [&](const Eigen::VectorXd & xdw, Eigen::MatrixXd & J)
    {
        // Integrate first 12 states (nu, eta) using RK4
        Eigen::VectorXd xk1 = RK4SDEHelper(xdw, dt, J);
        
        // Handle delayed pose states (12-17): stochastic cloning per slide 13
        // ζ[k+1] = ξ[k] where ξ[k] = { η[k] if last event was flow
        //                              ζ[k] if last event was other }
        
        // DEBUG: Print stochastic cloning decision
        static int clone_call = 0;
        if (clone_call % 10 == 0) {
            std::println("  [STOCHASTIC CLONING] lastEventWasFlow_={}, eta_yaw={:.3f}, zeta_yaw={:.3f}",
                       lastEventWasFlow_, xdw(11), xdw(17));
        }
        clone_call++;
        
        if (lastEventWasFlow_)
        {
            // Last event was flow → update delayed pose to current pose
            xk1.segment<6>(12) = xdw.segment<6>(6);  // ζ[k+1] = η[k]
        }
        else
        {
            // Last event was NOT flow → hold delayed pose unchanged
            xk1.segment<6>(12) = xdw.segment<6>(12); // ζ[k+1] = ζ[k]
        }
        
        // Map states (18+) remain unchanged (already handled by RK4SDEHelper returning zeros for their dynamics)
        
        return xk1;
    };
    
    // Map p(x[k], dw(idxQ)[k]) to p(x[k+1])
    density = pxdw.affineTransform(Phi);

    time_ = time;
}

// Evaluate f(x) from the SDE dx = f(x)*dt + dw
Eigen::VectorXd SystemVisualNav::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u) const
{
    assert(density.dim() == x.size());

    Eigen::VectorXd f(x.size());
    f.setZero();
    // TODO: Implement in Assignment(s)
    // State structure: x = [nu(6), eta(6), zeta(6), map(3*N)]
    // where nu = [vBNb, omegaBNb], eta = [rBNn, Thetanb], zeta = delayed eta from previous optical flow frame
    
    // Extract velocity and pose from state (same as SystemSLAM for first 12 states)
    Eigen::Vector3d vBNb = x.segment<3>(0);       // Body velocity (indices 0-2)
    Eigen::Vector3d omegaBNb = x.segment<3>(3);   // Body angular velocity (indices 3-5)
    double phi = x(9);                             // Roll
    double theta = x(10);                          // Pitch
    double psi = x(11);                            // Yaw
    
    // Compute rotation matrix Rz*Ry*Rx (ZYX Euler) - same as A1
    double sr = std::sin(phi),   cr = std::cos(phi);
    double sp = std::sin(theta), cp = std::cos(theta);
    double sy = std::sin(psi),   cy = std::cos(psi);

    Eigen::Matrix3d Rnb;
    Rnb << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
           sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
           -sp,   cp*sr,            cp*cr;

    // Angular mapping T(phi,theta) - transforms body rates to Euler angle rates
    double tp = std::tan(theta);
    Eigen::Matrix3d T;
    T << 1, sr*tp, cr*tp,
         0, cr,   -sr,
         0, sr/cp, cr/cp;
    
    // f(x) components:
    // Velocities (0-5): dnu/dt = 0 (change only due to process noise)
    f.segment<3>(0).setZero();
    f.segment<3>(3).setZero();
    
    // Position rate (6-8): drBNn/dt = Rnb * vBNb
    f.segment<3>(6) = Rnb * vBNb;
    
    // Orientation rate (9-11): d[rpy]/dt = T * omegaBNb
    f.segment<3>(9) = T * omegaBNb;
    
    // Delayed pose (12-17): dzeta/dt = 0 (updated discretely in predict())
    // f.segment<6>(12) already zero
    
    // Landmarks (18+): dm/dt = 0 (static environment)
    // Already zero from f.setZero()

    return f;
}

// Evaluate f(x) and its Jacobian J = df/fx from the SDE dx = f(x)*dt + dw
Eigen::VectorXd SystemVisualNav::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd f = dynamics(t, x, u);

    // Jacobian J = df/dx
    J.resize(f.size(), x.size());
    J.setZero();
    // TODO: Implement in Assignment(s)
    // Same structure as SystemSLAM from A1, but extended for delayed pose states
    
    // Extract velocity and pose from state
    Eigen::Vector3d vBNb = x.segment<3>(0);
    Eigen::Vector3d omegaBNb = x.segment<3>(3);
    double phi = x(9);
    double theta = x(10);
    double psi = x(11);

    // Precompute trig functions
    double sr = std::sin(phi),   cr = std::cos(phi);
    double sp = std::sin(theta), cp = std::cos(theta);
    double sy = std::sin(psi),   cy = std::cos(psi);
    double tp = std::tan(theta);
    double sec2 = 1.0/(cp*cp); // sec^2(theta)

    // Rotation matrix
    Eigen::Matrix3d Rnb;
    Rnb << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
           sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
           -sp,   cp*sr,            cp*cr;

    // Derivatives of rotation matrix
    Eigen::Matrix3d dR_dphi, dR_dtheta, dR_dpsi;
    dR_dphi << 0, cy*sp*cr + sy*sr, -cy*sp*sr + sy*cr,
               0, sy*sp*cr - cy*sr, -sy*sp*sr - cy*cr,
               0, cp*cr,            -cp*sr;
    dR_dtheta << -cy*sp, cy*cp*sr, cy*cp*cr,
                 -sy*sp, sy*cp*sr, sy*cp*cr,
                 -cp,    -sp*sr,   -sp*cr;
    dR_dpsi << -sy*cp, -sy*sp*sr - cy*cr, -sy*sp*cr + cy*sr,
                cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
                0,      0,                  0;

    // Angular mapping T
    Eigen::Matrix3d T;
    T << 1, sr*tp, cr*tp,
         0, cr,   -sr,
         0, sr/cp, cr/cp;

    // Derivatives of T matrix
    Eigen::Matrix3d dT_dphi;
    dT_dphi << 0, cr*tp, -sr*tp,
               0, -sr,   -cr,
               0, cr/cp, -sr/cp;

    Eigen::Matrix3d dT_dtheta;
    dT_dtheta << 0, sr*sec2, cr*sec2,
                  0, 0, 0,
                  0, sr*tp*sec2, cr*tp*sec2;

    // Fill Jacobian (first 12x12 block same as SystemSLAM)
    // df_r/dv = R
    J.block<3,3>(6,0) = Rnb;

    // df_r/d(rpy)
    J.block<3,1>(6,9)  = dR_dphi   * vBNb;
    J.block<3,1>(6,10) = dR_dtheta * vBNb;
    J.block<3,1>(6,11) = dR_dpsi   * vBNb;

    // df_rpy/dω = T
    J.block<3,3>(9,3) = T;

    // df_rpy/d(rpy)
    J.block<3,1>(9,9)  = dT_dphi   * omegaBNb;
    J.block<3,1>(9,10) = dT_dtheta * omegaBNb;
    // yaw has no effect on T, so column 11 remains zero
    
    // Delayed pose and landmarks have zero dynamics, so their Jacobian blocks remain zero

    return f;
}

Eigen::VectorXd SystemVisualNav::input(double t, const Eigen::VectorXd & x) const
{
    return Eigen::VectorXd(0);
}

GaussianInfo<double> SystemVisualNav::processNoiseDensity(double dt) const
{
    // SQ is an upper triangular matrix such that SQ.'*SQ = Q is the power spectral density of the continuous time process noise
    Eigen::MatrixXd SQ;
    
    // TODO: Assignment(s)
    // Process noise: acts only on velocity states (first 6), same as SystemSLAM from A1
    const int nx = 6; // velocity state dimension
    
    // ═══════════════════════════════════════════════════════════════
    // SCENARIO-SPECIFIC PROCESS NOISE TUNING
    // ═══════════════════════════════════════════════════════════════
    // Power spectral density Q = SQ^T * SQ (units: [state_units]^2 per second)
    // These are the BASE sigma values (not time-dependent)
    // Ordered by state index: [vN, vE, vD, ωx, ωy, ωz]
    
    double sigma_vN, sigma_vE, sigma_vD, sigma_wx, sigma_wy, sigma_wz;
    
    if (scenario_ == 4) {
        // SCENARIO 4: OUTDOOR FLIGHT
        // More process noise tolerance for natural outdoor motion
        sigma_vN = 2.0;     // North velocity noise [m/s/√s]
        sigma_vE = 2.0;     // East velocity 
        sigma_vD = 2.0;     // Down velocity 
        sigma_wx = 0.05;    // Roll rate 
        sigma_wy = 0.5;     // Pitch rate 
        sigma_wz = 0.3;     // Yaw rate 
    }
    else if (scenario_ == 5 || scenario_ == 6) {
        // SCENARIOS 5 INDOOR NAVIGATION
        // Tighter process noise for controlled indoor handheld motion
        sigma_vN = 0.3;     // North velocity noise 
        sigma_vE = 0.2;     // East velocity noise 
        sigma_vD = 0.01;    // Down velocity noise 
        sigma_wx = 0.000001;  // Roll rate noise 
        sigma_wy = 0.000001;  // Pitch rate
        sigma_wz = 0.01;    // Yaw rate noise [rad/s/√s]
    }
    else {
        // DEFAULT: Use indoor values as fallback (scenario 6)
        sigma_vN = 3.0;
        sigma_vE = 1.0;
        sigma_vD = 0.01;
        sigma_wx = 0.000001;
        sigma_wy = 0.000001;
        sigma_wz = 0.01;
    }
    
    // ═══════════════════════════════════════════════════════════════
    
    // Compute time-scaled process noise standard deviations
    // For continuous-time noise: σ_effective = σ_base * sqrt(dt)
    const double sigma_vN_scaled = sigma_vN * std::sqrt(dt);
    const double sigma_vE_scaled = sigma_vE * std::sqrt(dt);
    const double sigma_vD_scaled = sigma_vD * std::sqrt(dt);
    const double sigma_wx_scaled = sigma_wx * std::sqrt(dt);
    const double sigma_wy_scaled = sigma_wy * std::sqrt(dt);
    const double sigma_wz_scaled = sigma_wz * std::sqrt(dt);

    // Diagonal matrix with scaled sigma values
    SQ = Eigen::MatrixXd::Zero(nx, nx);
    SQ(0, 0) = sigma_vN_scaled;  // vN - North velocity
    SQ(1, 1) = sigma_vE_scaled;  // vE - East velocity
    SQ(2, 2) = sigma_vD_scaled;  // vD - Down velocity (altitude rate)
    SQ(3, 3) = sigma_wx_scaled;  // ωx - Roll rate
    SQ(4, 4) = sigma_wy_scaled;  // ωy - Pitch rate
    SQ(5, 5) = sigma_wz_scaled;  // ωz - Yaw rate

    // Distribution of noise increment dw ~ N(0, Q*dt) for time increment dt
    return GaussianInfo<double>::fromSqrtMoment(SQ*std::sqrt(dt));
}

std::vector<Eigen::Index> SystemVisualNav::processNoiseIndex() const
{
    // Indices of process model equations where process noise is injected
    std::vector<Eigen::Index> idxQ;
    // TODO: Assignment(s)
    // Process noise only affects velocity states (indices 0-5), same as SystemSLAM
    idxQ = {0, 1, 2, 3, 4, 5};
    return idxQ;
}

cv::Mat & SystemVisualNav::view()
{
    return view_;
};

const cv::Mat & SystemVisualNav::view() const
{
    return view_;
};

std::size_t SystemVisualNav::numberLandmarks() const
{
    // For Scenario 5: 6-DOF pose landmarks (position + orientation)
    // For Scenario 6: 3-DOF point landmarks
    // Check if using pose landmarks by checking if state dim is compatible
    // State: 18 (body) + 6*N (pose landmarks) or 18 + 3*N (point landmarks)
    
    std::size_t remaining = density.dim() - 18;
    
    // Try 6-DOF first (Scenario 5)
    if (remaining % 6 == 0) {
        return remaining / 6;
    }
    // Fall back to 3-DOF (Scenario 6)
    else if (remaining % 3 == 0) {
        return remaining / 3;
    }
    else {
        throw std::runtime_error("Invalid state dimension for landmarks");
    }
}

std::size_t SystemVisualNav::landmarkPositionIndex(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    
    // Check landmark type
    std::size_t remaining = density.dim() - 18;
    
    if (remaining % 6 == 0) {
        // 6-DOF pose landmarks (Scenario 5)
        return 18 + 6*idxLandmark;  
    } else {
        // 3-DOF point landmarks (Scenario 6)
        return 18 + 3*idxLandmark;
    }
}

std::size_t SystemVisualNav::landmarkOrientationIndex(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    
    std::size_t remaining = density.dim() - 18;
    
    if (remaining % 6 == 0) {
        // 6-DOF pose landmarks - orientation follows position
        return 18 + 6*idxLandmark + 3;
    } else {
        throw std::runtime_error("Orientation index only valid for pose landmarks");
    }
}

bool SystemVisualNav::hasPoseLandmarks() const
{
    std::size_t remaining = density.dim() - 18;
    return (remaining % 6 == 0);
}

GaussianInfo<double> SystemVisualNav::bodyPositionDensity() const
{
    return density.marginal(Eigen::seqN(6, 3));
}

GaussianInfo<double> SystemVisualNav::bodyOrientationDensity() const
{
    return density.marginal(Eigen::seqN(9, 3));
}

GaussianInfo<double> SystemVisualNav::bodyTranslationalVelocityDensity() const
{
    return density.marginal(Eigen::seqN(0, 3));
}

GaussianInfo<double> SystemVisualNav::bodyAngularVelocityDensity() const
{
    return density.marginal(Eigen::seqN(3, 3));
}

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

Eigen::Vector3d SystemVisualNav::cameraPosition(const Camera & camera, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> rCNn_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraPosition<autodiff::dual>, wrt(x_dual), at(camera, x_dual), rCNn_dual);
    return rCNn_dual.cast<double>();
};

GaussianInfo<double> SystemVisualNav::cameraPositionDensity(const Camera & camera) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return cameraPosition(camera, x, J); };
    return density.affineTransform(f);
}

Eigen::Vector3d SystemVisualNav::cameraOrientationEuler(const Camera & camera, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> Thetanc_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraOrientationEuler<autodiff::dual>, wrt(x_dual), at(camera, x_dual), Thetanc_dual);
    return Thetanc_dual.cast<double>();
};

GaussianInfo<double> SystemVisualNav::cameraOrientationEulerDensity(const Camera & camera) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return cameraOrientationEuler(camera, x, J); };
    return density.affineTransform(f);    
}

GaussianInfo<double> SystemVisualNav::landmarkPositionDensity(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    std::size_t idx = landmarkPositionIndex(idxLandmark);
    return density.marginal(Eigen::seqN(idx, 3));
}
