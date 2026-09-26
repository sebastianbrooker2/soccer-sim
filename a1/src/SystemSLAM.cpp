#include <cstddef>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "GaussianInfo.hpp"
#include "SystemEstimator.h"
#include "SystemSLAM.h"

SystemSLAM::SystemSLAM(const GaussianInfo<double> & density)
    : SystemEstimator(density)
{}

// Evaluate f(x) from the SDE dx = f(x)*dt + dw
Eigen::VectorXd SystemSLAM::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u) const
{
    assert(density.dim() == x.size());
    //
    //  dnu/dt =          0 + dwnu/dt
    // deta/dt = JK(eta)*nu +       0
    //   dm/dt =          0 +       0
    // \_____/   \________/   \_____/
    //  dx/dt  =    f(x)    +  dw/dt
    //
    //        [          0 ]
    // f(x) = [ JK(eta)*nu ]
    //        [          0 ] for all map states
    //
    //        [                    0 ]
    //        [                    0 ]
    // f(x) = [    Rnb(thetanb)*vBNb ]
    //        [ TK(thetanb)*omegaBNb ]
    //        [                    0 ] for all map states
    //
    Eigen::VectorXd f(x.size());
    f.setZero();
    // TODO: Implement in Assignment(s)
    // Extract velocity and pose from state (first 12 states are camera states)
    Eigen::Vector3d vBNb = x.segment<3>(0);       // Body velocity (indices 0-2)
    Eigen::Vector3d omegaBNb = x.segment<3>(3);   // Body angular velocity (indices 3-5)
    Eigen::Vector3d rBNn = x.segment<3>(6);       // Position (indices 6-8)
    double phi = x(9);                             // Roll
    double theta = x(10);                          // Pitch
    double psi = x(11);                            // Yaw
    
    // Compute rotation matrix Rz*Ry*Rx (ZYX Euler)
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
    
    // Landmarks (12+): dm/dt = 0 (static environment)
    // Already zero from f.setZero()

    return f;
}

// Evaluate f(x) and its Jacobian J = df/fx from the SDE dx = f(x)*dt + dw
Eigen::VectorXd SystemSLAM::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd f = dynamics(t, x, u);

    // Initialize Jacobian to zero
    J.resize(x.size(), x.size());
    J.setZero();

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
                cy*cp,  cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
                0,      0,                 0;

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

    // Fill Jacobian
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

    return f;
}

Eigen::VectorXd SystemSLAM::input(double t, const Eigen::VectorXd & x) const
{
    return Eigen::VectorXd(0);
}

GaussianInfo<double> SystemSLAM::processNoiseDensity(double dt) const
{
    // Process noise: acts only on velocity states (first 6)
    const int nx = 6; // velocity state dimension
    const double sigma = std::sqrt(1e-3 * dt);  // Reduced to slow covariance growth

    // Diagonal matrix with sigma on the diagonal
    Eigen::MatrixXd SQ = sigma * Eigen::MatrixXd::Identity(nx, nx);

    return GaussianInfo<double>::fromSqrtMoment(SQ);
}

std::vector<Eigen::Index> SystemSLAM::processNoiseIndex() const
{
    // Indices of process model equations where process noise is injected
    // Process noise only affects velocity states (indices 0-5)
    std::vector<Eigen::Index> idxQ = {0, 1, 2, 3, 4, 5};
    return idxQ;
}

cv::Mat & SystemSLAM::view()
{
    return view_;
};

const cv::Mat & SystemSLAM::view() const
{
    return view_;
};

GaussianInfo<double> SystemSLAM::bodyPositionDensity() const
{
    return density.marginal(Eigen::seqN(6, 3));
}

GaussianInfo<double> SystemSLAM::bodyOrientationDensity() const
{
    return density.marginal(Eigen::seqN(9, 3));
}

GaussianInfo<double> SystemSLAM::bodyTranslationalVelocityDensity() const
{
    return density.marginal(Eigen::seqN(0, 3));
}

GaussianInfo<double> SystemSLAM::bodyAngularVelocityDensity() const
{
    return density.marginal(Eigen::seqN(3, 3));
}

#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

Eigen::Vector3d SystemSLAM::cameraPosition(const Camera & camera, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> rCNn_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraPosition<autodiff::dual>, wrt(x_dual), at(camera, x_dual), rCNn_dual);
    return rCNn_dual.cast<double>();
};

GaussianInfo<double> SystemSLAM::cameraPositionDensity(const Camera & camera) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return cameraPosition(camera, x, J); };
    return density.affineTransform(f);
}

Eigen::Vector3d SystemSLAM::cameraOrientationEuler(const Camera & camera, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> Thetanc_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraOrientationEuler<autodiff::dual>, wrt(x_dual), at(camera, x_dual), Thetanc_dual);
    return Thetanc_dual.cast<double>();
};

GaussianInfo<double> SystemSLAM::cameraOrientationEulerDensity(const Camera & camera) const
{
    auto f = [&](const Eigen::VectorXd & x, Eigen::MatrixXd & J) { return cameraOrientationEuler(camera, x, J); };
    return density.affineTransform(f);    
}

GaussianInfo<double> SystemSLAM::landmarkPositionDensity(std::size_t idxLandmark) const
{
    assert(idxLandmark < numberLandmarks());
    std::size_t idx = landmarkPositionIndex(idxLandmark);
    return density.marginal(Eigen::seqN(idx, 3));
}
