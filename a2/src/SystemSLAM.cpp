#include <cstddef>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include "GaussianInfo.hpp"
#include "SystemEstimator.h"
#include "SystemSLAM.h"

SystemSLAM::SystemSLAM(const GaussianInfo<double> & density)
    : SystemEstimator(density)
{}

Eigen::VectorXd SystemSLAM::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u) const
{
    assert(density.dim() == x.size());
    Eigen::VectorXd f(x.size());
    f.setZero();
    
    Eigen::Vector3d vBNb = x.segment<3>(0);
    Eigen::Vector3d omegaBNb = x.segment<3>(3);
    double phi = x(9), theta = x(10), psi = x(11);
    
    double sr = std::sin(phi),   cr = std::cos(phi);
    double sp = std::sin(theta), cp = std::cos(theta);
    double sy = std::sin(psi),   cy = std::cos(psi);

    Eigen::Matrix3d Rnb;
    Rnb << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
           sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
           -sp,   cp*sr,            cp*cr;

    double tp = std::tan(theta);
    Eigen::Matrix3d T;
    T << 1, sr*tp, cr*tp,
         0, cr,   -sr,
         0, sr/cp, cr/cp;
    
    f.segment<3>(6) = Rnb * vBNb;
    f.segment<3>(9) = T * omegaBNb;

    return f;
}

Eigen::VectorXd SystemSLAM::dynamics(double t, const Eigen::VectorXd & x, const Eigen::VectorXd & u, Eigen::MatrixXd & J) const
{
    Eigen::VectorXd f = dynamics(t, x, u);
    J.resize(x.size(), x.size());
    J.setZero();

    Eigen::Vector3d vBNb = x.segment<3>(0);
    Eigen::Vector3d omegaBNb = x.segment<3>(3);
    double phi = x(9), theta = x(10), psi = x(11);

    double sr = std::sin(phi),   cr = std::cos(phi);
    double sp = std::sin(theta), cp = std::cos(theta);
    double sy = std::sin(psi),   cy = std::cos(psi);
    double tp = std::tan(theta);
    double sec2 = 1.0/(cp*cp);

    Eigen::Matrix3d Rnb;
    Rnb << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
           sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
           -sp,   cp*sr,            cp*cr;

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

    Eigen::Matrix3d T;
    T << 1, sr*tp, cr*tp,
         0, cr,   -sr,
         0, sr/cp, cr/cp;

    Eigen::Matrix3d dT_dphi;
    dT_dphi << 0, cr*tp, -sr*tp,
               0, -sr,   -cr,
               0, cr/cp, -sr/cp;

    Eigen::Matrix3d dT_dtheta;
    dT_dtheta << 0, sr*sec2, cr*sec2,
                  0, 0, 0,
                  0, sr*tp*sec2, cr*tp*sec2;

    J.block<3,3>(6,0) = Rnb;
    J.block<3,1>(6,9)  = dR_dphi   * vBNb;
    J.block<3,1>(6,10) = dR_dtheta * vBNb;
    J.block<3,1>(6,11) = dR_dpsi   * vBNb;
    J.block<3,3>(9,3) = T;
    J.block<3,1>(9,9)  = dT_dphi   * omegaBNb;
    J.block<3,1>(9,10) = dT_dtheta * omegaBNb;

    return f;
}

Eigen::VectorXd SystemSLAM::input(double t, const Eigen::VectorXd & x) const
{
    return Eigen::VectorXd(0);
}

GaussianInfo<double> SystemSLAM::processNoiseDensity(double dt) const
{
    const int nx = 6;
    const double sigma = std::sqrt(1e-3 * dt);
    Eigen::MatrixXd SQ = sigma * Eigen::MatrixXd::Identity(nx, nx);
    return GaussianInfo<double>::fromSqrtMoment(SQ);
}

std::vector<Eigen::Index> SystemSLAM::processNoiseIndex() const
{
    return {0, 1, 2, 3, 4, 5};
}

cv::Mat & SystemSLAM::view()
{
    return view_;
}

const cv::Mat & SystemSLAM::view() const
{
    return view_;
}

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

Eigen::Vector3d SystemSLAM::cameraPosition(const Camera & camera, const Eigen::VectorXd & x, Eigen::MatrixXd & J)
{
    Eigen::Vector3<autodiff::dual> rCNn_dual;
    Eigen::VectorX<autodiff::dual> x_dual = x.cast<autodiff::dual>();
    J = jacobian(cameraPosition<autodiff::dual>, wrt(x_dual), at(camera, x_dual), rCNn_dual);
    return rCNn_dual.cast<double>();
}

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
}

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
