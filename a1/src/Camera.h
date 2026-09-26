#ifndef CAMERA_H
#define CAMERA_H

#include <vector>
#include <filesystem>
#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include "serialisation.hpp"
#include "Pose.hpp"

struct Chessboard
{
    cv::Size boardSize;
    float squareSize;

    void write(cv::FileStorage & fs) const;                 // OpenCV serialisation
    void read(const cv::FileNode & node);                   // OpenCV serialisation

    std::vector<cv::Point3f> gridPoints() const;
    friend std::ostream & operator<<(std::ostream &, const Chessboard &);
};

struct Camera;

struct ChessboardImage
{
    ChessboardImage(const cv::Mat &, const Chessboard &, const std::filesystem::path & = "");
    cv::Mat image;
    std::filesystem::path filename;
    Pose<double> Tnc;                                               // Extrinsic camera parameters
    std::vector<cv::Point2f> corners;                       // Chessboard corners in image [rQOi]
    bool isFound;
    void drawCorners(const Chessboard &);
    void drawBox(const Chessboard &, const Camera &);
    void recoverPose(const Chessboard &, const Camera &);
};

struct ChessboardData
{
    explicit ChessboardData(const std::filesystem::path &); // Load from config file

    Chessboard chessboard;
    std::vector<ChessboardImage> chessboardImages;

    void drawCorners();
    void drawBoxes(const Camera &);
    void recoverPoses(const Camera &);
};

namespace Eigen {
using Matrix23d = Eigen::Matrix<double, 2, 3>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

struct Camera
{
    void calibrate(ChessboardData &);                       // Calibrate camera from chessboard data
    void printCalibration() const;

    template <typename Scalar> Pose<Scalar> cameraToBody(const Pose<Scalar> & Tnc) const { return Tnc*Tbc.inverse(); } // Tnb = Tnc*Tcb
    template <typename Scalar> Pose<Scalar> bodyToCamera(const Pose<Scalar> & Tnb) const { return Tnb*Tbc; } // Tnc = Tnb*Tbc
    cv::Vec3d worldToVector(const cv::Vec3d & rPNn, const Pose<double> & Tnb) const;
    cv::Vec2d worldToPixel(const cv::Vec3d &, const Pose<double> &) const;
    cv::Vec2d vectorToPixel(const cv::Vec3d &) const;
    template <typename Scalar> Eigen::Vector2<Scalar> vectorToPixel(const Eigen::Vector3<Scalar> &) const;
    Eigen::Vector2d vectorToPixel(const Eigen::Vector3d &, Eigen::Matrix23d &) const;

    cv::Vec3d pixelToVector(const cv::Vec2d &) const;

    bool isWorldWithinFOV(const cv::Vec3d & rPNn, const Pose<double> & Tnb) const;
    bool isVectorWithinFOV(const cv::Vec3d & rPCc) const;

    void calcFieldOfView();
    void write(cv::FileStorage &) const;                    // OpenCV serialisation
    void read(const cv::FileNode &);                        // OpenCV serialisation

    cv::Mat cameraMatrix;                                   // Camera matrix
    cv::Mat distCoeffs;                                     // Lens distortion coefficients
    int flags = 0;                                          // Calibration flags
    cv::Size imageSize;                                     // Image size

    Pose<double> Tbc;                                       // Relative pose of camera in body coordinates (Rbc, rCBb)

private:
    double hFOV = 0.0;                                      // Horizonal field of view
    double vFOV = 0.0;                                      // Vertical field of view
    double dFOV = 0.0;                                      // Diagonal field of view
};

#include <cmath>
#include <opencv2/calib3d.hpp>

template <typename Scalar>
Eigen::Vector2<Scalar> Camera::vectorToPixel(const Eigen::Vector3<Scalar> & rPCc) const
{
    bool isRationalModel    = (flags & cv::CALIB_RATIONAL_MODEL) == cv::CALIB_RATIONAL_MODEL;
    bool isThinPrismModel   = (flags & cv::CALIB_THIN_PRISM_MODEL) == cv::CALIB_THIN_PRISM_MODEL;
    assert(isRationalModel && isThinPrismModel);

    Eigen::Vector2<Scalar> rQOi;
    // TODO: Lab 8 (optional)
    // Implement the conversion from vector to pixel coordinates using the camera matrix and distortion coefficients
    Scalar x = rPCc(0);
    Scalar y = rPCc(1);
    Scalar z = rPCc(2);
    Scalar v = y / z;
    Scalar u = x / z;
    Scalar r2 = u*u + v*v;

    // Intrinsics
    const Scalar fx = static_cast<Scalar>(cameraMatrix.at<double>(0,0));
    const Scalar fy = static_cast<Scalar>(cameraMatrix.at<double>(1,1));
    const Scalar cx = static_cast<Scalar>(cameraMatrix.at<double>(0,2));
    const Scalar cy = static_cast<Scalar>(cameraMatrix.at<double>(1,2));

    // Distortion coeffs (OpenCV order): [k1 k2 p1 p2 k3 k4 k5 k6 s1 s2 s3 s4]
    const Scalar k1 = static_cast<Scalar>(distCoeffs.at<double>(0));
    const Scalar k2 = static_cast<Scalar>(distCoeffs.at<double>(1));
    const Scalar p1 = static_cast<Scalar>(distCoeffs.at<double>(2));
    const Scalar p2 = static_cast<Scalar>(distCoeffs.at<double>(3));
    const Scalar k3 = static_cast<Scalar>(distCoeffs.at<double>(4));
    const Scalar k4 = static_cast<Scalar>(distCoeffs.at<double>(5));
    const Scalar k5 = static_cast<Scalar>(distCoeffs.at<double>(6));
    const Scalar k6 = static_cast<Scalar>(distCoeffs.at<double>(7));
    const Scalar s1 = static_cast<Scalar>(distCoeffs.at<double>(8));
    const Scalar s2 = static_cast<Scalar>(distCoeffs.at<double>(9));
    const Scalar s3 = static_cast<Scalar>(distCoeffs.at<double>(10));
    const Scalar s4 = static_cast<Scalar>(distCoeffs.at<double>(11));

    // radial distortion
    const Scalar alpha = k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
    const Scalar beta  = k4*r2 + k5*r2*r2 + k6*r2*r2*r2;
    const Scalar c = (Scalar(1) + alpha) / (Scalar(1) + beta);
    const Scalar du_rad = c * u;
    const Scalar dv_rad = c * v;

    // decentering distortion
    const Scalar du_dec = Scalar(2)*p1*u*v + p2*(r2 + Scalar(2)*u*u);
    const Scalar dv_dec = p1*(r2 + Scalar(2)*v*v) + Scalar(2)*p2*u*v;

    // thin prism distortion
    const Scalar du_thin = s1*r2 + s2*r2*r2;
    const Scalar dv_thin = s3*r2 + s4*r2*r2;

    const Scalar du = du_rad + du_dec + du_thin;
    const Scalar dv = dv_rad + dv_dec + dv_thin;

    rQOi << fx*du + cx, fy*dv + cy;

    return rQOi;
}

#endif

