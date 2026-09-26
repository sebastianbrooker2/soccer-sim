#include <cassert>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <format>
#include <vector>
#include <filesystem>
#include <regex>
#include <print>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include "to_string.hpp"
#include "rotation.hpp"
#include "Pose.hpp"
#include "Camera.h"

void Chessboard::write(cv::FileStorage & fs) const
{
    fs << "{"
       << "grid_width"  << boardSize.width
       << "grid_height" << boardSize.height
       << "square_size" << squareSize
       << "}";
}

void Chessboard::read(const cv::FileNode & node)
{
    node["grid_width"]  >> boardSize.width;
    node["grid_height"] >> boardSize.height;
    node["square_size"] >> squareSize;
}

std::vector<cv::Point3f> Chessboard::gridPoints() const
{
    std::vector<cv::Point3f> rPNn_all;
    rPNn_all.reserve(boardSize.height*boardSize.width);
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            rPNn_all.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));   
    return rPNn_all; 
}

std::ostream & operator<<(std::ostream & os, const Chessboard & chessboard)
{
    return os << "boardSize: " << chessboard.boardSize << ", squareSize: " << chessboard.squareSize;
}

ChessboardImage::ChessboardImage(const cv::Mat & image_, const Chessboard & chessboard, const std::filesystem::path & filename_)
    : image(image_)
    , filename(filename_)
    , isFound(false)
{
    // TODO:
    //  - Detect chessboard corners in image and set the corners member
    isFound = cv::findChessboardCorners(image, chessboard.boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    //  - (optional) Do subpixel refinement of detected corners
    if (isFound) {
        // Convert to grayscale for subpixel refinement (if not already)
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image;
        }
        
        // Subpixel refinement parameters
        cv::Size winSize(11, 11);        // Search window size
        cv::Size zeroZone(-1, -1);       // Dead region in the middle of search zone
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        
        // Refine corner positions to subpixel accuracy
        cv::cornerSubPix(gray, corners, winSize, zeroZone, criteria);
    }
}

void ChessboardImage::drawCorners(const Chessboard & chessboard)
{
    cv::drawChessboardCorners(image, chessboard.boardSize, corners, isFound);
}

void ChessboardImage::drawBox(const Chessboard & chessboard, const Camera & camera)
{
    // DEBUG: Print pose and camera info
    std::println("\n=== DEBUG drawBox for {} ===", filename.string());
    std::println("Camera pose Tnc:");
    std::println("  Translation: [{}, {}, {}]", Tnc.translationVector(0), Tnc.translationVector(1), Tnc.translationVector(2));
    std::println("  Rotation:\n{}", to_string(Tnc.rotationMatrix));
    std::println("Camera intrinsics:");
    std::println("  fx={}, fy={}, cx={}, cy={}", 
                 camera.cameraMatrix.at<double>(0,0), 
                 camera.cameraMatrix.at<double>(1,1),
                 camera.cameraMatrix.at<double>(0,2), 
                 camera.cameraMatrix.at<double>(1,2));
    std::println("Distortion coeffs: {}", to_string(camera.distCoeffs.t()));
    
    // (If you're not using 'chessboard' here, suppress unused warning)
    (void)chessboard;

    // Box definition (same as lab10)
    const cv::Vec3d org(0.0,   0.0,   0.0);
    const cv::Vec3d dim(0.198, 0.132, -0.23);

    // Precompute the 8 vertices in world coords
    const cv::Vec3d v[8] = {
        {org[0],           org[1],           org[2]          }, // 0: (0,0,0)
        {org[0] + dim[0],  org[1],           org[2]          }, // 1: (x,0,0)
        {org[0] + dim[0],  org[1] + dim[1],  org[2]          }, // 2: (x,y,0)
        {org[0],           org[1] + dim[1],  org[2]          }, // 3: (0,y,0)
        {org[0],           org[1],           org[2] + dim[2] }, // 4: (0,0,z)
        {org[0] + dim[0],  org[1],           org[2] + dim[2] }, // 5: (x,0,z)
        {org[0] + dim[0],  org[1] + dim[1],  org[2] + dim[2] }, // 6: (x,y,z)
        {org[0],           org[1] + dim[1],  org[2] + dim[2] }  // 7: (0,y,z)
    };

    // Edges grouped by color (indices into v)
    const std::pair<int,int> edgesBlue[] = {
        {0,1}, {3,2}, {4,5}, {7,6}
    };
    const std::pair<int,int> edgesGreen[] = {
        {0,3}, {1,2}, {4,7}, {5,6}
    };
    const std::pair<int,int> edgesRed[] = {
        {0,4}, {1,5}, {3,7}, {2,6}
    };

    // Convert once: current camera pose Tnc -> body pose Tnb for worldToPixel/isWorldWithinFOV
    const Pose<double> Tnb = camera.cameraToBody(Tnc);

    // Helper: draw one 3D edge with segmentation & FOV checks
    auto drawEdge = [&](const cv::Vec3d& a, const cv::Vec3d& b, const cv::Scalar& color)
    {
        const int lineSegments = 10;

        cv::Vec2d prevPix{};
        bool prevIn = false;
        bool hasPrev = false;

        for (int i = 0; i <= lineSegments; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(lineSegments);
            const cv::Vec3d P = (1.0 - t) * a + t * b;

            const bool in = camera.isWorldWithinFOV(P, Tnb);
            cv::Vec2d pix;
            if (in)
                pix = camera.worldToPixel(P, Tnb);

            if (hasPrev && prevIn && in)
            {
                const cv::Point pt1(static_cast<int>(prevPix[0]), static_cast<int>(prevPix[1]));
                const cv::Point pt2(static_cast<int>(pix[0]),     static_cast<int>(pix[1]));
                cv::line(image, pt1, pt2, color, 3, cv::LINE_AA);
            }

            prevPix = pix;
            prevIn  = in;
            hasPrev = true;
        }
    };

    // Draw groups with the same colors as your original
    // Blue
    for (const auto& e : edgesBlue)  drawEdge(v[e.first],  v[e.second],  cv::Scalar(255, 0, 0));
    // Green
    for (const auto& e : edgesGreen) drawEdge(v[e.first],  v[e.second],  cv::Scalar(0, 255, 0));
    // Red
    for (const auto& e : edgesRed)   drawEdge(v[e.first],  v[e.second],  cv::Scalar(0, 0, 255));
}

void ChessboardImage::recoverPose(const Chessboard & chessboard, const Camera & camera)
{
    std::vector<cv::Point3f> rPNn_all = chessboard.gridPoints();

    cv::Mat Thetacn, rNCc;
    cv::solvePnP(rPNn_all, corners, camera.cameraMatrix, camera.distCoeffs, Thetacn, rNCc);

    Pose<double> Tcn(Thetacn, rNCc);
    Tnc = Tcn.inverse();
}

ChessboardData::ChessboardData(const std::filesystem::path & configPath)
{
    // Ensure the config file exists
    if (!std::filesystem::exists(configPath))
    {
        throw std::runtime_error("Config file does not exist: " + configPath.string());
    }

    // Open the config file
    cv::FileStorage fs(configPath.string(), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        throw std::runtime_error("Failed to open config file: " + configPath.string());
    }

    // Read chessboard configuration
    cv::FileNode node = fs["chessboard_data"];
    node["chessboard"] >> chessboard;
    std::println("Chessboard: {}", to_string(chessboard));

    // Read file pattern for chessboard images
    std::string pattern;
    node["file_regex"] >> pattern;
    fs.release();

    // Create regex object from pattern
    std::regex re(pattern, std::regex_constants::basic | std::regex_constants::icase);
    
    // Get the directory containing the config file
    std::filesystem::path root = configPath.parent_path();
    std::println("Scanning directory {} for file pattern \"{}\"", root.string(), pattern);

    // Populate chessboard images from regex
    chessboardImages.clear();
    if (std::filesystem::exists(root) && std::filesystem::is_directory(root))
    {
        // Iterate through all files in the directory and its subdirectories
        for (const auto & p : std::filesystem::recursive_directory_iterator(root))
        {
            if (std::filesystem::is_regular_file(p))
            {
                // Check if the file matches the regex pattern
                if (std::regex_match(p.path().filename().string(), re))
                {
                    std::print("Loading {}...", p.path().filename().string());

                    // Try to load the file as an image
                    cv::Mat image = cv::imread(p.path().string(), cv::IMREAD_COLOR);

                    bool isImage = !image.empty();
                    if (isImage)
                    {
                        // If it's an image, detect chessboard
                        std::print(" done, detecting chessboard...");
                        ChessboardImage ci(image, chessboard, p.path().filename());
                        std::println("{}", ci.isFound ? " found" : " not found");
                        if (ci.isFound)
                        {
                            chessboardImages.push_back(ci);
                        }
                    }
                    else
                    {
                        // If it's not an image, try to load it as a video
                        cv::VideoCapture cap(p.path().string());
                        bool isVideo = cap.isOpened();
                        if (isVideo)
                        {
                            // Get number of video frames
                            int nFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT)); // TODO 
                            std::println(" done, found {} frames", nFrames);

                            // Loop through selected frames
                            for (int idxFrame = 0; idxFrame < nFrames; idxFrame += 60)
                            {
                                // Read frame
                                std::print("Reading {} frame {}...", p.path().filename().string(), idxFrame);
                                cv::Mat frame;
                                cap.set(cv::CAP_PROP_POS_FRAMES, idxFrame);
                                cap.read(frame);

                                if (frame.empty())
                                {
                                    std::println(" end of file found");
                                    break;
                                }

                                // Detect chessboard in frame
                                std::print(" done, detecting chessboard...");
                                std::string baseName = p.path().stem().string();
                                std::string frameFilename = std::format("{}_{:05d}.jpg", baseName, idxFrame);
                                ChessboardImage ci(frame, chessboard, frameFilename);
                                std::println("{}", ci.isFound ? " found" : " not found");
                                if (ci.isFound)
                                {
                                    chessboardImages.push_back(ci);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void ChessboardData::drawCorners()
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.drawCorners(chessboard);
    }
}

void ChessboardData::drawBoxes(const Camera & camera)
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.drawBox(chessboard, camera);
    }
}

void ChessboardData::recoverPoses(const Camera & camera)
{
    for (auto & chessboardImage : chessboardImages)
    {
        chessboardImage.recoverPose(chessboard, camera);
    }
}

void Camera::calibrate(ChessboardData & chessboardData)
{
    std::vector<cv::Point3f> rPNn_all = chessboardData.chessboard.gridPoints();

    std::vector<std::vector<cv::Point2f>> rQOi_all;
    for (const auto & chessboardImage : chessboardData.chessboardImages)
    {
        rQOi_all.push_back(chessboardImage.corners);
    }
    assert(!rQOi_all.empty());

    imageSize = chessboardData.chessboardImages[0].image.size();
    
    // Use standard 5-parameter distortion model (more stable than 12-parameter)
    // k1, k2, p1, p2, k3 (radial + tangential distortion)
    flags = 0;

    // Find intrinsic and extrinsic camera parameters
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> Thetacn_all, rNCc_all;
    double rms;
    std::print("Calibrating camera...");
    // TODO: Calibrate camera from detected chessboard corners

    // Note: We need to replicate the object points for each image
    std::vector<std::vector<cv::Point3f>> objectPoints_all(rQOi_all.size(), rPNn_all);
    
    rms = cv::calibrateCamera(
        objectPoints_all,    // 3D object points for each image
        rQOi_all,           // 2D image points for each image
        imageSize,          // Image size
        cameraMatrix,       // Camera matrix (output)
        distCoeffs,         // Distortion coefficients (output)
        Thetacn_all,        // Rotation vectors for each image (output)
        rNCc_all,           // Translation vectors for each image (output)
        flags               // Calibration flags
    );
    std::println(" done");
    
    // Pre-compute constants used in isVectorWithinFOV
    calcFieldOfView();

    // Write extrinsic camera parameters for each chessboard image
    assert(chessboardData.chessboardImages.size() == rNCc_all.size());
    assert(chessboardData.chessboardImages.size() == Thetacn_all.size());
    for (std::size_t k = 0; k < chessboardData.chessboardImages.size(); ++k)
    {
        // Set the camera orientation and position (extrinsic camera parameters)
        Pose<double> & Tnc = chessboardData.chessboardImages[k].Tnc;
        cv::Matx33d Rcn;
        cv::Rodrigues(Thetacn_all[k], Rcn);
        Eigen::Map<Eigen::Matrix3d, Eigen::RowMajor> Rcn_eig(Rcn.val);
        
        cv::Vec3d rNCc_vec(rNCc_all[k]);
        
        Tnc.rotationMatrix = Rcn_eig;
        Tnc.translationVector = Eigen::Map<const Eigen::Vector3d>(rNCc_vec.val);
        
        // Store world-fixed pose (inverse of Tcn)
        chessboardData.chessboardImages[k].Tnc = Tnc.inverse();
    }
    
    printCalibration();
    std::println("{:>30} {}", "RMS reprojection error:", rms);

    assert(cv::checkRange(cameraMatrix));
    assert(cv::checkRange(distCoeffs));
}

void Camera::printCalibration() const
{
    std::bitset<8*sizeof(flags)> bitflag(flags);
    std::println("\nCalibration data:");
    std::println("{:>30} {}", "Bit flags:", bitflag.to_string());
    std::println("{:>30}\n{}", "cameraMatrix:", to_string(cameraMatrix));
    std::println("{:>30}\n{}", "distCoeffs:", to_string(distCoeffs.t()));
    std::println("{:>30} (fx, fy) = ({}, {})", "Focal lengths:",
              cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1));       
    std::println("{:>30} (cx, cy) = ({}, {})", "Principal point:",
              cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));     
    std::println("{:>30} {} deg", "Field of view (horizontal):", 180.0/CV_PI*hFOV);
    std::println("{:>30} {} deg", "Field of view (vertical):", 180.0/CV_PI*vFOV);
    std::println("{:>30} {} deg", "Field of view (diagonal):", 180.0/CV_PI*dFOV);
}

void Camera::calcFieldOfView()
{
    assert(cameraMatrix.rows == 3);
    assert(cameraMatrix.cols == 3);
    assert(cameraMatrix.type() == CV_64F);

    // TODO:
    const int w = imageSize.width;
    const int h = imageSize.height;
    const double cx = cameraMatrix.at<double>(0, 2);
    const double cy = cameraMatrix.at<double>(1, 2);

    auto ang = [](const cv::Vec3d &a, const cv::Vec3d &b) {
        double d = a.dot(b);
        d = std::clamp(d, -1.0, 1.0);
        return std::acos(d);
    };

    // Horizontal FOV: rays through left/right edge at principal row
    cv::Vec3d uL = pixelToVector(cv::Vec2d(0,     cy));
    cv::Vec3d uR = pixelToVector(cv::Vec2d(w - 1, cy));
    hFOV = ang(uL, uR);

    // Vertical FOV: rays through top/bottom edge at principal column
    cv::Vec3d uT = pixelToVector(cv::Vec2d(cx, 0));
    cv::Vec3d uB = pixelToVector(cv::Vec2d(cx, h - 1));
    vFOV = ang(uT, uB);

    // Diagonal FOV: rays through opposite corners
    cv::Vec3d u00 = pixelToVector(cv::Vec2d(0,     0));
    cv::Vec3d u11 = pixelToVector(cv::Vec2d(w - 1, h - 1));
    dFOV = ang(u00, u11);
}

cv::Vec3d Camera::worldToVector(const cv::Vec3d & rPNn, const Pose<double> & Tnb) const
{
    // Camera pose Tnc (i.e., Rnc, rCNn)
    Pose<double> Tnc = bodyToCamera(Tnb); // Tnb*Tbc

    // Compute the unit vector uPCc from the world position rPNn and camera pose Tnc
    cv::Vec3d uPCc;
    // Eigen::Map<const Eigen::Vector3d> uPCc_eig(uPCc.val);
    // TODO
    

    // Transform world point to camera coordinates: rPCc = Rcn * (rPNn - rCNn)
    cv::Matx33d Rcn = Tnc.rotationMatrixCV().t();
    cv::Vec3d rPCc = Rcn * (rPNn - Tnc.translationVectorCV());
    double len = cv::norm(rPCc);
    uPCc = rPCc / len;

    return uPCc;
}

cv::Vec2d Camera::worldToPixel(const cv::Vec3d & rPNn, const Pose<double> & Tnb) const
{
    return vectorToPixel(worldToVector(rPNn, Tnb));
}

cv::Vec2d Camera::vectorToPixel(const cv::Vec3d & rPCc) const
{
    // Compute the pixel location (rQOi) for the given vector (rPCc)
    cv::Vec2d rQOi;
    // TODO
    std::vector<cv::Point3d> objPts{ cv::Point3d(rPCc[0], rPCc[1], rPCc[2]) };
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // identity rotation (already in camera coords)
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // zero translation
    std::vector<cv::Point2d> imgPts;
    cv::projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, imgPts);
    rQOi = cv::Vec2d(imgPts[0].x, imgPts[0].y);
    return rQOi;
}

Eigen::Vector2d Camera::vectorToPixel(const Eigen::Vector3d & rPCc, Eigen::Matrix23d & J) const
{
    Eigen::Vector2d rQOi;
    cv::Vec2d rQOi_cv;
    // TODO: Lab 8 (optional)
    cv::Vec3d rPCc_cv;
    cv::eigen2cv(rPCc, rPCc_cv);  // copies into cv

    std::vector<cv::Point3d> objPts{ cv::Point3d(rPCc_cv[0], rPCc_cv[1], rPCc_cv[2]) };
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // identity rotation (already in camera coords)
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // zero translation
    std::vector<cv::Point2d> imgPts;
    cv::projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, imgPts);
    rQOi_cv = cv::Vec2d(imgPts[0].x, imgPts[0].y);
    cv::cv2eigen(rQOi_cv, rQOi);  // copies into eigen

    return rQOi;
}

cv::Vec3d Camera::pixelToVector(const cv::Vec2d & rQOi) const
{
    // Compute unit vector (uPCc) for the given pixel location (rQOi)
    cv::Vec3d uPCc;
    // TODO
    std::vector<cv::Point2d> imgPts{ cv::Point2d(rQOi[0], rQOi[1]) };
    std::vector<cv::Point2d> undistPts;
    cv::undistortPoints(imgPts, undistPts, cameraMatrix, distCoeffs);
    cv::Vec3d dir(undistPts[0].x, undistPts[0].y, 1.0);
    double n = cv::norm(dir);
    uPCc = (n > std::numeric_limits<double>::epsilon()) ? (dir / n) : cv::Vec3d(0,0,1);
    return uPCc;
}

bool Camera::isVectorWithinFOV(const cv::Vec3d & rPCc) const
{
    // Forward only
    if (rPCc[2] <= 0.0) return false;

    // Unit direction
    const double n = cv::norm(rPCc);
    if (n <= std::numeric_limits<double>::epsilon()) return false;
    const cv::Vec3d u = rPCc / n;

    // FOVs (fallback if not precomputed)
    double h = hFOV, v = vFOV, d = dFOV;
    if (h <= 0.0 || v <= 0.0 || d <= 0.0) {
        const double fx = cameraMatrix.at<double>(0, 0);
        const double fy = cameraMatrix.at<double>(1, 1);
        const double w  = static_cast<double>(imageSize.width);
        const double hpx= static_cast<double>(imageSize.height);

        // pinhole relations
        h = 2.0 * std::atan((w  * 0.5) / fx);
        v = 2.0 * std::atan((hpx* 0.5) / fy);
        // diagonal FOV: corner ray from principal point
        const double tx = (w  * 0.5) / fx;
        const double ty = (hpx* 0.5) / fy;
        d = 2.0 * std::atan(std::sqrt(tx*tx + ty*ty));
    }

    // --- Bigger cone: use diagonal FOV so corners are included ---
    // Tweak 'scale' if you want it slightly tighter (<1) or looser (>1, but clamp).
    const double scale = 0.99; // 1.00 = full screen; try 1.05 if you want a tiny buffer
    double halfAngle = 0.5 * d * scale;

    // Safety clamp (keep within forward hemisphere)
    const double maxHalf = 0.5 * CV_PI - 1e-4;
    if (halfAngle > maxHalf) halfAngle = maxHalf;

    // Cosine test (optical axis = +Z)
    const double cosTheta = u[2];
    const double cosHalf  = std::cos(halfAngle);
    return cosTheta >= cosHalf;
}

bool Camera::isWorldWithinFOV(const cv::Vec3d & rPNn, const Pose<double> & Tnb) const
{
    return isVectorWithinFOV(worldToVector(rPNn, Tnb));
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Camera::undistort(const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOi) const
{
    // Convert from Eigen matrix to std::vector of cv::Point2d
    std::vector<cv::Point2d> rQOi_cv(rQOi.cols());
    for (int i = 0; i < rQOi.cols(); ++i)
    {
        rQOi_cv[i] = cv::Point2d(rQOi(0, i), rQOi(1, i));
    }

    // Undistort points
    std::vector<cv::Point2d> rQbarOi_cv;
    
    // Use cv::undistortPoints to remove distortion
    // We want pixel coordinates back, so we pass the camera matrix as P parameter
    cv::undistortPoints(rQOi_cv, rQbarOi_cv, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);

    // Convert from std::vector of cv::Point2d to Eigen matrix
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOi(2, rQOi.cols());
    for (int i = 0; i < rQbarOi_cv.size(); ++i)
    {
        rQbarOi(0, i) = rQbarOi_cv[i].x;
        rQbarOi(1, i) = rQbarOi_cv[i].y;
    }

    return rQbarOi;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Camera::undistort(const Eigen::Matrix<double, 3, Eigen::Dynamic> & pQOi) const
{
    // Extract the Euclidean points from the homogeneous coordinates
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOi = pQOi.topRows<2>().array().rowwise() / pQOi.row(2).array();

    // Call the Euclidean undistort function
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOi = undistort(rQOi);

    // Create the output matrix with homogeneous coordinates
    Eigen::Matrix<double, 3, Eigen::Dynamic> pQbarOi(3, pQOi.cols());
    pQbarOi.topRows<2>() = rQbarOi;
    pQbarOi.row(2).setOnes();

    return pQbarOi;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Camera::distort(const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQbarOi) const
{
    double fx = cameraMatrix.at<double>( 0,  0);
    double fy = cameraMatrix.at<double>( 1,  1);
    double cx = cameraMatrix.at<double>( 0,  2);
    double cy = cameraMatrix.at<double>( 1,  2);

    // Convert from Euclidean coordinates to homogeneous coordinates
    Eigen::Matrix<double, 3, Eigen::Dynamic> pQbarOi(3, rQbarOi.cols());
    pQbarOi.topRows<2>() = rQbarOi;
    pQbarOi.row(2).setOnes();

    // Solve K*rPCc = pQbarOi for rPCc
    // K^(-1) * pQbarOi = rPCc
    // We need to "unproject" the undistorted pixel coordinates
    Eigen::Matrix<double, 3, Eigen::Dynamic> rPCc(3, rQbarOi.cols());
    
    for (int i = 0; i < rQbarOi.cols(); ++i)
    {
        // Convert undistorted pixel to normalized coordinates
        double xn = (rQbarOi(0, i) - cx) / fx;
        double yn = (rQbarOi(1, i) - cy) / fy;
        
        // Create 3D point in camera coordinates (z=1 for ray direction)
        rPCc(0, i) = xn;
        rPCc(1, i) = yn;
        rPCc(2, i) = 1.0;
    }

    // Use camera model (with lens distortion) to get pixel coordinates
    // Convert to cv::Point3d and use cv::projectPoints to apply distortion
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOi(2, rQbarOi.cols());
    
    for (int i = 0; i < rPCc.cols(); ++i)
    {
        std::vector<cv::Point3d> objPts{ cv::Point3d(rPCc(0, i), rPCc(1, i), rPCc(2, i)) };
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // identity rotation
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // zero translation
        std::vector<cv::Point2d> imgPts;
        cv::projectPoints(objPts, rvec, tvec, cameraMatrix, distCoeffs, imgPts);
        
        rQOi(0, i) = imgPts[0].x;
        rQOi(1, i) = imgPts[0].y;
    }
    
    return rQOi;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Camera::distort(const Eigen::Matrix<double, 3, Eigen::Dynamic> & pQbarOi) const
{
    // Convert from homogeneous coordinates to Euclidean coordinates
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQbarOi = pQbarOi.topRows<2>().array().rowwise() / pQbarOi.row(2).array();

    // Call the Euclidean distort function
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOi = distort(rQbarOi);

    // Convert from Euclidean coordinates to homogeneous coordinates
    Eigen::Matrix<double, 3, Eigen::Dynamic> pQOi(3, pQbarOi.cols());
    pQOi.topRows<2>() = rQOi;
    pQOi.row(2).setOnes();

    return pQOi;
}

void Camera::write(cv::FileStorage & fs) const
{
    fs << "{"
       << "camera_matrix"           << cameraMatrix
       << "distortion_coefficients" << distCoeffs
       << "flags"                   << flags
       << "imageSize"               << imageSize
       << "}";
}

void Camera::read(const cv::FileNode & node)
{
    node["camera_matrix"]           >> cameraMatrix;
    node["distortion_coefficients"] >> distCoeffs;
    node["flags"]                   >> flags;
    node["imageSize"]               >> imageSize;

    // Pre-compute constants used in isVectorWithinFOV
    calcFieldOfView();

    assert(cameraMatrix.cols == 3);
    assert(cameraMatrix.rows == 3);
    assert(cameraMatrix.type() == CV_64F);
    assert(distCoeffs.cols == 1);
    assert(distCoeffs.type() == CV_64F);
}
