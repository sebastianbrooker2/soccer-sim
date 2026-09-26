#include <cassert>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <format>
#include <vector>
#include <filesystem>
#include <regex>
#include <algorithm>
#include <print>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include "calibrate.h"
#include "Camera.h"
#include "visualNavigation.h"
#include "to_string.hpp"


// Main calibration function
void calibrateCamera(const std::filesystem::path & configPath)
{
    // - Read XML at configPath
    cv::FileStorage fs(configPath.string(), cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        throw std::runtime_error(std::string("Failed to open config: ") + configPath.string());
    }

    // Read chessboard 
    cv::FileNode node = fs["chessboard_data"];
    Chessboard chessboard;
    node["chessboard"] >> chessboard;  // Now works with the template
    std::println("Chessboard: {}", to_string(chessboard));

    // Read video file name from file_regex (removing quotes)
    std::string fileRegex;
    node["file_regex"] >> fileRegex;
    fs.release();

    // Remove quotes from the filename if present
    if (fileRegex.front() == '"' && fileRegex.back() == '"') {
        fileRegex = fileRegex.substr(1, fileRegex.length() - 2);
    }

    // - Parse XML and extract relevant frames from source video containing the chessboard
    // Construct full path to video file in same directory as config
    std::filesystem::path videoPath = configPath.parent_path() / fileRegex;
    std::println("Loading video: {}", videoPath.string());

    // Populate chessboard images from video
    std::vector<ChessboardImage> chessboardImages;
    
    // Load as video
    cv::VideoCapture cap(videoPath.string());
    if (cap.isOpened())
    {
        // Get number of video frames
        int nFrames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
        std::println("Found {} frames", nFrames);

        // Loop through selected frames
        for (int idxFrame = 0; idxFrame < nFrames; idxFrame += 10)
        {
            // Read frame
            std::print("Reading frame {}...", idxFrame);
            cv::Mat frame;
            cap.set(cv::CAP_PROP_POS_FRAMES, idxFrame);
            cap >> frame;

            if (frame.empty())
            {
                std::println(" end of file found");
                break;
            }

            // Detect chessboard in frame
            std::print(" done, detecting chessboard...");
            std::string frameFilename = std::format("frame_{:05d}.jpg", idxFrame);
            ChessboardImage ci(frame, chessboard, frameFilename);
            std::println("{}", ci.isFound ? " found" : " not found");
            if (ci.isFound)
            {
                chessboardImages.push_back(ci);
            }
        }
    }

    // - Perform camera calibration
    std::vector<cv::Point3f> rPNn_all = chessboard.gridPoints();

    std::vector<std::vector<cv::Point2f>> rQOi_all;
    for (const auto & chessboardImage : chessboardImages)
    {
        rQOi_all.push_back(chessboardImage.corners);
    }
    assert(!rQOi_all.empty());

    cv::Size imageSize = chessboardImages[0].image.size();
    int flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL;

    // Find intrinsic and extrinsic camera parameters
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(12, 1, CV_64F);
    std::vector<cv::Mat> Thetacn_all, rNCc_all;
    double rms;
    std::print("Calibrating camera...");

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

    // - Write the camera matrix and lens distortion parameters to camera.xml file in same directory as configPath
    std::filesystem::path cameraPath = configPath.parent_path() / "camera.xml";
    cv::FileStorage fsOut(cameraPath.string(), cv::FileStorage::WRITE);
    fsOut << "camera" << "{"
          << "camera_matrix" << cameraMatrix
          << "distortion_coefficients" << distCoeffs
          << "flags" << flags
          << "imageSize" << imageSize
          << "}";
    fsOut.release();

// - Visualise the camera calibration results
    std::println("\nCalibration data:");
    std::println("{:>30} {}", "RMS reprojection error:", rms);
    std::println("{:>30} (fx, fy) = ({}, {})", "Focal lengths:",
              cameraMatrix.at<double>(0, 0), cameraMatrix.at<double>(1, 1));       
    std::println("{:>30} (cx, cy) = ({}, {})", "Principal point:",
              cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));

    // Draw corners on detected images and display them
    for (auto & chessboardImage : chessboardImages)
    {
        cv::drawChessboardCorners(chessboardImage.image, chessboard.boardSize, chessboardImage.corners, chessboardImage.isFound);
        
        cv::imshow("Calibration images (press ESC to quit, any other key to continue)", chessboardImage.image);
        char c = static_cast<char>(cv::waitKey(0));
        if (c == 27) // ESC to quit, any other key to continue
            break;
    }
}
