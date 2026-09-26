#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "Camera.h"
#include "calibrate.h"

void calibrateCamera(const std::filesystem::path & configPath)
{
    // TODO (Assignment 2): This implementation copied from Lab 11
    // - Read XML at configPath
    // - Parse XML and extract relevant frames from source video containing the chessboard
    // - Perform camera calibration
    // - Write the camera matrix and lens distortion parameters to camera.xml file in same directory as configPath
    // - Visualise the camera calibration results

    // Read chessboard data using configuration file
    ChessboardData chessboardData(configPath);

    // Calibrate camera from chessboard data
    Camera cam;
    cam.calibrate(chessboardData);

    // Write camera calibration to file
    std::filesystem::path cameraPath = configPath.parent_path() / "camera.xml";
    cv::FileStorage fs(cameraPath.string(), cv::FileStorage::WRITE);
    fs << "camera" << cam;
    fs.release();

    // Set Tbc for calibration visualization to match visual odometry
    // b1 = c3, b2 = c1, b3 = c2 (per Lab 11 Section 4a)
    cam.Tbc.rotationMatrix << 0, 0, 1,
                               1, 0, 0,
                               0, 1, 0;
    cam.Tbc.translationVector.setZero();

    // Visualise the camera calibration results (draw both corners and 3D box)
    chessboardData.drawCorners();
    chessboardData.drawBoxes(cam);
    for (auto & chessboardImage : chessboardData.chessboardImages)
    {
        // Resize image for display (scale down by factor of 2)
        // Note: boxes are already drawn on chessboardImage.image, so resize that
        cv::Mat displayImage;
        cv::resize(chessboardImage.image, displayImage, cv::Size(), 0.5, 0.5);
        
        cv::imshow("Calibration images", displayImage);
        char c = static_cast<char>(cv::waitKey(0));
        if (c == 27 || c == 'q' || c == 'Q') // ESC, q or Q to quit, any other key to continue
            break;
    }
}
