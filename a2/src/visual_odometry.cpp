#include <print>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include "to_string.hpp"
#include "BufferedVideo.h"
#include "Pose.hpp"
#include "rotation.hpp"
#include "Camera.h"
#include "DJIVideoCaption.h"
#include "rotation.hpp"
#include "GaussianInfo.hpp"
#include "funcmin.hpp"
#include "SystemVisualNav.h"
#include "MeasurementOutdoorFlowBundle.h"
#include "visual_odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// Forward declarations
static Eigen::Vector6d getInitialPose(const DJIVideoCaption & caption0);
static void plotGroundPlane(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor);
static void plotHorizon(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor);
static void plotCompass(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor);
static void plotEpipole(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector6d & etakm1, const Camera & camera, const int & divisor);


void runVisualOdometryFromVideo(const std::filesystem::path & videoPath, const std::filesystem::path & cameraPath, const std::filesystem::path & outputDirectory)
{
    // TODO: Lab 11
    int divisor         = 2;                    // Image scaling factor (used for plotting only)
    int imgModulus      = 6;                    // Take frames divisible by this number

    assert(!videoPath.empty());

    // Subtitle path
    std::filesystem::path subtitlePath = videoPath.parent_path() / (videoPath.stem().string() + ".SRT");
    assert(std::filesystem::exists(subtitlePath));

    // Load and parse subtitle file
    std::vector<DJIVideoCaption> djiVideoCaption = getVideoCaptions(subtitlePath);

    // Output video path
    std::filesystem::path outputPath;
    bool doExport = !outputDirectory.empty();
    if (doExport)
    {
        std::string outputFilename = videoPath.stem().string()
                                   + "_"
                                   + std::to_string(divisor)
                                   + "_"
                                   + std::to_string(imgModulus)
                                   + videoPath.extension().string();
        outputPath = outputDirectory / outputFilename;
    }

    // Load camera calibration
    Camera camera;
    assert(std::filesystem::exists(cameraPath));
    cv::FileStorage fs(cameraPath.string(), cv::FileStorage::READ);
    assert(fs.isOpened());
    fs["camera"] >> camera;

    // Display loaded calibration data
    camera.printCalibration();

    // Set camera pose w.r.t. body
    // TODO: Lab 11
    // b1 = c3, b2 = c1, b3 = c2
    camera.Tbc.rotationMatrix << 0, 0, 1,
                                  1, 0, 0,
                                  0, 1, 0;
    camera.Tbc.translationVector.setZero();

    // Open input video
    cv::VideoCapture cap(videoPath.string());
    assert(cap.isOpened());
    int nFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    assert(nFrames > 0);

    std::println("Input video: {}", videoPath.string());
    std::println("Subtitle file: {}", subtitlePath.string());
    std::println("Total number of frames: {}", nFrames);
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::println("Input video frame rate: {}", fps);
    std::println("Input video dimensions: [{} x {}]",
                cap.get(cv::CAP_PROP_FRAME_WIDTH),
                cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    BufferedVideoReader bufferedVideoReader(5);
    bufferedVideoReader.start(cap);

    cv::VideoWriter videoOut;
    BufferedVideoWriter bufferedVideoWriter(3);
    if (doExport)
    {
        cv::Size frameSize;
        frameSize.width     = cap.get(cv::CAP_PROP_FRAME_WIDTH)/divisor;
        frameSize.height    = cap.get(cv::CAP_PROP_FRAME_HEIGHT)/divisor;
        double outputFps    = fps/imgModulus;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        videoOut.open(outputPath.string(), codec, outputFps, frameSize);
        bufferedVideoWriter.start(videoOut);
    }

    // Visual odometry
    auto p0 = GaussianInfo<double>::fromSqrtInfo(Eigen::VectorXd::Zero(18), Eigen::MatrixXd::Zero(18, 18));
    SystemVisualNav system(p0);
    Eigen::VectorXd etakm1(6);
    Eigen::VectorXd etak(6);
    etak = getInitialPose(djiVideoCaption[0]);
    
    // DEBUG: Print initial pose
    std::println("=== INITIAL POSE ===");
    std::println("  GPS altitude: {:.2f} m", djiVideoCaption[0].altitude);
    std::println("  Initial AGL:  {:.2f} m (GPS - 7m)", djiVideoCaption[0].altitude - 7);
    std::println("  Initial D (etak(2)): {:.2f} m", etak(2));
    std::println("  Initial altitude (-D): {:.2f} m", -etak(2));
    std::println("====================");

    cv::Mat imgk_raw;
    cv::Mat imgkm1_raw;
    Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1;

    for (int i = 0, k = 0;; ++i)
    {
        imgk_raw = bufferedVideoReader.read();
        if (imgk_raw.empty())
        {
            break;
        }

        if (i % imgModulus == 0)
        {
            if (k > 0)
            {
                MeasurementOutdoorFlowBundle measurement(i/fps, camera, imgk_raw, imgkm1_raw, rQOikm1);

                rQOikm1 = measurement.trackedPreviousFeatures();
                const Eigen::Matrix<double, 2, Eigen::Dynamic> & rQOik = measurement.trackedCurrentFeatures();

                auto costFunc = [&](const Eigen::VectorXd & etak, Eigen::VectorXd & g, Eigen::MatrixXd & H)
                {
                    return measurement.costOdometry(etak, etakm1, g, H);
                };

                etak = etakm1;
                const int verbosity = 3;
                int ret = funcmin::NewtonTrust(costFunc, etak, verbosity);
                assert(ret == 0);

                Eigen::Vector3d rBNn = etak.head<3>();
                Eigen::Matrix3d Rnb = rpy2rot(etak.tail<3>());

                std::print("rBNn: \n{}\n", to_string(rBNn));
                std::print("Rnb: \n{}\n", to_string(Rnb));
                
                // DEBUG: Altitude and orientation tracking
                double estimated_altitude = -etak(2);  // Negative because D is positive down in NED
                double estimated_altitude_prev = -etakm1(2);
                double altitude_change = estimated_altitude - estimated_altitude_prev;
                double gps_altitude = djiVideoCaption[i].altitude;
                double altitude_diff = estimated_altitude - gps_altitude;
                double roll_deg = etak(3) * 180.0 / M_PI;
                double pitch_deg = etak(4) * 180.0 / M_PI;
                double yaw_deg = etak(5) * 180.0 / M_PI;
                
                std::println("=== DEBUG Frame {} (t={:.2f}s) ===", i, i/fps);
                std::println("  Estimated altitude: {:.2f} m (change: {:+.3f} m)", estimated_altitude, altitude_change);
                std::println("  GPS altitude:       {:.2f} m", gps_altitude);
                std::println("  Altitude drift:     {:.2f} m", altitude_diff);
                std::println("  Roll:  {:.2f} deg", roll_deg);
                std::println("  Pitch: {:.2f} deg", pitch_deg);
                std::println("  Yaw:   {:.2f} deg", yaw_deg);
                std::println("=====================================");

                Eigen::VectorXd x(18);
                x.setZero();
                x.segment<6>(6) = etak;
                x.segment<6>(12) = etakm1;

                cv::Mat imgout;
                cv::resize(imgk_raw, imgout, cv::Size(), 1.0/divisor, 1.0/divisor);

                Eigen::Matrix<double, 2, Eigen::Dynamic> rQOik_hat = measurement.predictedFeatures(x, system);

                std::vector<cv::Point2d> rQOikm1_scaled, rQOik_scaled, rQOik_hat_scaled;
                int np = rQOik.cols();
                rQOikm1_scaled.resize(np);
                rQOik_scaled.resize(np);
                rQOik_hat_scaled.resize(np);
                for (int j = 0; j < np; ++j)
                {
                    rQOikm1_scaled[j].x = rQOikm1(0, j)/divisor;
                    rQOikm1_scaled[j].y = rQOikm1(1, j)/divisor;
                    rQOik_scaled[j].x = rQOik(0, j)/divisor;
                    rQOik_scaled[j].y = rQOik(1, j)/divisor;
                    rQOik_hat_scaled[j].x = rQOik_hat(0, j)/divisor;
                    rQOik_hat_scaled[j].y = rQOik_hat(1, j)/divisor;
                }

                for (int j = 0; j < rQOik.cols(); ++j)
                {
                    cv::arrowedLine(imgout, rQOikm1_scaled[j], rQOik_hat_scaled[j], cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

                    if (measurement.inlierMask()[j])
                    {
                        cv::arrowedLine(imgout, rQOikm1_scaled[j], rQOik_scaled[j], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                    }
                    else
                    {
                        cv::arrowedLine(imgout, rQOikm1_scaled[j], rQOik_scaled[j], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                    }
                }

                plotGroundPlane(imgout, etak, camera, divisor);
                plotHorizon(imgout, etak, camera, divisor);
                plotCompass(imgout, etak, camera, divisor);
                plotEpipole(imgout, etak, etakm1, camera, divisor);

                cv::imshow("Visual odometry demo", imgout);
                char key = cv::waitKey(1);
                if (key == 'q')
                {
                    std::println("Key '{}' pressed. Terminating program.", key);
                    break;
                }

                if (doExport)
                {
                    bufferedVideoWriter.write(imgout);
                }

                rQOikm1.resize(2, rQOik.cols());
                rQOikm1 = rQOik;
            }

            imgk_raw.copyTo(imgkm1_raw);
            etakm1 = etak;
            k++;
        }
    } 

    if (doExport)
    {
         bufferedVideoWriter.stop();
    }
    bufferedVideoReader.stop();
}

Eigen::Vector6d getInitialPose(const DJIVideoCaption & caption0)
{
    double h = caption0.altitude;
    double ga = h - 7;

    Eigen::Vector6d eta0;
    eta0(0) = 0;
    eta0(1) = 0;
    eta0(2) = -ga;
    eta0(3) = 0;
    eta0(4) = 0;
    eta0(5) = 0;
    
    return eta0;
}

void plotGroundPlane(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor)
{
    Pose<double> Tnb(etak);
    
    for (int e = -800; e <= 800; e += 80)
    {
        std::vector<cv::Point2d> linePoints;
        for (int n = -800; n <= 800; n += 10)
        {
            cv::Vec3d rPNn(n, e, 0);
            cv::Vec3d rPCc_cv = camera.worldToVector(rPNn, Tnb);
            
            if (camera.isVectorWithinFOV(rPCc_cv))
            {
                cv::Vec2d rQOi = camera.vectorToPixel(rPCc_cv);
                linePoints.push_back(cv::Point2d(rQOi[0] / divisor, rQOi[1] / divisor));
            }
        }
        
        for (size_t i = 1; i < linePoints.size(); ++i)
        {
            cv::line(img, linePoints[i-1], linePoints[i], cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        }
    }
    
    for (int n = -800; n <= 800; n += 80)
    {
        std::vector<cv::Point2d> linePoints;
        for (int e = -800; e <= 800; e += 10)
        {
            cv::Vec3d rPNn(n, e, 0);
            cv::Vec3d rPCc_cv = camera.worldToVector(rPNn, Tnb);
            
            if (camera.isVectorWithinFOV(rPCc_cv))
            {
                cv::Vec2d rQOi = camera.vectorToPixel(rPCc_cv);
                linePoints.push_back(cv::Point2d(rQOi[0] / divisor, rQOi[1] / divisor));
            }
        }
        
        for (size_t i = 1; i < linePoints.size(); ++i)
        {
            cv::line(img, linePoints[i-1], linePoints[i], cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        }
    }
}

void plotHorizon(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor)
{
    // Use direction vectors at infinity (altitude-independent)
    Eigen::Vector3d rpy = etak.tail<3>();
    Eigen::Matrix3d Rnb = rpy2rot(rpy);
    Eigen::Matrix3d Rbc = camera.Tbc.rotationMatrix;
    Eigen::Matrix3d Rnc = Rnb * Rbc;
    Eigen::Matrix3d Rnc_T = Rnc.transpose();
    
    std::vector<cv::Point2d> horizonPoints;
    
    // Sample 60 directions around horizon (horizontal plane in NED)
    for (double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/60.0)
    {
        // Unit vector in horizontal plane (NED frame)
        Eigen::Vector3d dir_n(cos(theta), sin(theta), 0.0);
        
        // Transform to camera frame
        Eigen::Vector3d dir_c = Rnc_T * dir_n;
        cv::Vec3d dir_c_cv(dir_c(0), dir_c(1), dir_c(2));
        
        if (camera.isVectorWithinFOV(dir_c_cv))
        {
            cv::Vec2d rQOi = camera.vectorToPixel(dir_c_cv);
            horizonPoints.push_back(cv::Point2d(rQOi[0] / divisor, rQOi[1] / divisor));
        }
    }
    
    if (horizonPoints.size() > 1)
    {
        for (size_t i = 0; i < horizonPoints.size() - 1; ++i)
        {
            cv::line(img, horizonPoints[i], horizonPoints[i+1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
    }
}

void plotCompass(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, const int & divisor)
{
    // Use direction vectors at infinity (altitude-independent)
    Eigen::Vector3d rpy = etak.tail<3>();
    Eigen::Matrix3d Rnb = rpy2rot(rpy);
    Eigen::Matrix3d Rbc = camera.Tbc.rotationMatrix;
    Eigen::Matrix3d Rnc = Rnb * Rbc;
    Eigen::Matrix3d Rnc_T = Rnc.transpose();
    
    const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    double angles[] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -3*M_PI/4, -M_PI/2, -M_PI/4};
    
    for (int i = 0; i < 8; ++i)
    {
        // Unit vector in horizontal plane (NED frame)
        Eigen::Vector3d dir_n(cos(angles[i]), sin(angles[i]), 0.0);
        
        // Transform to camera frame
        Eigen::Vector3d dir_c = Rnc_T * dir_n;
        cv::Vec3d dir_c_cv(dir_c(0), dir_c(1), dir_c(2));
        
        if (camera.isVectorWithinFOV(dir_c_cv))
        {
            cv::Vec2d rQOi = camera.vectorToPixel(dir_c_cv);
            cv::Point2d pt(rQOi[0] / divisor, rQOi[1] / divisor);
            cv::putText(img, directions[i], pt, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        }
    }
}

void plotEpipole(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector6d & etakm1, const Camera & camera, const int & divisor)
{
    Eigen::Vector3d rBNn_k = etak.head<3>();
    Eigen::Vector3d rBNn_km1 = etakm1.head<3>();
    Eigen::Vector3d velocity_approx = rBNn_k - rBNn_km1;
    
    if (velocity_approx.norm() > 0.001)
    {
        Pose<double> Tnb(etak);
        Eigen::Vector3d vel_scaled = velocity_approx * 1000;
        cv::Vec3d vel_c(vel_scaled(0), vel_scaled(1), vel_scaled(2));
        vel_c = camera.worldToVector(vel_c, Tnb);
        
        if (camera.isVectorWithinFOV(vel_c))
        {
            cv::Vec2d epipole = camera.vectorToPixel(vel_c);
            cv::Point2d pt(epipole[0] / divisor, epipole[1] / divisor);
            cv::circle(img, pt, 15, cv::Scalar(0, 165, 255), 3, cv::LINE_AA);
        }
    }
}
