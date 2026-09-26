#include <filesystem>
#include <string>
#include <print>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include "BufferedVideo.h"
#include "Camera.h"
#include "DJIVideoCaption.h"
#include "Event.h"
#include "GaussianInfo.hpp"
#include "MeasurementAltimeter.h"
#include "MeasurementOutdoorFlowBundle.h"
#include "MeasurementIndoorFlowBundle.h"
#include "MeasurementIdenticalTagBundle.h"
#include "MeasurementPointBundle.h"
#include "SystemSLAMPointLandmarks.h"
#include "Pose.hpp"
#include "rotation.hpp"
#include "SystemVisualNav.h"
#include "to_string.hpp"
#include "visual_odometry.h"
#include "visualNavigation.h"
#include "Plot.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// Forward declarations for visualization
static Eigen::Vector6d getBodyPose(const SystemVisualNav & system);
static void plotGroundPlane(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor);
static void plotHorizon(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor);
static void plotCompass(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor);
static void plotVelocityMarker(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector6d & etakm1, const Camera & camera, int divisor);
static void plotStateText(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector3d & gps_ned, double current_altitude);
static void plotGPSValidation(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector3d & gps_ned, double pos_error, double rms_error);
static void plotTrajectory(cv::Mat & img, const std::vector<Eigen::Vector2d> & est_path);

// Helper function to convert GPS lat/lon to NED coordinates
static Eigen::Vector3d latLonToNED(double lat, double lon, double alt, double lat0, double lon0, double alt0);

void runVisualNavigationFromVideo(const std::filesystem::path & videoPath, const std::filesystem::path & cameraPath, int scenario, int interactive, const std::filesystem::path & outputDirectory)
{
    assert(!videoPath.empty());

    // Subtitle path for Scenario 4
    std::filesystem::path subtitlePath;
    if (scenario == 4)
    {
        subtitlePath = videoPath.parent_path() / (videoPath.stem().string() + ".SRT");
        std::println("Subtitle file: {}", subtitlePath.string());
        assert(std::filesystem::exists(subtitlePath));
    }

    // Output video path
    std::filesystem::path outputPath;
    bool doExport = !outputDirectory.empty();
    if (doExport)
    {
        std::string outputFilename = videoPath.stem().string()
                                   + "_s" + std::to_string(scenario)
                                   + "_out"
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

    // Set camera pose w.r.t. body (SAME FOR ALL SCENARIOS!)
    // Assignment states: b1=c3, b2=c1, b3=c2 (universal relationship)
    camera.Tbc.rotationMatrix << 0, 0, 1,
                                  1, 0, 0,
                                  0, 1, 0;
    camera.Tbc.translationVector.setZero();

    // Open input video
    cv::VideoCapture cap(videoPath.string());
    assert(cap.isOpened());
    int nFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    assert(nFrames > 0);
    double fps = cap.get(cv::CAP_PROP_FPS);

    std::println("Input video: {}", videoPath.string());
    std::println("Total frames: {}", nFrames);
    std::println("Frame rate: {}", fps);

    BufferedVideoReader bufferedVideoReader(5);
    bufferedVideoReader.start(cap);

    cv::VideoWriter videoOut;
    BufferedVideoWriter bufferedVideoWriter(3);
    int divisor = 2;  // Image scaling for visualization
    if (doExport)
    {
        cv::Size frameSize;
        frameSize.width  = cap.get(cv::CAP_PROP_FRAME_WIDTH) / divisor;
        frameSize.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT) / divisor;
        double outputFps = fps;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        videoOut.open(outputPath.string(), codec, outputFps, frameSize);
        bufferedVideoWriter.start(videoOut);
    }

    // Visual navigation - Scenario 4 specific
    if (scenario == 4)
    {
        std::println("\n=== SCENARIO 4: OUTDOOR FLIGHT ===");
        std::println("Loading subtitle data...");
        
        // Load subtitle data
        std::vector<DJIVideoCaption> captions = getVideoCaptions(subtitlePath);
        std::println("Loaded {} caption entries", captions.size());
        
        // Initialize 18-state system (nu=6, eta=6, zeta=6, no landmarks)
        std::println("Initializing 18-state SystemVisualNav...");
        auto p0 = GaussianInfo<double>::fromSqrtInfo(Eigen::VectorXd::Zero(18), Eigen::MatrixXd::Zero(18, 18));
        SystemVisualNav system(p0);
        system.setScenario(4);  // Set scenario for process noise tuning
        
        // Compute initial yaw from GPS heading (look far ahead to get actual motion direction)
        // Need to look at frame ~200-300 to see actual flight direction (not just hover)
        double lat0 = captions[0].latitude;
        double lon0 = captions[0].longitude;
        double lat1 = captions[std::min(300, (int)captions.size()-1)].latitude;
        double lon1 = captions[std::min(300, (int)captions.size()-1)].longitude;
        
        double dLat = (lat1 - lat0) * M_PI / 180.0;  // Latitude → North
        double dLon = (lon1 - lon0) * M_PI / 180.0;  // Longitude → East
        
        // DEBUG: Print GPS motion to understand direction
        std::println("\n════ INITIAL YAW CALCULATION DEBUG ════");
        std::println("Frame 0:   lat={}, lon={}", lat0, lon0);
        std::println("Frame 300: lat={}, lon={}", lat1, lon1);
        std::println("dLat (North change) = {} rad = {}°", dLat, dLat * 180.0 / M_PI);
        std::println("dLon (East change)  = {} rad = {}°", dLon, dLon * 180.0 / M_PI);
        
        double initial_yaw = std::atan2(dLon, dLat);  // atan2(East, North) gives yaw from North axis
        std::println("initial_yaw = atan2({}, {}) = {} rad = {}°", dLon, dLat, initial_yaw, initial_yaw * 180.0 / M_PI);
        std::println("This means drone is heading {}° from North", initial_yaw * 180.0 / M_PI);
        std::println("════════════════════════════════════════\n");
        
        // Set initial pose from first subtitle
        double h0 = captions[0].altitude;
        double ga0 = h0 - 7.0;  // Ground altitude offset (from Lab 11)
        Eigen::VectorXd x0(18);
        x0.setZero();
        
        // Initialize eta (current pose) - states 6-11
        x0(8) = -ga0;                        // D component (negative altitude in NED)
        x0(10) = -0.7 * M_PI / 180.0;        // Pitch: -5° (camera looking slightly down)
        x0(11) = initial_yaw;                // Yaw from GPS heading
        
        // Initialize zeta (delayed pose) - states 12-17 
        // CRITICAL: Must match eta initially so homography calculations work on frame 1!
        x0(14) = -ga0;                       // Delayed D component (same as current)
        x0(16) = -0.7 * M_PI / 180.0;        // Delayed pitch (same as current)
        x0(17) = initial_yaw;                // Delayed yaw (same as current)
        
        // CRITICAL FIX: Much tighter initial conditions to prevent divergence
        Eigen::MatrixXd Sigma0 = Eigen::MatrixXd::Identity(18, 18);
        for (int i = 0; i < 18; ++i) {
            if (i == 8 || i == 14) {
                // Altitude: tight (altimeter available immediately)
                Sigma0(i, i) = 5.0 * 5.0;  // 5m std dev
            } else if (i == 9 || i == 10 || i == 15 || i == 16) {
                // Roll/Pitch: MODERATE (allow homography to constrain naturally)
                Sigma0(i, i) = 0.02 * 0.02;  // 0.02 rad (~1°) - moderate uncertainty
            } else if (i == 11 || i == 17) {
                // Yaw: moderate (GPS heading gives good initial estimate)
                Sigma0(i, i) = 0.5 * 0.5;  // 0.5 rad std dev (~30 degrees)
            } else if (i < 6) {
                // Velocities: TIGHT initial (drone starts nearly stationary)
                Sigma0(i, i) = 1.0 * 1.0;  // 1 m/s or rad/s std dev
            } else if (i == 6 || i == 7 || i == 12 || i == 13) {
                // Horizontal position: TIGHT (we know we start at origin!)
                Sigma0(i, i) = 10.0 * 10.0;  // 10m std dev (not 1000m!)
            }
        }
        
        std::println("Initializing state with:");
        std::println("  x0(8) = {:.2f} m (current D)", x0(8));
        std::println("  x0(10) = {:.3f} rad = {:.1f}° (current pitch - look down)", x0(10), x0(10)*180.0/M_PI);
        std::println("  x0(11) = {:.3f} rad = {:.1f}° (current yaw from GPS)", x0(11), x0(11)*180.0/M_PI);
        std::println("  x0(14) = {:.2f} m (delayed D)", x0(14));
        std::println("  x0(16) = {:.3f} rad = {:.1f}° (delayed pitch)", x0(16), x0(16)*180.0/M_PI);
        std::println("  x0(17) = {:.3f} rad = {:.1f}° (delayed yaw)", x0(17), x0(17)*180.0/M_PI);
        std::println("Prior uncertainties:");
        std::println("  σ_altitude = {:.2f} m", std::sqrt(Sigma0(8,8)));
        std::println("  σ_roll/pitch = {:.3f} rad = {:.1f}°", std::sqrt(Sigma0(9,9)), std::sqrt(Sigma0(9,9))*180.0/M_PI);
        std::println("  σ_yaw = {:.3f} rad = {:.1f}°", std::sqrt(Sigma0(11,11)), std::sqrt(Sigma0(11,11))*180.0/M_PI);
        
        system.density = GaussianInfo<double>::fromMoment(x0, Sigma0);
        
        // Verify initialization
        Eigen::VectorXd check_mean = system.density.mean();
        std::println("Verification: mean(8) = {:.2f}, mean(11) = {:.3f}, mean(14) = {:.2f}, mean(17) = {:.3f}", 
                     check_mean(8), check_mean(11), check_mean(14), check_mean(17));
        
        std::println("Initial GPS altitude: {:.2f} m", h0);
        std::println("Initial AGL (h - 7m): {:.2f} m", ga0);
        std::println("State vector dimension: {}", system.density.dim());
        std::println("\nProcessing video frames...");
        
        // Event storage
        std::vector<std::shared_ptr<Event>> events;
        
        // Frame skipping for longer flow vectors
        const int imgModulus = 10;  // Process every 10th frame for longer vectors
        
        // Previous frame data
        cv::Mat imgkm1_raw;
        Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1;
        Eigen::Vector6d etakm1 = x0.segment<6>(6);
        double prev_altitude = captions[0].altitude - 7.0;
        
        // GPS validation tracking
        double sum_sq_errors = 0.0;
        int error_count = 0;
        double pos_error = 0.0;
        double rms_error = 0.0;
        std::println("Initial GPS: lat={:.6f}, lon={:.6f}, alt={:.2f}", lat0, lon0, h0);
        
        // Trajectory tracking for visualization (estimated path only)
        std::vector<Eigen::Vector2d> est_path;
        
        int k = 0;  // Frame counter for optical flow
        for (int i = 0;; ++i)
        {
            // Get next input frame
            cv::Mat imgk_raw = bufferedVideoReader.read();
            if (imgk_raw.empty())
            {
                break;  // End of video
            }
            
            // Frame skipping - only process every imgModulus-th frame
            if (i % imgModulus != 0)
            {
                continue;
            }
            
            double time = i / fps;
            
            // Skip first frame - just initialize
            if (k == 0)
            {
                imgk_raw.copyTo(imgkm1_raw);
                k++;
                continue;
            }
            
            // Create measurement events
            events.clear();
            
            // CRITICAL: Per lecture, ALTIMETER must be processed FIRST for Scenario 4
            // to resolve speed/altitude ambiguity BEFORE flow update
            
            // Altimeter measurement (always on first frame, then only when value changes)
            double current_altitude = captions[i].altitude - 7.0;
            
            // Process altimeter update (only when value changes to avoid over-constraining)
            if (k == 1 || std::abs(current_altitude - prev_altitude) > 0.01)
            {
                // Altimeter sigma: WEAK trust (10.0m = very uncertain, lets flow dominate)
                auto altMeas = std::make_shared<MeasurementAltimeter>(time, current_altitude, 100.0);
                altMeas->process(system);
                system.setLastEventWasFlow(false);  // CRITICAL: Set AFTER process (for NEXT time update)
                prev_altitude = current_altitude;
            }
            
            // Optical flow measurement (AFTER altimeter to use resolved altitude)
            auto flowMeas = std::make_shared<MeasurementOutdoorFlowBundle>(
                time, camera, imgk_raw, imgkm1_raw, rQOikm1);
            
            // CRITICAL: Set flag BEFORE process() so predict() reads it!
            system.setLastEventWasFlow(true);
            flowMeas->process(system);
            rQOikm1 = flowMeas->trackedCurrentFeatures();
            
            // Store for visualization
            events.push_back(flowMeas);
            
            // Get current pose for visualization and debugging
            Eigen::VectorXd full_state = system.density.mean();
            Eigen::Vector6d etak = full_state.segment<6>(6);  // eta (6-11)
            
            // ORIENTATION DEBUG: Track roll/pitch drift every 50 frames
            if (i % 50 == 0) {
                double roll_deg = etak(3) * 180.0 / M_PI;
                double pitch_deg = etak(4) * 180.0 / M_PI;
                double yaw_deg = etak(5) * 180.0 / M_PI;
                std::println("[Frame {}] Orientation: R={:.2f}° P={:.2f}° Y={:.2f}° | Alt={:.1f}m", 
                           i, roll_deg, pitch_deg, yaw_deg, -etak(2));
                
                // Warn if roll/pitch getting large (horizon will tilt!)
                if (std::abs(roll_deg) > 5.0 || std::abs(pitch_deg) > 5.0) {
                    std::println("  ⚠️  ORIENTATION DRIFT DETECTED! Roll/Pitch > 5°");
                }
            }
            
            // GPS validation - convert GPS to NED and compute errors
            Eigen::Vector3d gps_ned = latLonToNED(captions[i].latitude, captions[i].longitude, 
                                                   captions[i].altitude, lat0, lon0, h0);
            
            // Compute position error (horizontal only)
            double est_north = etak(0);
            double est_east = etak(1);
            double pos_error = std::sqrt(std::pow(est_north - gps_ned(0), 2) + 
                                        std::pow(est_east - gps_ned(1), 2));
            
            // Track positions for trajectory plot (sample every 30 frames to avoid too many points)
            if (i % 30 == 0) {
                // NO GPS - only estimated path per user request
                est_path.push_back(Eigen::Vector2d(est_north, est_east));
            }
            
            // Accumulate for RMS
            sum_sq_errors += pos_error * pos_error;
            error_count++;
            double rms_error = std::sqrt(sum_sq_errors / error_count);
            
            // GPS validation tracked silently (printed in final summary only)
            
            // Create visualization
            cv::Mat imgout;
            cv::resize(imgk_raw, imgout, cv::Size(), 1.0/divisor, 1.0/divisor);
            
            // Draw visualization overlays IN ORDER per spec:
            // 1. Ground grid (2km x 2km, 100m spacing)
            plotGroundPlane(imgout, etak, camera, divisor);
            
            // 2. Horizon (red)
            plotHorizon(imgout, etak, camera, divisor);
            
            // 3. Cardinal directions on horizon
            plotCompass(imgout, etak, camera, divisor);
            
            // 4. Predicted flow vectors (BLUE) and measured flow vectors (GREEN=inliers, RED=outliers)
            if (k > 0 && events.size() > 0)
            {
                auto flowMeas = std::dynamic_pointer_cast<MeasurementOutdoorFlowBundle>(events[0]);
                if (flowMeas)
                {
                    const auto & rQOik = flowMeas->trackedCurrentFeatures();
                    const auto & rQOikm1_prev = flowMeas->trackedPreviousFeatures();
                    const auto & inliers = flowMeas->inlierMask();
                    
                    // Get predicted features for blue arrows
                    Eigen::VectorXd state = system.density.mean();
                    Eigen::Matrix<double, 2, Eigen::Dynamic> predicted = flowMeas->predictedFeatures(state, system);
                    
                    // Draw PREDICTED flow vectors in BLUE (drawn first, under measured)
                    for (int j = 0; j < predicted.cols() && j < rQOikm1_prev.cols(); ++j)
                    {
                        cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                        cv::Point2d pt_pred(predicted(0,j)/divisor, predicted(1,j)/divisor);
                        cv::arrowedLine(imgout, pt_prev, pt_pred, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);  // BLUE
                    }
                    
                    // Draw MEASURED flow vectors (GREEN=inliers, RED=outliers) on top
                    for (int j = 0; j < rQOik.cols() && j < rQOikm1_prev.cols(); ++j)
                    {
                        cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                        cv::Point2d pt_curr(rQOik(0,j)/divisor, rQOik(1,j)/divisor);
                        
                        if (j < (int)inliers.size() && inliers[j])
                        {
                            cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);  // GREEN
                        }
                        else
                        {
                            cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // RED
                        }
                    }
                }
            }
            
            // 5. Circular marker for body translational velocity (ORANGE)
            if (k > 0)
            {
                plotVelocityMarker(imgout, etak, etakm1, camera, divisor);
            }
            
            // 6. Text overlays (altitude and position)
            plotStateText(imgout, etak, gps_ned, current_altitude);
            
            // 7. Trajectory plot in top-right corner (estimated path only)
            plotTrajectory(imgout, est_path);
            
            // Display with proper interactive mode handling
            // i=0: Show window, no pause
            // i=1: Show window, pause only on last frame
            // i=2: Show window, pause on every frame
            cv::imshow("Scenario 4: Outdoor Flight", imgout);
            
            // Check if this is likely the last processed frame (within frame skip window of end)
            bool isLastFrame = (i >= nFrames - imgModulus);
            bool shouldPause = (interactive == 2) || (interactive == 1 && isLastFrame);
            
            // waitKey(0) pauses until keypress, waitKey(1) continues after 1ms
            int waitTime = shouldPause ? 0 : 1;
            char key = cv::waitKey(waitTime);
            
            if (key == 'q')
            {
                std::println("Key 'q' pressed. Terminating.");
                break;
            }
            
            // Write output frame
            if (doExport)
            {
                bufferedVideoWriter.write(imgout);
            }
            
            // Update for next iteration
            imgk_raw.copyTo(imgkm1_raw);
            etakm1 = etak;
            k++;
        }
        
        std::println("\nScenario 4 complete. {} frames processed", k);
    }
    else if (scenario == 5)
    {
        std::println("\n=== SCENARIO 5: INDOOR + IDENTICAL TAGS ===");
        
        // Initialize 18-state SystemVisualNav (with delayed pose, no landmarks initially)
        std::println("Initializing 18-state SystemVisualNav (nu + eta + zeta)...");
        auto p0 = GaussianInfo<double>::fromSqrtInfo(Eigen::VectorXd::Zero(18), Eigen::MatrixXd::Zero(18, 18));
        SystemVisualNav system(p0);
        system.setScenario(5);  // Set scenario for process noise tuning
        
        // Set initial pose and velocity - CAMERA STARTS AT ORIGIN (0,0,0)!
        Eigen::VectorXd x0(18);
        x0.setZero();
        // No initial velocity - start stationary
        // Position (6-8): All zero - camera at origin
        // Orientation (9-11): Small pitch down, zero roll/yaw
        x0(10) = -0.1;  // Current pitch: slight down (~5°)
        x0(16) = -0.1;  // Delayed pitch: same (CRITICAL for first frame!)
        
        // EXACTLY A1 Scenario 1 initialization (square-root form)
        Eigen::MatrixXd S0(18, 18);
        S0.setZero();
        
        // Velocities (0-5): [vN,vE,vD, wx,wy,wz]
        S0.topLeftCorner(6, 6).diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01;
        
        // Current pose (6-11): [rN,rE,rD, phi,theta,psi]
        // TIGHTER orientation constraints: 0.01 rad (~0.6°) for roll/pitch, 0.1 rad (~5.7°) for yaw
        S0.block<6,6>(6, 6).diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.1;
        
        // Delayed pose (12-17): same as current
        S0.block<6,6>(12, 12).diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.1;
        
        // Convert sqrt cov to full cov
        Eigen::MatrixXd Sigma0 = S0 * S0.transpose();
        
        system.density = GaussianInfo<double>::fromMoment(x0, Sigma0);
        
        std::println("Initial state dimension: {}", system.density.dim());
        std::println("Processing video frames...");
        
        // Previous frame data
        cv::Mat imgkm1_raw;
        Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1;
        
        // Trajectory tracking for visualization
        std::vector<Eigen::Vector2d> est_path;
        
        const double tagSize = 0.15;  // 15cm AprilTags
        
        // Frame skipping for longer flow vectors - INCREASED to get larger motion between frames
        const int imgModulus = 6;  // Skip 5 frames (process every 6th) for larger flow magnitudes
        
        int k = 0;
        for (int i = 0;; ++i)
        {
            cv::Mat imgk_raw = bufferedVideoReader.read();
            if (imgk_raw.empty())
            {
                std::println("[Frame {}] Empty frame - ending", i);
                
                // Pause on last frame if interactive=1
                if (interactive == 1)
                {
                    cv::waitKey(0);
                }
                break;
            }
            
            // Frame skipping - only process every imgModulus-th frame
            if (i % imgModulus != 0)
            {
                continue;
            }
            
            double time = i / fps;
            
            if (k == 0)
            {
                std::println("[Frame {}] First frame - initializing (k={})", i, k);
                imgk_raw.copyTo(imgkm1_raw);
                
                // CRITICAL: Initialize system time so first dt calculation works!
                // Process dummy event at time=0 to set initial system time  
                auto dummyFlow = std::make_shared<MeasurementIndoorFlowBundle>(
                    0.0, camera, imgk_raw, imgk_raw, rQOikm1);  // dummy flow at t=0
                dummyFlow->process(system);  // Sets system time to 0.0
                system.setLastEventWasFlow(true);  // Mark as flow event
                
                k++;
                continue;
            }
            
            // CRITICAL EVENT ORDER (PDF Slide 8): FLOW FIRST, THEN TAGS
            
            // 1. Flow measurement FIRST (reduces landmark uncertainty for data association)
            auto flowMeas = std::make_shared<MeasurementIndoorFlowBundle>(
                time, camera, imgk_raw, imgkm1_raw, rQOikm1);
            
            system.setLastEventWasFlow(true);  // CRITICAL: Set BEFORE process()!
            flowMeas->process(system);
            rQOikm1 = flowMeas->trackedCurrentFeatures();
            
            // 2. Tag measurement SECOND (benefits from reduced uncertainty)
            auto tagMeas = std::make_shared<MeasurementIdenticalTagBundle>(time, imgk_raw, camera, tagSize);
            
            // PDF Slide 9: ALL map management (association, deletion, initialization) in update()!
            tagMeas->update(system);  // Clean! No spaghetti!
            system.setLastEventWasFlow(false);
            
            // Camera state after tags
            Eigen::VectorXd state = system.density.mean();
            
            // Track position for trajectory plot (every 10 frames)
            if (i % 10 == 0)
            {
                double est_north = state(6);  // North position
                double est_east = state(7);   // East position
                est_path.push_back(Eigen::Vector2d(est_north, est_east));
            }
            
            // Visualization
            cv::Mat imgout;
            cv::resize(imgk_raw, imgout, cv::Size(), 1.0/divisor, 1.0/divisor);
            
            // Draw flow vectors (BLUE=predicted, GREEN=inlier, RED=outlier)
            if (k > 1)
            {
                const auto & rQOik = flowMeas->trackedCurrentFeatures();
                const auto & rQOikm1_prev = flowMeas->trackedPreviousFeatures();
                const auto & inliers = flowMeas->inlierMask();
                
                Eigen::Matrix<double, 2, Eigen::Dynamic> predicted = flowMeas->predictedFeatures(state, system);
                
                // Draw PREDICTED flow vectors in BLUE (drawn first, under measured)
                for (int j = 0; j < predicted.cols() && j < rQOikm1_prev.cols(); ++j)
                {
                    cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                    cv::Point2d pt_pred(predicted(0,j)/divisor, predicted(1,j)/divisor);
                    cv::arrowedLine(imgout, pt_prev, pt_pred, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);  // BLUE
                }
                
                // Draw MEASURED flow vectors (GREEN=inliers, RED=outliers) on top
                for (int j = 0; j < rQOik.cols() && j < rQOikm1_prev.cols(); ++j)
                {
                    cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                    cv::Point2d pt_curr(rQOik(0,j)/divisor, rQOik(1,j)/divisor);
                    
                    if (j < (int)inliers.size() && inliers[j])
                    {
                        cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);  // GREEN
                    }
                    else
                    {
                        cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);  // RED
                    }
                }
            }
            
            // Draw detected tags with OpenCV ArUco drawing
            const auto & tagCorners = tagMeas->detectedCorners();
            
            if (!tagCorners.empty()) {
                // Scale corners for display
                std::vector<std::vector<cv::Point2f>> cornersScaled;
                for (const auto & corners : tagCorners) {
                    std::vector<cv::Point2f> scaled;
                    for (const auto & pt : corners) {
                        scaled.push_back(cv::Point2f(pt.x/divisor, pt.y/divisor));
                    }
                    cornersScaled.push_back(scaled);
                }
                
                // Create dummy IDs (all same since identical tags)
                std::vector<int> dummyIds(tagCorners.size(), 0);
                
                // Use OpenCV's built-in ArUco marker drawing
                cv::aruco::drawDetectedMarkers(imgout, cornersScaled, dummyIds);
            }
            
            // Text overlay
            double est_north = state(6);
            double est_east = state(7);
            double est_up = -state(8);
            
            std::string coord_text = std::format("Est: N={:.1f} E={:.1f} Up={:.1f} m", est_north, est_east, est_up);
            std::string num_landmarks_text = std::format("Landmarks: {}", system.numberLandmarks());
            
            cv::putText(imgout, coord_text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);  // BLUE like S4
            cv::putText(imgout, num_landmarks_text, cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            
            // Trajectory plot in top-right corner (just like Scenario 4!)
            plotTrajectory(imgout, est_path);
            
            // Display
            cv::imshow("Scenario 5: Indoor + Identical Tags", imgout);
            
            bool isLastFrame = (i >= nFrames - 1);
            bool shouldPause = (interactive == 2) || (interactive == 1 && isLastFrame);
            int waitTime = shouldPause ? 0 : 1;
            char key = cv::waitKey(waitTime);
            
            if (key == 'q')
            {
                std::println("Key 'q' pressed. Terminating.");
                break;
            }
            
            if (doExport)
            {
                bufferedVideoWriter.write(imgout);
            }
            
            imgk_raw.copyTo(imgkm1_raw);
            k++;
        }
        
        std::println("\nScenario 5 complete. {} frames processed", k);
    }
    else if (scenario == 6)
    {
        std::println("\n=== SCENARIO 6: INDOOR ROOM (CONV EX) ===");
        
        // Initialize 18-state SystemVisualNav
        std::println("Initializing 18-state SystemVisualNav...");
        auto p0 = GaussianInfo<double>::fromSqrtInfo(Eigen::VectorXd::Zero(18), Eigen::MatrixXd::Zero(18, 18));
        SystemVisualNav system(p0);
        system.setScenario(6);  // Set scenario for process noise tuning
        
        // Initialize 3D visualization (Plot class)
        Plot plot(camera);
        
        // Initial state
        Eigen::VectorXd x0(18);
        x0.setZero();
        x0(0) = 0.1;    // Initial velocity
        x0(10) = -0.1;  // Current pitch
        x0(16) = -0.1;  // Delayed pitch
        
        Eigen::MatrixXd Sigma0 = Eigen::MatrixXd::Identity(18, 18);
        for (int i = 0; i < 18; ++i) {
            if (i < 6) {
                Sigma0(i, i) = 0.5 * 0.5;
            } else if (i < 12) {
                Sigma0(i, i) = 5.0 * 5.0;
            } else {
                Sigma0(i, i) = 5.0 * 5.0;
            }
        }
        
        system.density = GaussianInfo<double>::fromMoment(x0, Sigma0);
        
        // Previous frame data
        cv::Mat imgkm1_raw;
        Eigen::Matrix<double, 2, Eigen::Dynamic> rQOikm1;
        
        int k = 0;
        for (int i = 0;; ++i)
        {
            cv::Mat imgk_raw = bufferedVideoReader.read();
            if (imgk_raw.empty())
            {
                std::println("[Frame {}] Empty frame - ending", i);
                
                // Pause on last frame if interactive=1
                if (interactive == 1)
                {
                    cv::waitKey(0);
                }
                break;
            }
            
            double time = i / fps;
            
            if (k == 0)
            {
                std::println("[Frame {}] First frame - initializing", i);
                imgk_raw.copyTo(imgkm1_raw);
                
                // Initialize system time
                auto dummyFlow = std::make_shared<MeasurementIndoorFlowBundle>(
                    0.0, camera, imgk_raw, imgk_raw, rQOikm1);
                dummyFlow->process(system);
                system.setLastEventWasFlow(true);
                
                k++;
                continue;
            }
            
            std::println("[Frame {}] Processing...", i);
            
            // Flow measurement FIRST
            auto flowMeas = std::make_shared<MeasurementIndoorFlowBundle>(
                time, camera, imgk_raw, imgkm1_raw, rQOikm1);
            flowMeas->process(system);
            system.setLastEventWasFlow(true);
            rQOikm1 = flowMeas->trackedCurrentFeatures();
            
            // Point SLAM SECOND
            auto pointMeas = std::make_shared<MeasurementPointBundle>(time, imgk_raw, camera);
            pointMeas->update(system);  // update() handles map management
            system.setLastEventWasFlow(false);
            
            // Simple visualization for now
            cv::Mat imgout;
            cv::resize(imgk_raw, imgout, cv::Size(), 1.0/divisor, 1.0/divisor);
            
            // Draw flow vectors
            if (k > 1)
            {
                const auto & rQOik = flowMeas->trackedCurrentFeatures();
                const auto & rQOikm1_prev = flowMeas->trackedPreviousFeatures();
                const auto & inliers = flowMeas->inlierMask();
                
                Eigen::VectorXd state = system.density.mean();
                Eigen::Matrix<double, 2, Eigen::Dynamic> predicted = flowMeas->predictedFeatures(state, system);
                
                // Blue predicted
                for (int j = 0; j < predicted.cols() && j < rQOikm1_prev.cols(); ++j)
                {
                    cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                    cv::Point2d pt_pred(predicted(0,j)/divisor, predicted(1,j)/divisor);
                    cv::arrowedLine(imgout, pt_prev, pt_pred, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
                }
                
                // Green/Red measured
                for (int j = 0; j < rQOik.cols() && j < rQOikm1_prev.cols(); ++j)
                {
                    cv::Point2d pt_prev(rQOikm1_prev(0,j)/divisor, rQOikm1_prev(1,j)/divisor);
                    cv::Point2d pt_curr(rQOik(0,j)/divisor, rQOik(1,j)/divisor);
                    
                    if (j < (int)inliers.size() && inliers[j])
                    {
                        cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                    }
                    else
                    {
                        cv::arrowedLine(imgout, pt_prev, pt_curr, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                    }
                }
            }
            
            // Draw detected point features
            const auto & features = pointMeas->detectedFeatures();
            for (const auto & pt : features)
            {
                cv::circle(imgout, cv::Point2d(pt.x/divisor, pt.y/divisor), 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            }
            
            // Text overlay
            Eigen::VectorXd state = system.density.mean();
            std::string text = std::format("Landmarks: {}", system.numberLandmarks());
            cv::putText(imgout, text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            
            // Update 3D visualization in ONE window (split-screen: left=image, right=3D)
            system.view() = imgout;  // Store camera image with overlays
            plot.setData(system, *pointMeas);
            plot.render();  // This creates split-screen window automatically
            
            bool isLastFrame = (i >= nFrames - 1);
            bool shouldPause = (interactive == 2) || (interactive == 1 && isLastFrame);
            int waitTime = shouldPause ? 0 : 1;
            char key = cv::waitKey(waitTime);
            
            if (key == 'q')
            {
                std::println("Key 'q' pressed. Terminating.");
                break;
            }
            
            if (doExport)
            {
                bufferedVideoWriter.write(imgout);
            }
            
            imgk_raw.copyTo(imgkm1_raw);
            k++;
        }
        
        std::println("\nScenario 6 complete. {} frames processed", k);
    }
    else
    {
        std::println("Scenario {} not yet implemented", scenario);
    }

    if (doExport)
    {
         bufferedVideoWriter.stop();
    }
    bufferedVideoReader.stop();
}

// Helper function implementations
Eigen::Vector6d getBodyPose(const SystemVisualNav & system)
{
    return system.density.mean().segment<6>(6);  // Extract eta (position + orientation)
}

void plotGroundPlane(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor)
{
    Pose<double> Tnb(etak);
    
    // Draw East gridlines (constant E, varying N) - 4km x 4km @ 100m spacing (EXTENDED!)
    for (int e = -2000; e <= 2000; e += 100)
    {
        std::vector<cv::Point2d> linePoints;
        for (int n = -2000; n <= 2000; n += 10)
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
            cv::line(img, linePoints[i-1], linePoints[i], cv::Scalar(0, 0, 0), 1, cv::LINE_AA);  // BLACK (thin)
        }
    }
    
    // Draw North gridlines (constant N, varying E) - 4km x 4km @ 100m spacing (EXTENDED!)
    for (int n = -2000; n <= 2000; n += 100)
    {
        std::vector<cv::Point2d> linePoints;
        for (int e = -2000; e <= 2000; e += 10)
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
            cv::line(img, linePoints[i-1], linePoints[i], cv::Scalar(0, 0, 0), 1, cv::LINE_AA);  // BLACK (thin)
        }
    }
}

void plotHorizon(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor)
{
    Eigen::Vector3d rpy = etak.tail<3>();
    Eigen::Matrix3d Rnb = rpy2rot(rpy);
    Eigen::Matrix3d Rbc = camera.Tbc.rotationMatrix;
    Eigen::Matrix3d Rnc = Rnb * Rbc;
    Eigen::Matrix3d Rnc_T = Rnc.transpose();
    
    std::vector<cv::Point2d> horizonPoints;
    
    for (double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/60.0)
    {
        Eigen::Vector3d dir_n(cos(theta), sin(theta), 0.0);
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

void plotCompass(cv::Mat & img, const Eigen::Vector6d & etak, const Camera & camera, int divisor)
{
    Eigen::Vector3d rpy = etak.tail<3>();
    Eigen::Matrix3d Rnb = rpy2rot(rpy);
    Eigen::Matrix3d Rbc = camera.Tbc.rotationMatrix;
    Eigen::Matrix3d Rnc = Rnb * Rbc;
    Eigen::Matrix3d Rnc_T = Rnc.transpose();
    
    const char* directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    double angles[] = {0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -3*M_PI/4, -M_PI/2, -M_PI/4};
    
    for (int i = 0; i < 8; ++i)
    {
        Eigen::Vector3d dir_n(cos(angles[i]), sin(angles[i]), 0.0);
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

void plotEpipole(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector6d & etakm1, const Camera & camera, int divisor)
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

void plotVelocityMarker(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector6d & etakm1, const Camera & camera, int divisor)
{
    // Compute velocity from position difference
    Eigen::Vector3d rBNn_k = etak.head<3>();
    Eigen::Vector3d rBNn_km1 = etakm1.head<3>();
    Eigen::Vector3d velocity_approx = rBNn_k - rBNn_km1;
    
    // Only draw if drone has moved (norm > 0.001 m)
    if (velocity_approx.norm() > 0.001)
    {
        Pose<double> Tnb(etak);
        Eigen::Vector3d vel_scaled = velocity_approx * 1000;  // Scale for visibility
        cv::Vec3d vel_c(vel_scaled(0), vel_scaled(1), vel_scaled(2));
        vel_c = camera.worldToVector(vel_c, Tnb);
        
        if (camera.isVectorWithinFOV(vel_c))
        {
            cv::Vec2d pixel = camera.vectorToPixel(vel_c);
            cv::Point2d pt(pixel[0] / divisor, pixel[1] / divisor);
            
            // Draw ORANGE circular marker (BGR: 0, 165, 255)
            cv::circle(img, pt, 20, cv::Scalar(0, 165, 255), -1, cv::LINE_AA);  // Filled circle
            cv::circle(img, pt, 20, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);       // Black outline
            cv::circle(img, pt, 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA); // White center dot
        }
    }
}

void plotStateText(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector3d & gps_ned, double current_altitude)
{
    // Extract estimated states
    double est_north = etak(0);   // N position
    double est_east = etak(1);    // E position
    double est_up = -etak(2);     // Up (negative of D component)
    
    // Extract GPS NED values (for validation display)
    double meas_north = gps_ned(0);  // GPS North in NED (relative displacement from start)
    double meas_east = gps_ned(1);   // GPS East in NED (relative displacement from start)
    double meas_up = current_altitude;  // GPS actual altitude (AGL with -7m offset already applied)
    
    // DEBUG: Print to console to solve sign mystery
    static int debug_count = 0;
    if (debug_count++ % 10 == 0) {  // Print every 10th frame
        std::println("[DEBUG plotStateText] etak(0)={:.1f}, display est_north={:.1f}", etak(0), est_north);
    }
    
    // Format text - all N,E,Up on one line each
    std::string est_text = std::format("Est:  N={:.1f} E={:.1f} Up={:.1f} m", est_north, est_east, est_up);
    std::string meas_text = std::format("Meas: N={:.1f} E={:.1f} Up={:.1f} m", meas_north, meas_east, meas_up);
    
    int x = 10;
    int y_start = 30;
    int line_height = 30;
    double font_scale = 0.6;
    int thickness = 2;
    
    // Draw text overlays - only 2 lines now
    cv::putText(img, est_text, cv::Point(x, y_start), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);  // BLUE
    
    cv::putText(img, meas_text, cv::Point(x, y_start + line_height), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);  // GREEN
}

Eigen::Vector3d latLonToNED(double lat, double lon, double alt, double lat0, double lon0, double alt0)
{
    // WGS84 parameters
    constexpr double a = 6378137.0;           // Earth radius (m)
    constexpr double f = 1.0 / 298.257223563; // Flattening
    constexpr double e2 = 2.0 * f - f * f;    // Eccentricity squared
    
    // Convert to radians
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double lat0_rad = lat0 * M_PI / 180.0;
    double lon0_rad = lon0 * M_PI / 180.0;
    
    // Radius of curvature
    double N0 = a / std::sqrt(1.0 - e2 * std::sin(lat0_rad) * std::sin(lat0_rad));
    
    // Small angle approximation for local tangent plane (valid for < 100km)
    double dLat = lat_rad - lat0_rad;
    double dLon = lon_rad - lon0_rad;
    
    // NED coordinates
    double north = dLat * (N0 + alt0);
    double east = dLon * (N0 + alt0) * std::cos(lat0_rad);
    double down = -(alt - alt0);
    
    return Eigen::Vector3d(north, east, down);
}

void plotGPSValidation(cv::Mat & img, const Eigen::Vector6d & etak, const Eigen::Vector3d & gps_ned, double pos_error, double rms_error)
{
    // Display GPS validation metrics in YELLOW (highly visible)
    std::string pos_err_text = std::format("GPS Pos Error: {:.2f} m", pos_error);
    std::string rms_err_text = std::format("GPS RMS Error: {:.2f} m", rms_error);
    std::string gps_north_text = std::format("GPS North: {:.2f} m", gps_ned(0));
    std::string gps_east_text = std::format("GPS East: {:.2f} m", gps_ned(1));
    
    int x = 10;
    int y_start = 150;  // Below state text
    int line_height = 30;
    double font_scale = 0.7;
    int thickness = 2;
    
    // Draw validation metrics in YELLOW (BGR: 0, 255, 255)
    cv::putText(img, pos_err_text, cv::Point(x, y_start), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), thickness, cv::LINE_AA);
    
    cv::putText(img, rms_err_text, cv::Point(x, y_start + line_height), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255), thickness, cv::LINE_AA);
    
    cv::putText(img, gps_north_text, cv::Point(x, y_start + 2*line_height), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
    
    cv::putText(img, gps_east_text, cv::Point(x, y_start + 3*line_height), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
}

void plotTrajectory(cv::Mat & img, const std::vector<Eigen::Vector2d> & est_path)
{
    // Inset plot parameters
    const int plot_size = 150;  // Made smaller (was 300)
    const int margin = 10;
    const int plot_x = img.cols - plot_size - margin;
    const int plot_y = margin;
    
    // Create semi-transparent background
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, cv::Point(plot_x, plot_y), 
                  cv::Point(plot_x + plot_size, plot_y + plot_size),
                  cv::Scalar(40, 40, 40), -1);
    cv::addWeighted(overlay, 0.7, img, 0.3, 0, img);
    
    // Draw border
    cv::rectangle(img, cv::Point(plot_x, plot_y), 
                  cv::Point(plot_x + plot_size, plot_y + plot_size),
                  cv::Scalar(255, 255, 255), 2);
    
    if (est_path.empty())
        return;
    
    // Find bounds of trajectory
    double min_north = std::numeric_limits<double>::max();
    double max_north = std::numeric_limits<double>::lowest();
    double min_east = std::numeric_limits<double>::max();
    double max_east = std::numeric_limits<double>::lowest();
    
    for (const auto & pt : est_path) {
        min_north = std::min(min_north, pt(0));
        max_north = std::max(max_north, pt(0));
        min_east = std::min(min_east, pt(1));
        max_east = std::max(max_east, pt(1));
    }
    
    // Add margins
    double range_north = max_north - min_north;
    double range_east = max_east - min_east;
    double range = std::max(range_north, range_east);
    if (range < 1.0) range = 1.0;  // Minimum 1m range
    
    range *= 1.2;  // 20% margin
    double center_north = (min_north + max_north) / 2.0;
    double center_east = (min_east + max_east) / 2.0;
    
    // Scale factor (pixels per meter)
    double scale = (plot_size - 40) / range;  // Leave 20px margin on each side
    
    // Helper lambda to convert NE coordinates to pixel coordinates
    // Standard: X=East (horizontal), Y=North (vertical, inverted)
    auto toPixel = [&](double north, double east) -> cv::Point {
        int px = plot_x + 20 + static_cast<int>((east - (center_east - range/2)) * scale);
        int py = plot_y + plot_size - 20 - static_cast<int>((north - (center_north - range/2)) * scale);
        return cv::Point(px, py);
    };
    
    // Draw grid lines
    for (int i = 0; i <= 4; ++i) {
        double frac = i / 4.0;
        double north_val = center_north - range/2 + frac * range;
        double east_val = center_east - range/2 + frac * range;
        
        // Horizontal lines (constant North value)
        cv::Point pt1 = toPixel(north_val, center_east - range/2);
        cv::Point pt2 = toPixel(north_val, center_east + range/2);
        cv::line(img, pt1, pt2, cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
        
        // Vertical lines (constant East value)
        cv::Point pt3 = toPixel(center_north - range/2, east_val);
        cv::Point pt4 = toPixel(center_north + range/2, east_val);
        cv::line(img, pt3, pt4, cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    }
    
    // Draw estimated path (CYAN for visibility)
    for (size_t i = 1; i < est_path.size(); ++i) {
        cv::Point pt1 = toPixel(est_path[i-1](0), est_path[i-1](1));
        cv::Point pt2 = toPixel(est_path[i](0), est_path[i](1));
        cv::line(img, pt1, pt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    }
    
    // Draw current position marker
    if (!est_path.empty()) {
        cv::Point est_curr = toPixel(est_path.back()(0), est_path.back()(1));
        cv::circle(img, est_curr, 7, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
        cv::circle(img, est_curr, 7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    
    // Draw axis labels (N on left for Y-axis, E at bottom for X-axis)
    cv::putText(img, "N", cv::Point(plot_x + 5, plot_y + plot_size/2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(img, "E", cv::Point(plot_x + plot_size/2 - 10, plot_y + plot_size - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    
    // Draw title
    cv::putText(img, "Est Path", cv::Point(plot_x + 10, plot_y + plot_size - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}
