#include <filesystem>
#include <string>
#include <iostream>
#include <cassert>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "BufferedVideo.h"
#include "Camera.h"
#include "serialisation.hpp"
#include "visualNavigation.h"
#include "GaussianInfo.hpp"
#include "SystemSLAMPoseLandmarks.h"
#include "SystemSLAMPointLandmarks.h"
#include "Plot.h"
#include <opencv2/aruco.hpp>
#include <algorithm>
#include "MeasurementSLAMUniqueTagBundle.h"
#include "MeasurementSLAMDuckBundle.h"
#include "MeasurementDummy.h"
#include "imagefeatures.h"
#include "camera_states_plot.h"
#include "DuckDetectorONNX.h"
#include "DuckDetectionCache.h"
#include <filesystem>
    
void runVisualNavigationFromVideo(const std::filesystem::path & videoPath, const std::filesystem::path & cameraPath, int scenario, int interactive, const std::filesystem::path & outputDirectory)
{
    assert(!videoPath.empty());

    // Output video path
    std::filesystem::path outputPath;
    bool doExport = !outputDirectory.empty();
    if (doExport)
    {
        std::string outputFilename = videoPath.stem().string()
                                   + "_out"
                                   + videoPath.extension().string();
        outputPath = outputDirectory / outputFilename;
    }

    // Load camera calibration - different paths for different scenarios
    Camera camera;
    std::filesystem::path actualCameraPath = cameraPath;
    if (scenario == 2) {
        // Indoor camera for duck scenario
        actualCameraPath = std::filesystem::path("data/indoor/camera.xml");
        if (!std::filesystem::exists(actualCameraPath)) {
            actualCameraPath = cameraPath;  // fallback to provided path
        }
    }

    cv::FileStorage fsc(actualCameraPath.string(), cv::FileStorage::READ);
    camera.read(fsc["camera"]);   // call the member read() on the "camera" node
    fsc.release();

    // Set camera extrinsics (camera in body frame)
    // Camera is 0.2m forward of body center, no rotation
    camera.Tbc.translationVector << 0.2, 0.0, 0.0;
    camera.Tbc.rotationMatrix.setIdentity();

    // Display loaded calibration data
    camera.printCalibration();

    // Open input video
    cv::VideoCapture cap(videoPath.string());
    assert(cap.isOpened());
    int nFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    assert(nFrames > 0);
    double fps = cap.get(cv::CAP_PROP_FPS);

    BufferedVideoReader bufferedVideoReader(5);
    bufferedVideoReader.start(cap);

    cv::VideoWriter videoOut;
    BufferedVideoWriter bufferedVideoWriter(3);
    if (doExport)
    {
        cv::Size frameSize;
        frameSize.width     = 2*cap.get(cv::CAP_PROP_FRAME_WIDTH);
        frameSize.height    = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double outputFps    = fps;
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // manually specify output video codec
        videoOut.open(outputPath.string(), codec, outputFps, frameSize);
        bufferedVideoWriter.start(videoOut);
    }

    // Visual navigation

    // Initialisation
    // Initial state: 12 DOF (camera velocities + pose), 0 landmarks initially
    Eigen::VectorXd mu(12);
    mu.setZero();
    
    // Set initial yaw to π/2 (90 degrees) to align camera facing East
    // This corrects the coordinate frame if camera starts facing East instead of North
    // mu(11) = M_PI / 4.0;  // yaw (psi)

    // Initial state covariance (square-root form)
    Eigen::MatrixXd S(12, 12);
    S.setZero();

    // Set initial velocity/acceleration noise (top-left 6×6 block)
    S.topLeftCorner(6, 6).diagonal() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01;  // [vBNb, omegaBNb]

    // Set initial pose uncertainty (bottom-right 6×6 block)  
    S.bottomRightCorner(6, 6).diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;  // [rBNn, Thetanb]

    // Create initial Gaussian density
    auto p0 = GaussianInfo<double>::fromSqrtMoment(mu, S);

    // Initialize SLAM system based on scenario
    SystemSLAM* system = nullptr;
    if (scenario == 2) {
        // Scenario 2: Duck SLAM with point landmarks (3-DOF)
        system = new SystemSLAMPointLandmarks(p0);
    } else {
        // Scenario 1: ArUco tag SLAM with pose landmarks (6-DOF)
        system = new SystemSLAMPoseLandmarks(p0);
    }

    // Initialize plot/visualization
    Plot plot(camera);

    // Track marker IDs for data association (maps marker ID -> landmark index)
    std::vector<int> markerIdToLandmarkIdx;

    // Frame counter
    int frameIdx = 0;
    bool isLastFrame = false;

    // History tracking for plotting
    std::vector<double> timeHistory;
    std::vector<Eigen::VectorXd> stateHistory;
    std::vector<Eigen::VectorXd> sigmaHistory;
    
    // Optional: limit processing time (set to 0 to process entire video)
    double maxProcessingTime = 20;  // Process only first X seconds
    // double maxProcessingTime = 0.0;  // Process entire video

    while (true)
    {
        // Get next input frame
        cv::Mat imgin = bufferedVideoReader.read();
        if (imgin.empty())
        {
            break;
        }

        frameIdx++;
        isLastFrame = (frameIdx == nFrames);
        
        // Check if we've reached time limit
        double currentTime = frameIdx / fps;
        if (maxProcessingTime > 0 && currentTime >= maxProcessingTime)
        {
            std::cout << "\nReached time limit of " << maxProcessingTime << " seconds" << std::endl;
            break;
        }

        std::cout << "\n========== FRAME " << frameIdx << " ==========" << std::endl;
        std::cout << "  [DEBUG] Frame start - State dim: " << system->density.dim() 
                  << ", Landmarks: " << system->numberLandmarks() << std::endl;

        cv::Mat imgAnnotated;
        double time = frameIdx / fps;  // Calculate time from frame index

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<cv::Point2f> duckCentroids;
        std::vector<int> duckAreas;

        if (scenario == 2) {
            // Scenario 2: Duck SLAM
            
            // Try to use cached detections for performance
            static DuckDetectionCache* cache = nullptr;
            static DuckDetectorONNX* detector = nullptr;
            static bool useCache = false;
            static bool initialized = false;

            if (!initialized) {
                // Check if cache file exists
                std::filesystem::path cache_file = "../data/duck_detections.bin";
                if (std::filesystem::exists(cache_file)) {
                    try {
                        cache = new DuckDetectionCache(cache_file.string());
                        useCache = true;
                        std::cout << "Loaded duck detection cache from " << cache_file.string() << std::endl;
                        std::cout << "  Cache contains " << cache->getNumFrames() << " frames" << std::endl;
                        std::cout << "  Using pre-computed detections (FAST!)" << std::endl;
                    } catch (const std::exception& e) {
                        std::cerr << "Failed to load cache: " << e.what() << std::endl;
                        std::cerr << "Falling back to ONNX detector" << std::endl;
                        useCache = false;
                    }
                }
                
                if (!useCache) {
                    // Fall back to ONNX detector
                    std::filesystem::path onnx_file = "../data/duck_with_postprocessing.onnx";
                    detector = new DuckDetectorONNX(onnx_file.string());
                    std::cout << "Loaded ONNX duck detector from " << onnx_file.string() << std::endl;
                    std::cout << "WARNING: Running ONNX inference on every frame (SLOW!)" << std::endl;
                    std::cout << "         Consider generating cache with lab3/python/create_duck_cache.py" << std::endl;
                }
                
                initialized = true;
            }

            // Get detections from cache or detector
            if (useCache && frameIdx <= static_cast<int>(cache->getNumFrames())) {
                // Use cached detections (FAST!)
                duckCentroids = cache->getCentroids(frameIdx - 1);  // frameIdx is 1-based
                duckAreas = cache->getAreas(frameIdx - 1);
                
                // Still need annotated image for visualization
                imgAnnotated = imgin.clone();
                imgAnnotated *= 0.5;  // Darken like the detector does
            } else {
                // Fall back to ONNX detector
                if (detector == nullptr) {
                    std::filesystem::path onnx_file = "../data/duck_with_postprocessing.onnx";
                    detector = new DuckDetectorONNX(onnx_file.string());
                }
                
                imgAnnotated = detector->detect(imgin);
                duckCentroids = detector->getCentroids();
                duckAreas = detector->getAreas();
            }

        } else {
            // Scenario 1: ArUco tag SLAM
            // Process frame - Detect ArUco markers using your function
            imgAnnotated = detectAndDrawArUco(imgin, 100, markerIds, markerCorners);
        }

        // Landmark initialization for ArUco tags (Scenario 1 only)
        // Duck initialization (Scenario 2) now handled by candidate-based system below
        if (scenario == 1) {
            // Augment state with new landmarks BEFORE calling process()
            // (process() calls predict() which needs correct dimensions)
            static std::vector<int> knownMarkerIds;
            for (const int id : markerIds)
            {
                // Check if this is a new marker ID
                bool isNew = true;
                for (size_t i = 0; i < knownMarkerIds.size(); ++i)
                {
                    if (knownMarkerIds[i] == id)
                    {
                        isNew = false;
                        break;
                    }
                }

                if (isNew)
                {
                    // Get current camera orientation and position in world frame
                    Eigen::VectorXd current_state = system->density.mean();
                    Eigen::Matrix3d Rnc = SystemSLAM::cameraOrientation(camera, current_state);
                    Eigen::Vector3d rCNn = SystemSLAM::cameraPosition(camera, current_state);

                    // Find this marker's corners to get center pixel
                    size_t markerIdx = 0;
                    for (size_t j = 0; j < markerIds.size(); ++j) {
                        if (markerIds[j] == id) {
                            markerIdx = j;
                            break;
                        }
                    }

                    // Calculate marker center pixel from corners
                    const auto& corners = markerCorners[markerIdx];
                    cv::Point2f center(0, 0);
                    for (const auto& corner : corners) {
                        center += corner;
                    }
                    center *= (1.0f / corners.size());

                    // Get ray direction from center pixel
                    cv::Vec3d uPCc = camera.pixelToVector(cv::Vec2d(center.x, center.y));
                    Eigen::Vector3d ray_cam(uPCc[0], uPCc[1], uPCc[2]);

                    // Initialize landmark along ray at estimated depth
                    double initialDepth = 1.5;  // meters - initial guess (closer to typical marker distance)
                    Eigen::Vector3d ray_world = Rnc * ray_cam;
                    Eigen::Vector3d rPNn = rCNn + initialDepth * ray_world;

                    // Initialize landmark position and orientation
                    Eigen::VectorXd mu_new(6);
                    mu_new.head<3>() = rPNn;  // Position along ray
                    mu_new.tail<3>() << 0.0, 0.0, 0.0;  // Orientation (unknown)

                    // Construct covariance directly in world frame along viewing ray
                    // (Same approach as scenario 2 for consistency)
                    Eigen::Vector3d ray_unit = ray_world.normalized();
                    
                    // Manual cross product helper (avoids linker issues)
                    auto crossProduct = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> Eigen::Vector3d {
                        return Eigen::Vector3d(
                            a(1)*b(2) - a(2)*b(1),
                            a(2)*b(0) - a(0)*b(2),
                            a(0)*b(1) - a(1)*b(0)
                        );
                    };
                    
                    // Create orthogonal basis with ray as one axis
                    Eigen::Vector3d perp1, perp2;
                    if (std::abs(ray_unit(2)) < 0.9) {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(0, 0, 1)).normalized();
                    } else {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(1, 0, 0)).normalized();
                    }
                    perp2 = crossProduct(ray_unit, perp1).normalized();
                    
                    // Build rotation matrix: columns are [perp1, perp2, ray_unit]
                    Eigen::Matrix3d R_ray;
                    R_ray.col(0) = perp1;
                    R_ray.col(1) = perp2;
                    R_ray.col(2) = ray_unit;
                    
                    // Diagonal uncertainty in ray frame: small perp, large along ray
                    Eigen::Matrix3d S_ray;
                    S_ray.setZero();
                    S_ray.diagonal() << 0.05, 0.05, 0.5;  // [perp1, perp2, along_ray]
                    
                    // Rotate to world frame
                    Eigen::Matrix3d Cov_pos = R_ray * (S_ray * S_ray.transpose()) * R_ray.transpose();
                    Eigen::LLT<Eigen::Matrix3d> llt_pos(Cov_pos);
                    Eigen::Matrix3d S_pos = llt_pos.matrixU();

                    // Construct full 6x6 sqrt covariance (position + orientation)
                    Eigen::MatrixXd S_new(6, 6);
                    S_new.setZero();
                    S_new.topLeftCorner<3,3>() = S_pos;  // Position block from ray tracing
                    S_new.bottomRightCorner<3,3>().diagonal() << 0.1, 0.1, 0.5;  // Tighter orientation uncertainty

                    // Debug output
                    std::cout << "  [INIT] Marker ID " << id << " initialized at "
                             << initialDepth << "m along ray (world pos: ["
                             << rPNn(0) << ", " << rPNn(1) << ", " << rPNn(2) << "])" << std::endl;

                    auto p_new = GaussianInfo<double>::fromSqrtMoment(mu_new, S_new);
                    system->density *= p_new;

                    knownMarkerIds.push_back(id);
                }
            }
        }

        // Store processed measurement for both scenarios
        MeasurementUniqueTagBundle* processedMeasurement1 = nullptr;
        MeasurementDuckBundle* processedMeasurement2 = nullptr;
        
        // HYBRID DUCK INITIALIZATION LOGIC
        // Bootstrap phase: When no landmarks exist, initialize immediately
        // Growth phase: When landmarks exist, use candidate-based tracking
        
        // Helper function to remove farthest landmark from camera
        auto removeFarthestLandmark = [&system, &camera]() {
            if (system->numberLandmarks() == 0) return;
            
            // Get current camera position
            Eigen::Vector3d rCNn = system->cameraPositionDensity(camera).mean();
            
            // Find farthest landmark
            size_t farthestIdx = 0;
            double maxDistance = -1.0;
            
            for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                Eigen::Vector3d rLNn = system->landmarkPositionDensity(i).mean();
                double distance = (rLNn - rCNn).norm();
                
                if (distance > maxDistance) {
                    maxDistance = distance;
                    farthestIdx = i;
                }
            }
            
            std::cout << "  [PRUNE] Removing farthest landmark LM" << farthestIdx 
                      << " at distance " << maxDistance << "m" << std::endl;
            
            // Marginalize out this landmark
            size_t landmarkDim = 3;  // 3-DOF point landmarks
            size_t bodyDim = 12;     // Camera state dimension
            
            // Build index vector: keep body state + all landmarks except farthest
            std::vector<int> keepIndices;
            
            // Keep body state (first 12 elements)
            for (size_t i = 0; i < bodyDim; ++i) {
                keepIndices.push_back(i);
            }
            
            // Keep landmarks before farthest
            for (size_t i = 0; i < farthestIdx; ++i) {
                size_t landmarkStart = bodyDim + i * landmarkDim;
                for (size_t j = 0; j < landmarkDim; ++j) {
                    keepIndices.push_back(landmarkStart + j);
                }
            }
            
            // Keep landmarks after farthest
            for (size_t i = farthestIdx + 1; i < system->numberLandmarks(); ++i) {
                size_t landmarkStart = bodyDim + i * landmarkDim;
                for (size_t j = 0; j < landmarkDim; ++j) {
                    keepIndices.push_back(landmarkStart + j);
                }
            }
            
            // Marginalize
            Eigen::VectorXi keepIndicesEigen = Eigen::Map<Eigen::VectorXi>(keepIndices.data(), keepIndices.size());
            system->density = system->density.marginal(keepIndicesEigen);
        };
        
        // Landmark limit for performance
        const size_t MAX_LANDMARKS = 40;
        
        if (scenario == 2 && !duckCentroids.empty()) {
            if (system->numberLandmarks() == 0) {
                // BOOTSTRAP: Initialize first landmarks immediately (no waiting)
                std::cout << "  [BOOTSTRAP] No landmarks - initializing first " << duckCentroids.size() << " ducks immediately" << std::endl;
                
                for (size_t detIdx = 0; detIdx < duckCentroids.size(); ++detIdx) {
                    // Check landmark limit
                    if (system->numberLandmarks() >= MAX_LANDMARKS) {
                        std::cout << "  [BOOTSTRAP] Landmark limit reached - stopping initialization" << std::endl;
                        break;
                    }
                    
                    // Get current camera orientation and position
                    Eigen::VectorXd current_state = system->density.mean();
                    Eigen::Matrix3d Rnc = SystemSLAM::cameraOrientation(camera, current_state);
                    Eigen::Vector3d rCNn = SystemSLAM::cameraPosition(camera, current_state);
                    
                    // Get ray direction
                    const cv::Point2f& centroid = duckCentroids[detIdx];
                    cv::Vec3d uPCc = camera.pixelToVector(cv::Vec2d(centroid.x, centroid.y));
                    Eigen::Vector3d ray_cam(uPCc[0], uPCc[1], uPCc[2]);
                    
                    // Area-based depth estimation
                    const double fx = camera.cameraMatrix.at<double>(0, 0);
                    const double fy = camera.cameraMatrix.at<double>(1, 1);
                    const double duckRadius = 0.018;  // meters
                    const double area = duckAreas[detIdx];
                    double initialDepth = std::sqrt((fx * fy * M_PI * duckRadius * duckRadius) / area);
                    initialDepth = std::max(0.3, std::min(initialDepth, 3.0));
                    
                    // Initialize landmark position
                    Eigen::Vector3d ray_world = Rnc * ray_cam;
                    Eigen::Vector3d rPNn = rCNn + initialDepth * ray_world;
                    Eigen::VectorXd mu_new(3);
                    mu_new = rPNn;
                    
                    // Construct covariance along viewing ray
                    Eigen::Vector3d ray_unit = ray_world.normalized();
                    auto crossProduct = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> Eigen::Vector3d {
                        return Eigen::Vector3d(a(1)*b(2) - a(2)*b(1), a(2)*b(0) - a(0)*b(2), a(0)*b(1) - a(1)*b(0));
                    };
                    
                    Eigen::Vector3d perp1, perp2;
                    if (std::abs(ray_unit(2)) < 0.9) {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(0, 0, 1)).normalized();
                    } else {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(1, 0, 0)).normalized();
                    }
                    perp2 = crossProduct(ray_unit, perp1).normalized();
                    
                    Eigen::Matrix3d R_ray;
                    R_ray.col(0) = perp1;
                    R_ray.col(1) = perp2;
                    R_ray.col(2) = ray_unit;
                    
                    Eigen::Matrix3d S_ray;
                    S_ray.setZero();
                    S_ray.diagonal() << 0.1, 0.1, 0.3;
                    
                    Eigen::Matrix3d Cov_pos = R_ray * (S_ray * S_ray.transpose()) * R_ray.transpose();
                    Eigen::LLT<Eigen::Matrix3d> llt_pos(Cov_pos);
                    Eigen::Matrix3d S_pos = llt_pos.matrixU();
                    
                    std::cout << "  [BOOTSTRAP] Duck " << detIdx << " initialized at distance=" 
                              << initialDepth << "m, area=" << area << " px²" << std::endl;
                    
                    auto p_new = GaussianInfo<double>::fromSqrtMoment(mu_new, S_pos);
                    system->density *= p_new;
                }
                
                std::cout << "  [BOOTSTRAP] Complete. State dim: " << system->density.dim() 
                          << ", Landmarks: " << system->numberLandmarks() << std::endl;
                
            } else {
                // GROWTH PHASE: Use candidate-based tracking for new ducks
            // Tracking structure for candidate detections
            struct CandidateDetection {
                cv::Point2f position;
                int area;
                int consecutiveFrames;
                int lastSeenFrame;
            };
            
            static std::vector<CandidateDetection> candidates;
            static int currentFrame = 0;
            currentFrame++;
            
            const int REQUIRED_FRAMES = 3;      // Must be unassociated for 3 consecutive frames
            const double PROXIMITY_THRESHOLD = 50.0;  // pixels - same detection if within this distance
            const int STALE_THRESHOLD = 10;     // Remove candidates not seen for 10 frames
            const double EDGE_MARGIN = 25.0;    // pixels - ignore detections within this margin from edges
            
            // Run association to see which detections are unassociated
            std::vector<std::size_t> landmarkIndices;
            for (std::size_t i = 0; i < system->numberLandmarks(); ++i) {
                landmarkIndices.push_back(i);
            }
            
            MeasurementDuckBundle tempMeas(time, duckCentroids, duckAreas, camera, 1);
            const std::vector<int>& tempAssoc = tempMeas.associate(*dynamic_cast<SystemSLAMPointLandmarks*>(system), landmarkIndices);
            
            // Find unassociated detections
            std::vector<size_t> unassociatedDetections;
            for (size_t i = 0; i < duckCentroids.size(); ++i) {
                bool isAssociated = false;
                for (size_t j = 0; j < tempAssoc.size(); ++j) {
                    if (tempAssoc[j] >= 0 && static_cast<size_t>(tempAssoc[j]) == i) {
                        isAssociated = true;
                        break;
                    }
                }
                if (!isAssociated) {
                    unassociatedDetections.push_back(i);
                }
            }
            
            // Update candidate tracking
            std::vector<CandidateDetection> updatedCandidates;
            
            for (size_t detIdx : unassociatedDetections) {
                const cv::Point2f& pos = duckCentroids[detIdx];
                int area = duckAreas[detIdx];
                
                // EDGE CHECK: Skip detections near frame boundaries (ducks entering/leaving)
                bool nearEdge = (pos.x < EDGE_MARGIN || pos.x > (camera.imageSize.width - EDGE_MARGIN) ||
                                pos.y < EDGE_MARGIN || pos.y > (camera.imageSize.height - EDGE_MARGIN));
                
                std::cout << "  [EDGE CHECK] Detection #" << detIdx << " at (" << pos.x << ", " << pos.y << ")"
                          << " frame=(" << camera.imageSize.width << "x" << camera.imageSize.height << ")"
                          << " nearEdge=" << (nearEdge ? "YES" : "NO") << std::endl;
                
                if (nearEdge) {
                    std::cout << "  [EDGE SKIP] Detection #" << detIdx << " too close to frame edge - ignoring" << std::endl;
                    continue;  // Skip detections at frame boundaries
                }
                
                // Check if this matches an existing candidate
                bool foundMatch = false;
                for (auto& candidate : candidates) {
                    double dx = pos.x - candidate.position.x;
                    double dy = pos.y - candidate.position.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < PROXIMITY_THRESHOLD) {
                        // Update existing candidate
                        candidate.position = pos;  // Update position
                        candidate.area = area;
                        candidate.consecutiveFrames++;
                        candidate.lastSeenFrame = currentFrame;
                        updatedCandidates.push_back(candidate);
                        foundMatch = true;
                        
                        std::cout << "  [CANDIDATE] Detection #" << detIdx 
                                  << " tracked for " << candidate.consecutiveFrames << " frames" << std::endl;
                        break;
                    }
                }
                
                // New candidate
                if (!foundMatch) {
                    CandidateDetection newCandidate;
                    newCandidate.position = pos;
                    newCandidate.area = area;
                    newCandidate.consecutiveFrames = 1;
                    newCandidate.lastSeenFrame = currentFrame;
                    updatedCandidates.push_back(newCandidate);
                    
                    std::cout << "  [NEW CANDIDATE] Detection #" << detIdx 
                              << " at (" << pos.x << ", " << pos.y << ")" << std::endl;
                }
            }
            
            // Keep candidates that are still recent (even if not seen this frame)
            for (const auto& candidate : candidates) {
                if ((currentFrame - candidate.lastSeenFrame) < STALE_THRESHOLD) {
                    bool alreadyAdded = false;
                    for (const auto& updated : updatedCandidates) {
                        if (updated.lastSeenFrame == currentFrame && 
                            std::abs(updated.position.x - candidate.position.x) < 1.0 &&
                            std::abs(updated.position.y - candidate.position.y) < 1.0) {
                            alreadyAdded = true;
                            break;
                        }
                    }
                    if (!alreadyAdded && candidate.lastSeenFrame != currentFrame) {
                        updatedCandidates.push_back(candidate);
                    }
                }
            }
            
            candidates = updatedCandidates;
            
            // Initialize landmarks for candidates that meet the threshold
            for (const auto& candidate : candidates) {
                if (candidate.consecutiveFrames >= REQUIRED_FRAMES && 
                    candidate.lastSeenFrame == currentFrame) {
                    
                    std::cout << "  [INIT] Candidate reached " << REQUIRED_FRAMES 
                              << " consecutive frames - checking if still unassociated..." << std::endl;
                    
                    // CRITICAL: Find detection by POSITION, not index (indices change between frames!)
                    // Search for detection closest to candidate's tracked position
                    size_t detIdx = 0;
                    double minDist = std::numeric_limits<double>::max();
                    bool foundMatch = false;
                    
                    for (size_t i = 0; i < duckCentroids.size(); ++i) {
                        double dx = duckCentroids[i].x - candidate.position.x;
                        double dy = duckCentroids[i].y - candidate.position.y;
                        double dist = std::sqrt(dx*dx + dy*dy);
                        
                        if (dist < minDist && dist < PROXIMITY_THRESHOLD) {
                            minDist = dist;
                            detIdx = i;
                            foundMatch = true;
                        }
                    }
                    
                    if (!foundMatch) {
                        std::cout << "  [INIT BLOCKED] No detection found near candidate position (" 
                                  << candidate.position.x << ", " << candidate.position.y << ") - skipping" << std::endl;
                        continue;
                    }
                    
                    std::cout << "  [INIT] Found detection #" << detIdx << " at distance=" << minDist 
                              << "px from candidate position" << std::endl;
                    
                    // CRITICAL CHECK: Re-verify detection is still unassociated
                    // (Another landmark may have claimed it during candidate tracking)
                    bool detectionStillUnassociated = true;
                    for (size_t j = 0; j < tempAssoc.size(); ++j) {
                        if (tempAssoc[j] >= 0 && static_cast<size_t>(tempAssoc[j]) == detIdx) {
                            detectionStillUnassociated = false;
                            std::cout << "  [INIT BLOCKED] Detection #" << detIdx 
                                      << " now claimed by LM" << j << " - skipping initialization" << std::endl;
                            break;
                        }
                    }
                    
                    if (!detectionStillUnassociated) {
                        continue;  // Skip this candidate - detection is taken
                    }
                    
                    std::cout << "  [INIT] Detection #" << detIdx << " still unassociated - proceeding with initialization" << std::endl;
                    
                    // DUPLICATE CHECK 1: Compare against predicted pixel positions
                    const double PREDICTION_THRESHOLD = 150.0;  // pixels
                    const int LARGE_DUCK_THRESHOLD = 1000;      // px² - large ducks are genuinely new
                    bool alreadyClaimed = false;
                    size_t claimingLandmarkIdx = 0;
                    
                    for (size_t j = 0; j < system->numberLandmarks(); ++j) {
                        try {
                            MeasurementDuckBundle tempMeas2(time, duckCentroids, duckAreas, camera, 1);
                            GaussianInfo<double> prQOi = tempMeas2.predictFeatureDensity(*dynamic_cast<SystemSLAMPointLandmarks*>(system), j);
                            Eigen::Vector2d predictedPixel = prQOi.mean();
                            
                            double dx = candidate.position.x - predictedPixel(0);
                            double dy = candidate.position.y - predictedPixel(1);
                            double dist = std::sqrt(dx*dx + dy*dy);
                            
                            if (dist < PREDICTION_THRESHOLD && candidate.area < LARGE_DUCK_THRESHOLD) {
                                alreadyClaimed = true;
                                claimingLandmarkIdx = j;
                                std::cout << "  [INIT BLOCKED] Candidate near LM" << j 
                                          << " prediction (dist=" << dist << "px)" << std::endl;
                                break;
                            }
                        } catch (...) {
                            continue;
                        }
                    }
                    
                    // DUPLICATE CHECK 2: Compare against current 3D landmark positions projected to image
                    // This catches duplicates even when predictions are uncertain
                    if (!alreadyClaimed) {
                        const double POSITION_THRESHOLD = 100.0;  // pixels - stricter than prediction check
                        
                        // Get current camera state for projection
                        Eigen::VectorXd current_state = system->density.mean();
                        Eigen::Matrix3d Rnc = SystemSLAM::cameraOrientation(camera, current_state);
                        Eigen::Vector3d rCNn = SystemSLAM::cameraPosition(camera, current_state);
                        
                        for (size_t j = 0; j < system->numberLandmarks(); ++j) {
                            try {
                                // Get landmark 3D position
                                Eigen::Vector3d rLNn = system->landmarkPositionDensity(j).mean();
                                
                                // Project to camera frame
                                Eigen::Vector3d rLCc = Rnc.transpose() * (rLNn - rCNn);
                                
                                // Check if in front of camera
                                if (rLCc(2) > 0.0) {
                                    // Project to pixel coordinates
                                    cv::Vec3d rLCc_cv(rLCc(0), rLCc(1), rLCc(2));
                                    cv::Vec2d pixel = camera.vectorToPixel(rLCc_cv);
                                    
                                    // Check distance to candidate
                                    double dx = candidate.position.x - pixel[0];
                                    double dy = candidate.position.y - pixel[1];
                                    double dist = std::sqrt(dx*dx + dy*dy);
                                    
                                    if (dist < POSITION_THRESHOLD) {
                                        alreadyClaimed = true;
                                        claimingLandmarkIdx = j;
                                        std::cout << "  [INIT BLOCKED] Candidate near LM" << j 
                                                  << " current position (dist=" << dist << "px)" << std::endl;
                                        break;
                                    }
                                }
                            } catch (...) {
                                continue;
                            }
                        }
                    }
                    
                    if (alreadyClaimed) {
                        std::cout << "  [INIT BLOCKED] Candidate at (" << candidate.position.x << "," 
                                  << candidate.position.y << ") too close to LM" << claimingLandmarkIdx 
                                  << " - skipping initialization" << std::endl;
                        continue;  // Skip this candidate - don't initialize
                    }
                    
                    // Check landmark limit - remove farthest if needed
                    if (system->numberLandmarks() >= MAX_LANDMARKS) {
                        std::cout << "  [LIMIT] Max landmarks (" << MAX_LANDMARKS 
                                  << ") reached - removing farthest before adding new" << std::endl;
                        removeFarthestLandmark();
                    }
                    
                    // Get current camera orientation and position
                    Eigen::VectorXd current_state = system->density.mean();
                    Eigen::Matrix3d Rnc = SystemSLAM::cameraOrientation(camera, current_state);
                    Eigen::Vector3d rCNn = SystemSLAM::cameraPosition(camera, current_state);
                    
                    // Get ray direction
                    const cv::Point2f& centroid = duckCentroids[detIdx];
                    cv::Vec3d uPCc = camera.pixelToVector(cv::Vec2d(centroid.x, centroid.y));
                    Eigen::Vector3d ray_cam(uPCc[0], uPCc[1], uPCc[2]);
                    
                    // Area-based depth estimation
                    const double fx = camera.cameraMatrix.at<double>(0, 0);
                    const double fy = camera.cameraMatrix.at<double>(1, 1);
                    const double duckRadius = 0.018;  // meters - 18mm characteristic radius
                    const double area = duckAreas[detIdx];
                    double initialDepth = std::sqrt((fx * fy * M_PI * duckRadius * duckRadius) / area);
                    initialDepth = std::max(0.3, std::min(initialDepth, 3.0));
                    
                    // Initialize landmark position
                    Eigen::Vector3d ray_world = Rnc * ray_cam;
                    Eigen::Vector3d rPNn = rCNn + initialDepth * ray_world;
                    Eigen::VectorXd mu_new(3);
                    mu_new = rPNn;
                    
                    // Construct covariance along viewing ray
                    Eigen::Vector3d ray_unit = ray_world.normalized();
                    auto crossProduct = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> Eigen::Vector3d {
                        return Eigen::Vector3d(a(1)*b(2) - a(2)*b(1), a(2)*b(0) - a(0)*b(2), a(0)*b(1) - a(1)*b(0));
                    };
                    
                    Eigen::Vector3d perp1, perp2;
                    if (std::abs(ray_unit(2)) < 0.9) {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(0, 0, 1)).normalized();
                    } else {
                        perp1 = crossProduct(ray_unit, Eigen::Vector3d(1, 0, 0)).normalized();
                    }
                    perp2 = crossProduct(ray_unit, perp1).normalized();
                    
                    Eigen::Matrix3d R_ray;
                    R_ray.col(0) = perp1;
                    R_ray.col(1) = perp2;
                    R_ray.col(2) = ray_unit;
                    
                    Eigen::Matrix3d S_ray;
                    S_ray.setZero();
                    S_ray.diagonal() << 0.1, 0.1, 0.3;
                    
                    Eigen::Matrix3d Cov_pos = R_ray * (S_ray * S_ray.transpose()) * R_ray.transpose();
                    Eigen::LLT<Eigen::Matrix3d> llt_pos(Cov_pos);
                    Eigen::Matrix3d S_pos = llt_pos.matrixU();
                    
                    std::cout << "  [INIT] New duck at detection #" << detIdx 
                              << " initialized at distance=" << initialDepth 
                              << "m, area=" << area << " px²" << std::endl;
                    
                    auto p_new = GaussianInfo<double>::fromSqrtMoment(mu_new, S_pos);
                    system->density *= p_new;
                    
                    std::cout << "  [DEBUG] After new duck init: State dim: " << system->density.dim() 
                              << ", Landmarks: " << system->numberLandmarks() << std::endl;
                }
            }
            }  // End of else block (GROWTH PHASE)
        }  // End of scenario 2 initialization
        
        // Process measurement through SLAM system
        if (scenario == 2) {
            // Scenario 2: Duck SLAM processing
            if (!duckCentroids.empty()) {
                // Create measurement - no pre-associations needed
                if (processedMeasurement2 == nullptr) {
                    processedMeasurement2 = new MeasurementDuckBundle(time, duckCentroids, duckAreas, camera, 1);
                }
                
                // Process measurement - pre-associations were already set if needed
                processedMeasurement2->process(*system);
                
                // Display association results
                const std::vector<int>& associations = processedMeasurement2->getAssociationStatus();
                std::cout << "  [ASSOC] " << associations.size() << " detections processed" << std::endl;
                
                // RED ELLIPSE PRUNING: Track and remove stale landmarks
                static std::vector<int> landmarkRedFrameCount;
                static int totalDucksDiscovered = 0;  // Cumulative count (never decreases)
                
                // Track new landmarks discovered this frame
                int landmarksBeforePruning = system->numberLandmarks();
                if (landmarksBeforePruning > totalDucksDiscovered) {
                    totalDucksDiscovered = landmarksBeforePruning;
                }
                
                // Resize tracking vector if needed
                while (landmarkRedFrameCount.size() < system->numberLandmarks()) {
                    landmarkRedFrameCount.push_back(0);
                }
                
                // Determine which landmarks are red (visible but unassociated)
                std::vector<bool> isLandmarkRed(system->numberLandmarks(), false);
                
                for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                    // Check if this landmark is associated using landmark-to-detection mapping
                    // associations[i] = detection index for landmark i, or -1 if unassociated
                    bool isAssociated = (i < associations.size() && associations[i] >= 0);
                    
                    if (isAssociated) {
                        // Blue - reset counter
                        landmarkRedFrameCount[i] = 0;
                    } else {
                        // Check if visible (in FOV and within image bounds)
                        Eigen::Vector3d rLNn = system->landmarkPositionDensity(i).mean();
                        Eigen::VectorXd state = system->density.mean();
                        Eigen::Vector3d rCNn = system->cameraPositionDensity(camera).mean();
                        Eigen::Vector3d Thetanc = system->cameraOrientationEulerDensity(camera).mean();
                        Eigen::Matrix3d Rnc = rpy2rot(Thetanc);
                        Eigen::Vector3d rLCc = Rnc.transpose() * (rLNn - rCNn);
                        
                        if (rLCc(2) > 0.0) {
                            try {
                                GaussianInfo prQOi = processedMeasurement2->predictFeatureDensity(*system, i);
                                Eigen::VectorXd murQOi = prQOi.mean();
                                
                                // Check if within actual image bounds
                                bool inBounds = (murQOi(0) >= 0 && murQOi(0) < camera.imageSize.width &&
                                                murQOi(1) >= 0 && murQOi(1) < camera.imageSize.height);
                                
                                if (inBounds) {
                                    // Check if there's an UNASSOCIATED detection nearby
                                    // RED = visible with unassociated detection (gate rejected it)
                                    // YELLOW = no unassociated detection nearby (off-screen or detection already claimed)
                                    const double NEARBY_THRESHOLD = 100.0;  // pixels
                                    bool hasNearbyUnassociatedDetection = false;
                                    
                                    for (size_t detIdx = 0; detIdx < duckCentroids.size(); ++detIdx) {
                                        // Check if this detection is already associated with another landmark
                                        bool detIsAssociated = false;
                                        for (size_t j = 0; j < associations.size(); ++j) {
                                            if (associations[j] >= 0 && 
                                                static_cast<size_t>(associations[j]) == detIdx) {
                                                detIsAssociated = true;
                                                break;
                                            }
                                        }
                                        
                                        // Only check proximity for unassociated detections
                                        if (!detIsAssociated) {
                                            const auto& detection = duckCentroids[detIdx];
                                            double dx = detection.x - murQOi(0);
                                            double dy = detection.y - murQOi(1);
                                            double dist = std::sqrt(dx*dx + dy*dy);
                                            if (dist < NEARBY_THRESHOLD) {
                                                hasNearbyUnassociatedDetection = true;
                                                break;
                                            }
                                        }
                                    }
                                    
                                    if (hasNearbyUnassociatedDetection) {
                                        // Red - visible with unassociated detection (gate rejected)
                                        isLandmarkRed[i] = true;
                                        landmarkRedFrameCount[i]++;
                                    } else {
                                        // Yellow - no unassociated detection nearby
                                        landmarkRedFrameCount[i] = 0;  // Reset counter!
                                    }
                                } else {
                                    landmarkRedFrameCount[i] = 0;  // Reset counter
                                }
                            } catch (...) {
                                // Prediction failed - treat as not visible
                                landmarkRedFrameCount[i] = 0;  // Reset counter
                            }
                        } else {
                            landmarkRedFrameCount[i] = 0;  // Reset counter
                        }
                    }
                }
                
                // Delete landmarks that have been red for 3+ consecutive frames
                const int RED_FRAME_THRESHOLD = 3;
                std::vector<size_t> landmarksToDelete;
                
                for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                    if (landmarkRedFrameCount[i] >= RED_FRAME_THRESHOLD) {
                        landmarksToDelete.push_back(i);
                        std::cout << "  [PRUNE] LM" << i << " red for " << landmarkRedFrameCount[i] 
                                  << " frames - DELETING" << std::endl;
                    }
                }
                
                // Delete landmarks in reverse order to maintain indices
                for (auto it = landmarksToDelete.rbegin(); it != landmarksToDelete.rend(); ++it) {
                    size_t landmarkIdx = *it;
                    
                    // Marginalize out this landmark
                    size_t stateDim = system->density.dim();
                    size_t landmarkDim = 3;  // 3-DOF point landmarks
                    size_t bodyDim = 12;     // Camera state dimension
                    
                    // Build index vector: keep body state + all landmarks except this one
                    std::vector<int> keepIndices;
                    
                    // Keep body state (first 12 elements)
                    for (size_t i = 0; i < bodyDim; ++i) {
                        keepIndices.push_back(i);
                    }
                    
                    // Keep landmarks before this one
                    for (size_t i = 0; i < landmarkIdx; ++i) {
                        size_t landmarkStart = bodyDim + i * landmarkDim;
                        for (size_t j = 0; j < landmarkDim; ++j) {
                            keepIndices.push_back(landmarkStart + j);
                        }
                    }
                    
                    // Keep landmarks after this one
                    for (size_t i = landmarkIdx + 1; i < system->numberLandmarks(); ++i) {
                        size_t landmarkStart = bodyDim + i * landmarkDim;
                        for (size_t j = 0; j < landmarkDim; ++j) {
                            keepIndices.push_back(landmarkStart + j);
                        }
                    }
                    
                    // Marginalize
                    Eigen::VectorXi keepIndicesEigen = Eigen::Map<Eigen::VectorXi>(keepIndices.data(), keepIndices.size());
                    system->density = system->density.marginal(keepIndicesEigen);
                    
                    // Remove from tracking vector
                    landmarkRedFrameCount.erase(landmarkRedFrameCount.begin() + landmarkIdx);
                    
                    std::cout << "  [PRUNE] Landmark removed. New state dim: " << system->density.dim() 
                              << ", Landmarks: " << system->numberLandmarks() << std::endl;
                }
            }
        }
        
        // Process scenario 1 measurement and store for visualization
        if (scenario == 1) {
            // Debug: Check landmark covariances BEFORE measurement update
            std::cout << "\n  [COV BEFORE] Landmark uncertainties before measurement:" << std::endl;
            for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                auto landmarkDensity = system->landmarkPositionDensity(i);
                Eigen::Matrix3d cov = landmarkDensity.cov();
                Eigen::Vector3d sigma = cov.diagonal().array().sqrt();
                double maxSigma = sigma.maxCoeff();
                std::cout << "    Landmark " << i << " sigma: ["
                          << sigma(0) << ", " << sigma(1) << ", " << sigma(2) 
                          << "] max=" << maxSigma << "m";
                if (maxSigma > 2.0) std::cout << " [LARGE!]";
                std::cout << std::endl;
            }
            
            processedMeasurement1 = new MeasurementUniqueTagBundle(time, markerIds, markerCorners, camera, 0.166);
            processedMeasurement1->process(*system);
            
            // Debug: Track association results
            const auto& assoc = processedMeasurement1->getAssociationStatus();
            std::cout << "\n  [ASSOC] Frame " << frameIdx << " association results:" << std::endl;
            std::cout << "    Detected markers: " << markerIds.size() << std::endl;
            std::cout << "    Existing landmarks: " << system->numberLandmarks() << std::endl;
            std::cout << "    Associations: ";
            for (size_t i = 0; i < assoc.size(); ++i) {
                std::cout << "Marker[" << markerIds[i] << "]->Landmark[" << assoc[i] << "] ";
            }
            std::cout << std::endl;
            
            // Track which landmarks were updated
            std::vector<bool> landmarkUpdated(system->numberLandmarks(), false);
            for (int landmarkIdx : assoc) {
                if (landmarkIdx >= 0 && static_cast<size_t>(landmarkIdx) < system->numberLandmarks()) {
                    landmarkUpdated[landmarkIdx] = true;
                }
            }
            
            std::cout << "    Updated landmarks: ";
            for (size_t i = 0; i < landmarkUpdated.size(); ++i) {
                if (landmarkUpdated[i]) std::cout << i << " ";
            }
            std::cout << std::endl;
            
            std::cout << "    Unassociated landmarks: ";
            for (size_t i = 0; i < landmarkUpdated.size(); ++i) {
                if (!landmarkUpdated[i]) std::cout << i << " ";
            }
            std::cout << std::endl;
            
            // Debug: Check landmark covariances AFTER measurement update
            std::cout << "\n  [COV AFTER] Landmark uncertainties after measurement:" << std::endl;
            for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                auto landmarkDensity = system->landmarkPositionDensity(i);
                Eigen::Matrix3d cov = landmarkDensity.cov();
                Eigen::Vector3d sigma = cov.diagonal().array().sqrt();
                double maxSigma = sigma.maxCoeff();
                std::cout << "    Landmark " << i << " sigma: ["
                          << sigma(0) << ", " << sigma(1) << ", " << sigma(2) 
                          << "] max=" << maxSigma << "m";
                if (!landmarkUpdated[i]) std::cout << " [UNASSOCIATED]";
                if (maxSigma > 2.0) std::cout << " [LARGE!]";
                std::cout << std::endl;
            }
        }

        // Visualization and status tracking
        if (scenario == 2 && processedMeasurement2 != nullptr) {
            // Count ducks on screen (associated landmarks = blue in visualization)
            int totalDucks = system->numberLandmarks();
            int ducksOnScreen = 0;
            int associatedDucks = 0;
            
            // Get association status from the measurement
            const std::vector<int>& associations = processedMeasurement2->getAssociationStatus();
            
            // Count associated landmarks (blue ellipses)
            for (size_t i = 0; i < associations.size(); ++i) {
                if (associations[i] >= 0) {
                    associatedDucks++;
                }
            }
            
            // Ducks on screen = blue + red ellipses (all visible landmarks)
            ducksOnScreen = associatedDucks;  // Start with blue
            int redEllipses = 0;
            
            // Add red ellipses (visible but unassociated)
            for (size_t i = 0; i < system->numberLandmarks(); ++i) {
                bool isAssociated = (i < associations.size() && associations[i] >= 0);
                
                if (!isAssociated) {
                    // Check if this landmark is visible (same logic as Plot uses for RED ellipses)
                    // RED = visible AND has nearby unassociated detection
                    try {
                        Eigen::Vector3d rLNn = system->landmarkPositionDensity(i).mean();
                        Eigen::Vector3d rCNn = system->cameraPositionDensity(camera).mean();
                        Eigen::Vector3d Thetanc = system->cameraOrientationEulerDensity(camera).mean();
                        Eigen::Matrix3d Rnc = rpy2rot(Thetanc);
                        Eigen::Vector3d rLCc = Rnc.transpose() * (rLNn - rCNn);
                        
                        if (rLCc(2) > 0.0) {  // In front of camera
                            GaussianInfo prQOi = processedMeasurement2->predictFeatureDensity(*system, i);
                            Eigen::VectorXd murQOi = prQOi.mean();
                            
                            // Check if within image bounds
                            if (murQOi(0) >= 0 && murQOi(0) < camera.imageSize.width &&
                                murQOi(1) >= 0 && murQOi(1) < camera.imageSize.height) {
                                
                                // Check if there's a nearby unassociated detection (for RED)
                                const double NEARBY_THRESHOLD = 100.0;
                                bool hasNearbyUnassociatedDetection = false;
                                
                                for (size_t detIdx = 0; detIdx < duckCentroids.size(); ++detIdx) {
                                    bool detIsAssociated = false;
                                    for (size_t j = 0; j < associations.size(); ++j) {
                                        if (associations[j] >= 0 && static_cast<size_t>(associations[j]) == detIdx) {
                                            detIsAssociated = true;
                                            break;
                                        }
                                    }
                                    
                                    if (!detIsAssociated) {
                                        const auto& detection = duckCentroids[detIdx];
                                        double dx = detection.x - murQOi(0);
                                        double dy = detection.y - murQOi(1);
                                        double dist = std::sqrt(dx*dx + dy*dy);
                                        if (dist < NEARBY_THRESHOLD) {
                                            hasNearbyUnassociatedDetection = true;
                                            break;
                                        }
                                    }
                                }
                                
                                if (hasNearbyUnassociatedDetection) {
                                    redEllipses++;
                                    ducksOnScreen++;  // Add red ellipse to on-screen count
                                }
                            }
                        }
                    } catch (...) {
                        // Prediction failed - not visible
                    }
                }
            }
            
            // Off screen ducks = yellow ellipses (not visible)
            int offScreenDucks = totalDucks - ducksOnScreen;
            
            // Access the cumulative count from the pruning section
            extern int totalDucksDiscovered;  // Declared static in pruning section above
            
            // Position box at bottom left
            int boxX = 20;
            int boxY = imgAnnotated.rows - 145;  // 145 pixels from bottom
            int boxWidth = 230;
            int boxHeight = 125;
            double fontSize = 0.5;
            int thickness = 1;
            int lineHeight = 25;
            
            // Semi-transparent background
            cv::Mat overlay = imgAnnotated.clone();
            cv::rectangle(overlay,
                         cv::Point(boxX - 10, boxY - 10),
                         cv::Point(boxX + boxWidth, boxY + boxHeight),
                         cv::Scalar(240, 240, 240), -1);
            cv::addWeighted(overlay, 0.7, imgAnnotated, 0.3, 0, imgAnnotated);
            
            // Title
            cv::putText(imgAnnotated, "Duck Statistics:",
                       cv::Point(boxX, boxY + 20),
                       cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 0, 0), thickness);
            
            // Stats - use static variable from pruning block
            static int lastTotalDiscovered = 0;
            if (processedMeasurement2 != nullptr) {
                // Update cumulative discovered count
                int discoveredCount = std::max(totalDucks, lastTotalDiscovered);
                if (totalDucks > lastTotalDiscovered) {
                    lastTotalDiscovered = totalDucks;
                }
                
                std::string line1 = "Ducks Discovered: " + std::to_string(lastTotalDiscovered);
                std::string line2 = "Associated Ducks: " + std::to_string(associatedDucks);
                std::string line3 = "Ducks On Screen: " + std::to_string(ducksOnScreen);
                std::string line4 = "Off Screen Ducks: " + std::to_string(offScreenDucks);
                
                cv::putText(imgAnnotated, line1,
                           cv::Point(boxX, boxY + 45),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 100, 0), thickness);
                cv::putText(imgAnnotated, line2,
                           cv::Point(boxX, boxY + 70),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 100, 0), thickness);
                cv::putText(imgAnnotated, line3,
                           cv::Point(boxX, boxY + 95),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 100, 0), thickness);
                cv::putText(imgAnnotated, line4,
                           cv::Point(boxX, boxY + 120),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 100, 0), thickness);
            }
        }
        
        if (scenario == 1 && processedMeasurement1 != nullptr) {
            // Track discovered ArUco markers and their association status
            struct MarkerInfo {
                int id;
                int discoveryOrder;
                std::string status;  // "Associated" or "Unassociated"
            };
            static std::vector<MarkerInfo> discoveredMarkers;
            static int nextDiscoveryOrder = 1;

            // Get association status from the PROCESSED measurement
            const std::vector<int>& associationStatus = processedMeasurement1->getAssociationStatus();
            const std::vector<int>& detectedIds = processedMeasurement1->getMarkerIds();

            // Update status for each detected marker
            for (size_t i = 0; i < detectedIds.size(); ++i) {
                int id = detectedIds[i];

                // Determine if this marker is associated
                bool isAssociated = (i < associationStatus.size() &&
                                    associationStatus[i] >= 0 &&
                                    static_cast<size_t>(associationStatus[i]) < system->numberLandmarks());

                // Find if this marker already exists in discoveredMarkers
                bool found = false;
                for (auto& marker : discoveredMarkers) {
                    if (marker.id == id) {
                        marker.status = isAssociated ? "Associated" : "Unassociated";
                        found = true;
                        break;
                    }
                }

                // If new marker, add it
                if (!found) {
                    MarkerInfo newMarker;
                    newMarker.id = id;
                    newMarker.discoveryOrder = nextDiscoveryOrder++;
                    newMarker.status = isAssociated ? "Associated" : "Unassociated";
                    discoveredMarkers.push_back(newMarker);
                }

                // Draw red box around unassociated markers
                if (!isAssociated && i < markerCorners.size()) {
                    const auto& corners = markerCorners[i];
                    if (corners.size() == 4) {
                        // Find bounding box of marker
                        float minX = corners[0].x, maxX = corners[0].x;
                        float minY = corners[0].y, maxY = corners[0].y;
                        for (const auto& corner : corners) {
                            minX = std::min(minX, corner.x);
                            maxX = std::max(maxX, corner.x);
                            minY = std::min(minY, corner.y);
                            maxY = std::max(maxY, corner.y);
                        }

                        // Draw thick red rectangle
                        cv::rectangle(imgAnnotated,
                                    cv::Point(static_cast<int>(minX), static_cast<int>(minY)),
                                    cv::Point(static_cast<int>(maxX), static_cast<int>(maxY)),
                                    cv::Scalar(0, 0, 255), 4);
                    }
                }
            }

            // Draw table overlay showing discovered markers
            if (!discoveredMarkers.empty()) {
                int tableX = 20;
                int tableY = 20;
                int lineHeight = 25;
                double fontSize = 0.5;  // Must be double for cv::putText
                int thickness = 1;

                // Semi-transparent background
                int tableWidth = 250;
                int tableHeight = 60 + discoveredMarkers.size() * lineHeight;
                cv::Mat overlay = imgAnnotated.clone();
                cv::rectangle(overlay,
                            cv::Point(tableX - 10, tableY - 10),
                            cv::Point(tableX + tableWidth, tableY + tableHeight),
                            cv::Scalar(240, 240, 240), -1);
                cv::addWeighted(overlay, 0.7, imgAnnotated, 0.3, 0, imgAnnotated);

                // Title
                cv::putText(imgAnnotated, "ArUco Tags Found:",
                           cv::Point(tableX, tableY + 20),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 0, 0), thickness);

                // Header
                cv::putText(imgAnnotated, " ID | Status",
                           cv::Point(tableX, tableY + 45),
                           cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(0, 0, 0), thickness);
                cv::line(imgAnnotated,
                        cv::Point(tableX, tableY + 50),
                        cv::Point(tableX + tableWidth - 20, tableY + 50),
                        cv::Scalar(0, 0, 0), 1);

                // Marker rows
                int rowY = tableY + 70;
                for (const auto& marker : discoveredMarkers) {
                    std::string rowText = std::to_string(marker.id) + "  | " + marker.status;
                    cv::Scalar color = (marker.status == "Associated") ?
                                     cv::Scalar(0, 150, 0) : cv::Scalar(0, 0, 200);
                    cv::putText(imgAnnotated, rowText,
                               cv::Point(tableX, rowY),
                               cv::FONT_HERSHEY_SIMPLEX, fontSize, color, thickness);
                    rowY += lineHeight;
                }
            }
        }
        
        // Record state history for plotting
        std::cout << "  [DEBUG] Recording state history" << std::endl;
        timeHistory.push_back(time);
        std::cout << "  [DEBUG] Getting mean state" << std::endl;
        Eigen::VectorXd current_state = system->density.mean().head(12);
        std::cout << "  [DEBUG] Pushing state to history" << std::endl;
        stateHistory.push_back(current_state);

        // Debug: Print state values occasionally
        if (frameIdx % 30 == 0) {
            std::cout << "[DEBUG] Frame " << frameIdx << " state: "
                      << "vel=[" << current_state(0) << "," << current_state(1) << "," << current_state(2) << "] "
                      << "pos=[" << current_state(6) << "," << current_state(7) << "," << current_state(8) << "]" << std::endl;
        }

        // Extract standard deviations from covariance diagonal
        std::cout << "  [DEBUG] About to extract covariance" << std::endl;
        Eigen::MatrixXd cov = system->density.cov();
        std::cout << "  [DEBUG] Covariance extracted, size: " << cov.rows() << "x" << cov.cols() << std::endl;
        Eigen::VectorXd sigma(12);
        for (int i = 0; i < 12; ++i) {
            sigma(i) = std::sqrt(cov(i, i));
        }
        sigmaHistory.push_back(sigma);
        
        // Print uncertainty for camera position
        std::cout << "  [COVARIANCE] Position uncertainty (std dev): ["
                  << sigma(6) << ", " << sigma(7) << ", " << sigma(8) << "]" << std::endl;
        
        std::cout << "  [DEBUG] Covariance processing complete" << std::endl;
        
        // Update plot
        std::cout << "  [DEBUG] Updating plot view" << std::endl;
        system->view() = imgAnnotated.clone();   // supply frame to plot

        std::cout << "  [DEBUG] Creating measurement for plotting" << std::endl;
        
        // Use the processed measurement for plotting (with correct associations)
        if (scenario == 1 && processedMeasurement1 != nullptr) {
            plot.setData(*system, *processedMeasurement1);
        } else if (scenario == 2 && processedMeasurement2 != nullptr) {
            plot.setData(*system, *processedMeasurement2);
        }
        
        std::cout << "  [DEBUG] About to call plot.render()" << std::endl;
        plot.render();                          // update visualisation
        std::cout << "  [DEBUG] Plot rendered successfully" << std::endl;

        // Handle interactivity based on --interactive parameter
        if (interactive == 2)
        {
            // Enable full VTK interaction at each frame
            // User can rotate, zoom, pan the 3D view
            // Press 'q' in the VTK window to continue to next frame
            std::cout << "\nInteracting with frame " << frameIdx << ". Press 'q' in the VTK window to continue to next frame." << std::endl;
            plot.start();
            // Mark that user has interacted - preserve their camera view for subsequent frames
            plot.markUserInteraction();
        }
        else if (interactive == 1 && isLastFrame)
        {
            // Enable interaction only on the last frame
            plot.start();
            plot.markUserInteraction();
        }
        // If interactive == 0, don't block - just continue

        // Write output frame 
        if (doExport)
        {
            cv::Mat imgout = plot.getFrame();
            bufferedVideoWriter.write(imgout);
        }
        
        // Clean up measurement pointers at end of frame (after all plotting is complete)
        if (processedMeasurement1 != nullptr) {
            delete processedMeasurement1;
            processedMeasurement1 = nullptr;
        }
        if (processedMeasurement2 != nullptr) {
            delete processedMeasurement2;
            processedMeasurement2 = nullptr;
        }
    }

    if (doExport)
    {
         bufferedVideoWriter.stop();
    }
    bufferedVideoReader.stop();
    
    // Plot camera state history
    if (!timeHistory.empty())
    {
        // Convert vectors to Eigen matrices for plotting
        int nSteps = timeHistory.size();
        Eigen::VectorXd t_hist(nSteps);
        Eigen::MatrixXd mu_hist(12, nSteps);
        Eigen::MatrixXd sigma_hist(12, nSteps);
        
        for (int i = 0; i < nSteps; ++i)
        {
            t_hist(i) = timeHistory[i];
            mu_hist.col(i) = stateHistory[i];
            sigma_hist.col(i) = sigmaHistory[i];
        }
        
        // Call plotting function
        std::cout << "\nCamera state history collected: " << nSteps << " frames" << std::endl;
        std::cout << "Generating state history plots..." << std::endl;
        plotCameraStates(t_hist, mu_hist, sigma_hist);
    }

    // Clean up
    delete system;
}
