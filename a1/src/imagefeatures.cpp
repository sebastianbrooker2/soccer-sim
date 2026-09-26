#include <string>  
#include <print>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/aruco.hpp>
#include "imagefeatures.h"

cv::Mat detectAndDrawHarris(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }
    
    // Apply Harris corner detection
    cv::Mat harris_response;
    int blockSize = 3;      // Size of neighborhood considered for corner detection
    int ksize = 3;          // Aperture parameter for Sobel derivative
    double k = 0.04;        // Harris detector free parameter
    
    cv::cornerHarris(gray, harris_response, blockSize, ksize, k);
    
    // Create structure to store corner information
    struct CornerPoint {
        int x, y;
        float score;
        
        // For sorting by score (highest first)
        bool operator<(const CornerPoint& other) const {
            return score > other.score;
        }
    };
    
    std::vector<CornerPoint> corners;
    
    // Find maximum Harris response to set threshold
    double minVal, maxVal;
    cv::minMaxLoc(harris_response, &minVal, &maxVal);
    float threshold = 0.2 * maxVal;  // Adjust this value as needed
    
    // Task 1a: Store all pixels above threshold
    for (int y = 0; y < harris_response.rows; y++) {
        for (int x = 0; x < harris_response.cols; x++) {
            float response = harris_response.at<float>(y, x);
            if (response > threshold) {
                corners.push_back({x, y, response});
            }
        }
    }
    
    // Sort corners by Harris score (highest first)
    std::sort(corners.begin(), corners.end());
    
    // Task 1b: Plot all features above threshold
    for (const auto& corner : corners) {
        cv::circle(imgout, cv::Point(corner.x, corner.y), 3, cv::Scalar(0, 255, 0), 1);
    }
    
    // Task 1c: Print sorted list of N most highly textured features
    std::println("Using harris feature detector");
    std::println("Image width: {}", img.cols);
    std::println("Image height: {}", img.rows);
    std::println("Features requested: {}", maxNumFeatures);
    std::println("Features detected: {}", corners.size());
    
    int numToPrint = std::min(maxNumFeatures, (int)corners.size());
    for (int i = 0; i < numToPrint; i++) {
        std::println("idx: {} at point: ({},{}) Harris Score: {}", 
                    i, corners[i].x, corners[i].y, corners[i].score);
    }
    
    // Task 1d: Plot indices of N most highly textured features
    for (int i = 0; i < numToPrint; i++) {
        cv::putText(imgout, std::to_string(i), 
                   cv::Point(corners[i].x + 5, corners[i].y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }
    
    return imgout;
}

cv::Mat detectAndDrawShiAndTomasi(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }
    
    // Apply Shi & Tomasi corner detection using cornerMinEigenVal
    cv::Mat eigenval_response;
    int blockSize = 3;      // Size of neighborhood considered for corner detection
    int ksize = 3;          // Aperture parameter for Sobel derivative
    
    cv::cornerMinEigenVal(gray, eigenval_response, blockSize, ksize);
    
    // Create structure to store corner information
    struct CornerPoint {
        int x, y;
        float score;
        
        // For sorting by score (highest first)
        bool operator<(const CornerPoint& other) const {
            return score > other.score;
        }
    };
    
    std::vector<CornerPoint> corners;
    
    // Find maximum eigenvalue response to set threshold
    double minVal, maxVal;
    cv::minMaxLoc(eigenval_response, &minVal, &maxVal);
    float threshold = 0.3 * maxVal;  // Adjust this value as needed
    
    // Task 2a: Store all pixels above threshold
    for (int y = 0; y < eigenval_response.rows; y++) {
        for (int x = 0; x < eigenval_response.cols; x++) {
            float response = eigenval_response.at<float>(y, x);
            if (response > threshold) {
                corners.push_back({x, y, response});
            }
        }
    }
    
    // Sort corners by eigenvalue score (highest first)
    std::sort(corners.begin(), corners.end());
    
    // Task 2b: Plot all features above threshold
    for (const auto& corner : corners) {
        cv::circle(imgout, cv::Point(corner.x, corner.y), 3, cv::Scalar(0, 255, 0), 1);
    }
    
    // Task 2c: Print sorted list of N most highly textured features
    std::println("Using shi feature detector");
    std::println("Image width: {}", img.cols);
    std::println("Image height: {}", img.rows);
    std::println("Features requested: {}", maxNumFeatures);
    std::println("Features detected: {}", corners.size());
    
    int numToPrint = std::min(maxNumFeatures, (int)corners.size());
    for (int i = 0; i < numToPrint; i++) {
        std::println("idx: {} at point: ({},{}) Shi-Tomasi Score: {}", 
                    i, corners[i].x, corners[i].y, corners[i].score);
    }
    
    // Task 2d: Plot indices of N most highly textured features
    for (int i = 0; i < numToPrint; i++) {
        cv::putText(imgout, std::to_string(i), 
                   cv::Point(corners[i].x + 5, corners[i].y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }
    
    return imgout;
}

cv::Mat detectAndDrawFAST(const cv::Mat & img, int maxNumFeatures)
{
    cv::Mat imgout = img.clone();
    
    // Convert to grayscale if needed
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }
    
    // Create FAST detector with adjustable threshold
    int threshold = 100;        // Intensity threshold - higher values = fewer features
    bool nonmaxSuppression = true;  // Apply non-maximum suppression
    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(threshold, nonmaxSuppression);
    
    // Detect keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray, keypoints);
    
    // Create structure to store corner information
    struct CornerPoint {
        int x, y;
        float score;
        
        // For sorting by score (highest first)
        bool operator<(const CornerPoint& other) const {
            return score > other.score;
        }
    };
    
    std::vector<CornerPoint> corners;
    
    // Task 3a: Store all detected keypoints and their scores
    for (const auto& keypoint : keypoints) {
        corners.push_back({(int)keypoint.pt.x, (int)keypoint.pt.y, keypoint.response});
    }
    
    // Sort corners by FAST score (highest first)
    std::sort(corners.begin(), corners.end());
    
    // Task 3b: Plot all detected features
    for (const auto& corner : corners) {
        cv::circle(imgout, cv::Point(corner.x, corner.y), 5, cv::Scalar(0, 255, 0), 1);
    }
    
    // Task 3c: Print sorted list of N strongest features
    std::println("Using fast feature detector");
    std::println("Image width: {}", img.cols);
    std::println("Image height: {}", img.rows);
    std::println("Features requested: {}", maxNumFeatures);
    std::println("Features detected: {}", corners.size());
    
    int numToPrint = std::min(maxNumFeatures, (int)corners.size());
    for (int i = 0; i < numToPrint; i++) {
        std::println("idx: {} at point: ({},{}) FAST Score: {}", 
                    i, corners[i].x, corners[i].y, (int)corners[i].score);
    }
    
    // Task 3d: Plot indices of N strongest features
    for (int i = 0; i < numToPrint; i++) {
        cv::putText(imgout, std::to_string(i), 
                   cv::Point(corners[i].x + 5, corners[i].y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }
    
    return imgout;
}

cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures, 
                           std::vector<int> & markerIds, 
                           std::vector<std::vector<cv::Point2f>> & markerCorners)
{
    cv::Mat imgout = img.clone();
    
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }
    
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    
    detector.detectMarkers(gray, markerCorners, markerIds);
    
    // Create structure to store marker information for sorting
    struct MarkerInfo {
        int id;
        std::vector<cv::Point2f> corners;
        
        // For sorting by ID
        bool operator<(const MarkerInfo& other) const {
            return id < other.id;
        }
    };
    
    std::vector<MarkerInfo> markers;
    
    // Task 4a: Store all detected markers
    for (size_t i = 0; i < markerIds.size(); i++) {
        markers.push_back({markerIds[i], markerCorners[i]});
    }
    
    // Sort markers by ID
    std::sort(markers.begin(), markers.end());
    
    // Task 4b: Print sorted list of marker corner locations
    std::println("Using aruco feature detector");
    std::println("Image width: {}", img.cols);
    std::println("Image height: {}", img.rows);
    
    for (const auto& marker : markers) {
        // Print corners in the format: (x,y) (x,y) (x,y) (x,y)
        std::println("ID: {} with corners: ({},{}) ({},{}) ({},{}) ({},{})",
                    marker.id,
                    (int)marker.corners[0].x, (int)marker.corners[0].y,
                    (int)marker.corners[1].x, (int)marker.corners[1].y,
                    (int)marker.corners[2].x, (int)marker.corners[2].y,
                    (int)marker.corners[3].x, (int)marker.corners[3].y);
    }
    
    // Task 4c: Plot ArUco markers using drawDetectedMarkers
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(imgout, markerCorners, markerIds);
        
        // Additionally draw dots at each corner and center
        for (size_t i = 0; i < markerCorners.size(); i++) {
            const auto & corners = markerCorners[i];
            
            // Draw small blue dots at each corner
            for (const auto & corner : corners) {
                cv::circle(imgout, corner, 3, cv::Scalar(255, 0, 0), -1);  // Blue filled circles
            }
            
            // Compute and draw center (average of 4 corners)
            cv::Point2f center(0, 0);
            for (const auto & corner : corners) {
                center.x += corner.x;
                center.y += corner.y;
            }
            center.x /= 4.0f;
            center.y /= 4.0f;
            
            // Draw larger red dot at center
            cv::circle(imgout, center, 5, cv::Scalar(0, 0, 255), -1);  // Red filled circle
        }
    }
    
    return imgout;
}
