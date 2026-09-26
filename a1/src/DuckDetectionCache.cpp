#include "DuckDetectionCache.h"
#include <fstream>
#include <stdexcept>
#include <cstdint>

DuckDetectionCache::DuckDetectionCache(const std::string& cache_path) {
    std::ifstream file(cache_path, std::ios::binary);
    
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open duck detection cache file: " + cache_path);
    }
    
    // Read all frames from the cache file
    while (file.good()) {
        uint32_t num_detections;
        file.read(reinterpret_cast<char*>(&num_detections), sizeof(uint32_t));
        
        if (!file.good()) {
            break;  // End of file
        }
        
        std::vector<DuckDetection> frame_detections;
        frame_detections.reserve(num_detections);
        
        for (uint32_t i = 0; i < num_detections; ++i) {
            DuckDetection detection;
            file.read(reinterpret_cast<char*>(&detection.centroid_x), sizeof(float));
            file.read(reinterpret_cast<char*>(&detection.centroid_y), sizeof(float));
            file.read(reinterpret_cast<char*>(&detection.area), sizeof(int32_t));
            
            if (!file.good()) {
                throw std::runtime_error("Error reading detection data from cache file");
            }
            
            frame_detections.push_back(detection);
        }
        
        frames_.push_back(std::move(frame_detections));
    }
    
    file.close();
    
    if (frames_.empty()) {
        throw std::runtime_error("No frames loaded from cache file");
    }
}

const std::vector<DuckDetection>& DuckDetectionCache::getFrame(size_t frame_idx) const {
    if (frame_idx >= frames_.size()) {
        throw std::out_of_range("Frame index out of range");
    }
    return frames_[frame_idx];
}

std::vector<cv::Point2f> DuckDetectionCache::getCentroids(size_t frame_idx) const {
    const auto& detections = getFrame(frame_idx);
    std::vector<cv::Point2f> centroids;
    centroids.reserve(detections.size());
    
    for (const auto& detection : detections) {
        centroids.emplace_back(detection.centroid_x, detection.centroid_y);
    }
    
    return centroids;
}

std::vector<int> DuckDetectionCache::getAreas(size_t frame_idx) const {
    const auto& detections = getFrame(frame_idx);
    std::vector<int> areas;
    areas.reserve(detections.size());
    
    for (const auto& detection : detections) {
        areas.push_back(detection.area);
    }
    
    return areas;
}
