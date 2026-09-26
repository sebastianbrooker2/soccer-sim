#ifndef DUCKDETECTIONCACHE_H
#define DUCKDETECTIONCACHE_H

#include <vector>
#include <string>
#include <opencv2/core.hpp>

/**
 * @brief Structure to hold duck detection data for a single duck
 */
struct DuckDetection {
    float centroid_x;    // x coordinate in pixels
    float centroid_y;    // y coordinate in pixels
    int32_t area;        // area in pixels
};

/**
 * @brief Class to load and access pre-computed duck detections from a binary cache file
 * 
 * The binary cache file format is:
 * - For each frame:
 *   - num_detections (uint32_t): number of ducks detected
 *   - For each detection:
 *     - centroid_x (float32): x coordinate of duck centroid in pixels
 *     - centroid_y (float32): y coordinate of duck centroid in pixels  
 *     - area (int32): area of duck mask in pixels
 */
class DuckDetectionCache {
public:
    /**
     * @brief Construct a new Duck Detection Cache object
     * @param cache_path Path to the binary cache file
     */
    explicit DuckDetectionCache(const std::string& cache_path);
    
    /**
     * @brief Get the number of frames in the cache
     * @return size_t Number of frames
     */
    size_t getNumFrames() const { return frames_.size(); }
    
    /**
     * @brief Get detections for a specific frame
     * @param frame_idx Frame index (0-based)
     * @return const std::vector<DuckDetection>& Vector of detections for this frame
     */
    const std::vector<DuckDetection>& getFrame(size_t frame_idx) const;
    
    /**
     * @brief Get centroids for a specific frame (for MeasurementDuckBundle)
     * @param frame_idx Frame index (0-based)
     * @return std::vector<cv::Point2f> Vector of centroids
     */
    std::vector<cv::Point2f> getCentroids(size_t frame_idx) const;
    
    /**
     * @brief Get areas for a specific frame (for MeasurementDuckBundle)
     * @param frame_idx Frame index (0-based)
     * @return std::vector<int> Vector of areas
     */
    std::vector<int> getAreas(size_t frame_idx) const;
    
    /**
     * @brief Check if cache is loaded and valid
     * @return true if cache is valid
     */
    bool isValid() const { return !frames_.empty(); }

private:
    std::vector<std::vector<DuckDetection>> frames_;  // Detections for all frames
};

#endif // DUCKDETECTIONCACHE_H
