#ifndef DUCKDETECTORBASE_H
#define DUCKDETECTORBASE_H

#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/core/mat.hpp>

class DuckDetectorBase
{
public:
    virtual ~DuckDetectorBase();
    virtual cv::Mat detect(const cv::Mat & image) = 0;

    // Accessors for detected duck data
    const std::vector<cv::Point2f>& getCentroids() const { return centroids_; }
    const std::vector<int>& getAreas() const { return areas_; }

public:
    void preprocess(const cv::Mat & img, std::vector<float> & input_tensor_values);
    void postprocess(const std::vector<float> & class_scores_data, const std::vector<float> & mask_probs_data,
                     const std::vector<std::int64_t> & class_scores_shape, const std::vector<std::int64_t> & mask_probs_shape,
                     cv::Mat & imgout);

protected:
    std::vector<cv::Point2f> centroids_;
    std::vector<int> areas_;
};

#endif
