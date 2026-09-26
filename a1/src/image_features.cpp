#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "image_features.h"

PointFeature::PointFeature()
    : score(0)
    , x(0)
    , y(0)
{}

PointFeature::PointFeature(const double & score_, const double & x_, const double & y_)
    : score(score_)
    , x(x_)
    , y(y_)
{}

bool PointFeature::operator<(const PointFeature & other) const
{
    return (score > other.score);
}

std::vector<PointFeature> detectFeatures(const cv::Mat & img, const int & maxNumFeatures)
{
    std::vector<PointFeature> features;

    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }

    // 2) Response image (Shi–Tomasi)
    cv::Mat eigenval_response;              // float image
    const int blockSize = 5;
    const int ksize     = 5;
    cv::cornerMinEigenVal(gray, eigenval_response, blockSize, ksize);

    // 3) Threshold relative to max response (same style as your code)
    double minVal, maxVal;
    cv::minMaxLoc(eigenval_response, &minVal, &maxVal);
    const float threshold = 0.1f * static_cast<float>(maxVal);

    // 4) Collect pixels above threshold as features
    for (int y = 0; y < eigenval_response.rows; ++y) {
        for (int x = 0; x < eigenval_response.cols; ++x) {
            const float resp = eigenval_response.at<float>(y, x);
            if (resp > threshold) {
                PointFeature f;
                f.x     = x;
                f.y     = y;
                f.score = resp;
                features.push_back(f);
            }
        }
    }

    // 5) Sort by score (your PointFeature comparator orders highest-first)
    std::sort(features.begin(), features.end());

    // 6) Cap to maxNumFeatures
    if (features.size() > static_cast<size_t>(maxNumFeatures)) {
        features.resize(maxNumFeatures);
    }

    return features;
}
