#include <cstdint>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "DuckDetectorBase.h"

DuckDetectorBase::~DuckDetectorBase() = default;

void DuckDetectorBase::preprocess(const cv::Mat & img, std::vector<float> & input_tensor_values) {
    cv::Mat img_rgb;
    cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);

    cv::Mat resized;
    cv::resize(img_rgb, resized, cv::Size(512, 512));

    cv::Mat float_image;
    resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

    cv::Scalar mean(0.36055567, 0.26455822, 0.1505872);
    cv::Scalar std(0.13891927, 0.10404531, 0.09613165);

    cv::Mat normalized;
    cv::subtract(float_image, mean, normalized);
    cv::divide(normalized, std, normalized);

    for (int c = 0; c < 3; ++c)
        for (int h = 0; h < 512; ++h)
            for (int w = 0; w < 512; ++w)
                input_tensor_values[c * 512 * 512 + h * 512 + w] = normalized.at<cv::Vec3f>(h, w)[c];
}

void DuckDetectorBase::postprocess(const std::vector<float> & class_scores_data,
                                  const std::vector<float> & mask_probs_data,
                                  const std::vector<std::int64_t> & class_scores_shape,
                                  const std::vector<std::int64_t> & mask_probs_shape,
                                  cv::Mat & imgout) {

    int num_queries = mask_probs_shape[1];
    int mask_height = mask_probs_shape[2];
    int mask_width = mask_probs_shape[3];
    int num_classes = class_scores_shape[2];

    cv::Mat labelMask = cv::Mat::zeros(imgout.size(), CV_32SC1);
    int label = 1;

    std::vector<int> validLabels;
    centroids_.clear();
    areas_.clear();

    for (int query = 0; query < num_queries; ++query) {
        const float * query_class_scores = class_scores_data.data() + query*num_classes;
        int predicted_class = std::distance(query_class_scores,
                                          std::max_element(query_class_scores, query_class_scores + num_classes));
        float class_score = query_class_scores[predicted_class];

        if (predicted_class == 1 && class_score > 0.5) {
            cv::Mat query_mask(mask_height, mask_width, CV_32F,
                             const_cast<float *>(mask_probs_data.data() + query*mask_height*mask_width));

            cv::Mat resized_query_mask;
            cv::resize(query_mask, resized_query_mask, imgout.size(), 0, 0, cv::INTER_LINEAR);

            cv::Mat binary_mask;
            cv::threshold(resized_query_mask, binary_mask, 0.5, 1, cv::THRESH_BINARY);
            binary_mask.convertTo(binary_mask, CV_8U);

            labelMask.setTo(label, binary_mask);
            validLabels.push_back(label);

            // Compute centroid and area
            cv::Moments moments = cv::moments(binary_mask);
            cv::Point2f centroid;
            int area = 0;

            if (moments.m00 > 0) {
                centroid.x = moments.m10 / moments.m00;
                centroid.y = moments.m01 / moments.m00;
                area = static_cast<int>(moments.m00);
            } else {
                centroid.x = 0;
                centroid.y = 0;
                area = 0;
            }

            centroids_.push_back(centroid);
            areas_.push_back(area);

            label++;
        }
    }

    // Proximity filter: remove detections too close to each other
    const int MIN_DISTANCE = 50;  // pixels - minimum distance between duck centroids
    
    std::vector<cv::Point2f> filteredCentroids;
    std::vector<int> filteredAreas;
    std::vector<int> filteredLabels;
    std::vector<bool> kept(validLabels.size(), true);
    
    // Check each pair of detections
    for (size_t i = 0; i < validLabels.size(); ++i) {
        if (!kept[i]) continue;
        
        for (size_t j = i + 1; j < validLabels.size(); ++j) {
            if (!kept[j]) continue;
            
            // Calculate distance between centroids
            float dx = centroids_[i].x - centroids_[j].x;
            float dy = centroids_[i].y - centroids_[j].y;
            float dist = std::sqrt(dx*dx + dy*dy);
            
            // If too close, keep the one with larger area
            if (dist < MIN_DISTANCE) {
                if (areas_[i] >= areas_[j]) {
                    kept[j] = false;  // Reject j, keep i
                } else {
                    kept[i] = false;  // Reject i, keep j
                    break;  // i is rejected, no need to check further
                }
            }
        }
    }
    
    // Build filtered lists (only kept detections)
    for (size_t i = 0; i < validLabels.size(); ++i) {
        if (kept[i]) {
            filteredCentroids.push_back(centroids_[i]);
            filteredAreas.push_back(areas_[i]);
            filteredLabels.push_back(validLabels[i]);
        }
    }
    
    // Update member variables with filtered results
    centroids_ = filteredCentroids;
    areas_ = filteredAreas;
    validLabels = filteredLabels;
    
    // Area and edge filters: remove small/distant ducks and edge ducks
    const int MIN_AREA = 1200;  // px² - reduced to allow more distant ducks
    const int EDGE_MARGIN = 20;  // pixels - relaxed to allow more landmarks near edges
    const int imgWidth = imgout.cols;
    const int imgHeight = imgout.rows;
    
    std::vector<cv::Point2f> finalCentroids;
    std::vector<int> finalAreas;
    std::vector<int> finalLabels;
    
    for (size_t i = 0; i < centroids_.size(); ++i) {
        // Filter by minimum area
        if (areas_[i] < MIN_AREA) continue;
        
        // Edge filter removed - allow ducks near image boundaries
        // (This was causing Landmark #12 at top-right corner to be filtered out)
        // if (centroids_[i].x < EDGE_MARGIN || 
        //     centroids_[i].y < EDGE_MARGIN ||
        //     centroids_[i].x > (imgWidth - EDGE_MARGIN) || 
        //     centroids_[i].y > (imgHeight - EDGE_MARGIN)) continue;
        
        finalCentroids.push_back(centroids_[i]);
        finalAreas.push_back(areas_[i]);
        finalLabels.push_back(validLabels[i]);
    }
    
    // Update with final filtered results
    centroids_ = finalCentroids;
    areas_ = finalAreas;
    validLabels = finalLabels;
    
    // Generate unique colors for each label
    std::vector<cv::Vec3b> colorMap(label);
    for (int i = 0; i < validLabels.size(); ++i) {
        cv::Mat color(1, 1, CV_8UC3);
        color.at<cv::Vec3b>(0, 0) = cv::Vec3b(180*i/validLabels.size(), 255, 255);
        cv::cvtColor(color, color, cv::COLOR_HSV2BGR);
        colorMap[validLabels[i]] = color.at<cv::Vec3b>(0, 0);
    }

    // Color each pixel according to its label
    for (int y = 0; y < imgout.rows; ++y) {
        for (int x = 0; x < imgout.cols; ++x) {
            int pixelLabel = labelMask.at<int>(y, x);
            if (pixelLabel > 0) {
                imgout.at<cv::Vec3b>(y, x) = colorMap[pixelLabel];
            }
        }
    }

    // Draw centroids and labels onto imgout
    for (int i = 0; i < validLabels.size(); ++i) {
        cv::Point2f centroid = centroids_[i];
        int area = areas_[i];
        int currentLabel = validLabels[i];

        // Draw centroid as a circle
        cv::circle(imgout, centroid, 5, cv::Scalar(0, 0, 0), -1); // Black filled circle
        cv::circle(imgout, centroid, 6, cv::Scalar(255, 255, 255), 2); // White outline

        // Prepare label text
        std::string labelText = "Duck " + std::to_string(currentLabel) + " (Area: " + std::to_string(area) + ")";

        // Calculate text position (offset from centroid)
        cv::Point textPos(centroid.x + 10, centroid.y - 10);

        // Ensure text stays within image bounds
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(labelText, fontFace, fontScale, thickness, &baseline);

        if (textPos.x + textSize.width > imgout.cols) {
            textPos.x = imgout.cols - textSize.width - 5;
        }
        if (textPos.y - textSize.height < 0) {
            textPos.y = textSize.height + 5;
        }

        // Draw text background rectangle for better visibility
        cv::rectangle(imgout,
                     cv::Point(textPos.x - 2, textPos.y + baseline + 2),
                     cv::Point(textPos.x + textSize.width + 2, textPos.y - textSize.height - 2),
                     cv::Scalar(255, 255, 255), -1);

        // Draw text
        cv::putText(imgout, labelText, textPos, fontFace, fontScale, cv::Scalar(0, 0, 0), thickness);
    }
}
