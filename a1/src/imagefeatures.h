#ifndef IMAGEFEATURES_H
#define IMAGEFEATURES_H 

#include <opencv2/core.hpp>

cv::Mat detectAndDrawHarris(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawShiAndTomasi(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawFAST(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures);
cv::Mat detectAndDrawArUco(const cv::Mat & img, int maxNumFeatures, 
                           std::vector<int> & markerIds, 
                           std::vector<std::vector<cv::Point2f>> & markerCorners);

#endif