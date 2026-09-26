#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <filesystem>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>


void calibrateCamera(const std::filesystem::path & configPath);

// Stream operator for Chessboard (enables to_string template)
// std::ostream & operator<<(std::ostream & os, const Chessboard & chessboard);

#endif