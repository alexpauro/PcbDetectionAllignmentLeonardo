#pragma once

#include "output_processor.hpp"
#include "common_utils.hpp"
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

namespace pcb {

// Helper per conversione stringa in minuscolo
inline std::string toLower(const std::string& str) {
    std::string s = str;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

// Colori fissi per la visualizzazione grafica
inline const cv::Scalar COLOR_FUSED(0, 255, 0);          // Verde brillante: FUSED_DL_CV
inline const cv::Scalar COLOR_DL_ONLY(255, 200, 0);      // Ciano/Azzurro: DL_ONLY
inline const cv::Scalar COLOR_CV_ONLY(255, 0, 255);      // Magenta/Viola: CV_ONLY
inline const cv::Scalar COLOR_SIMILARITY(0, 165, 255);   // Arancione brillante: SIMILARITY_DISCOVERED
inline const cv::Scalar COLOR_ELEMENT(120, 255, 0);      // Verde chiaro Step 1

// Feature visuali estratte con Computer Vision per comparazione di componenti
struct ComponentVisualFeature {
    cv::Mat patch_gray;
    cv::Mat hist_hsv;
    float width = 0.0f;
    float height = 0.0f;
    float area = 0.0f;
    ShapeType shape = ShapeType::RECTANGLE;
    bool valid = false;
};

ComponentVisualFeature extractVisualFeature(const cv::Mat& image, const FinalComponent& comp);

bool areFeaturesIdentical(const ComponentVisualFeature& f1,
                          const ComponentVisualFeature& f2,
                          float geom_tol,
                          float ncc_thresh,
                          float hist_thresh);

} // namespace pcb

