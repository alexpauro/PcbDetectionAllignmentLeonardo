#pragma once

#include "aoi_verification.hpp"
#include "common_utils.hpp"
#include <map>
#include <string>
#include <algorithm>
#include <cctype>

namespace pcb {

// Palette colori ufficiale per le classi di componenti (BGR)
inline const std::map<std::string, cv::Scalar> CLASS_COLORS = {
    {"Resistor",               cv::Scalar(0, 165, 255)},    // Arancione
    {"Capacitor",              cv::Scalar(255, 144, 30)},   // Blu chiaro
    {"Electrolytic Capacitor", cv::Scalar(180, 105, 255)},  // Rosa vivo
    {"IC",                     cv::Scalar(0, 215, 255)},    // Oro / Giallo caldo
    {"Inductor",               cv::Scalar(205, 90, 106)},   // Viola cadetto
    {"Diode",                  cv::Scalar(50, 205, 50)},    // Verde lime
    {"LED",                    cv::Scalar(0, 255, 127)},    // Verde primavera
    {"Transistor",             cv::Scalar(238, 130, 238)},  // Violetto
    {"Oscillator",             cv::Scalar(147, 20, 255)},   // Rosa porpora
    {"Connector",              cv::Scalar(255, 191, 0)},    // Deep sky blue
    {"Switch/Button",          cv::Scalar(0, 69, 255)},     // Rosso arancio
    {"Other",                  cv::Scalar(128, 128, 128)},  // Grigio
    {"Marker",                 cv::Scalar(255, 255, 0)},    // Ciano
    {"?",                      cv::Scalar(80, 80, 80)}      // Grigio scuro
};

// Colori di stato per la verifica AOI (BGR)
inline const cv::Scalar COLOR_EXACT_MATCH     = cv::Scalar(40, 200, 40);   // Verde brillante
inline const cv::Scalar COLOR_CLASS_CORRECTED = cv::Scalar(0, 160, 255);   // Ambra / Arancio
inline const cv::Scalar COLOR_RECOVERED       = cv::Scalar(240, 220, 20);  // Ciano dorato

inline std::string trim(const std::string& s) {
    auto start = s.begin();
    while (start != s.end() && std::isspace(static_cast<unsigned char>(*start))) {
        start++;
    }
    auto end = s.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(static_cast<unsigned char>(*end)));
    return (start < end + 1) ? std::string(start, end + 1) : "";
}

inline std::string toUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

} // namespace pcb

