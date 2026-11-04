#include "common_utils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace pcb {
namespace common {

namespace {

const std::map<std::string, std::string> SHORT_MAP = {
    {"Resistor", "R"},
    {"Capacitor", "C"},
    {"Electrolytic Capacitor", "EC"},
    {"IC", "IC"},
    {"Inductor", "L"},
    {"Diode", "D"},
    {"LED", "LED"},
    {"Transistor", "Q"},
    {"Oscillator", "OSC"},
    {"Connector", "CONN"},
    {"Switch/Button", "SW"},
    {"Other", "OTH"},
    {"Marker", "MK"},
    {"?", "?"}
};

const std::vector<cv::Scalar> BASE_PALETTE = {
    {0, 215, 255},   // Resistor - Gold/Yellow
    {200, 100, 0},   // Capacitor - Deep Blue
    {255, 140, 0},   // Electrolytic Capacitor - Deep Sky Blue
    {34, 34, 220},   // IC - Crimson/Red
    {0, 165, 255},   // Inductor - Orange
    {0, 140, 255},   // Diode - Dark Orange
    {50, 205, 50},   // LED - Lime Green
    {180, 105, 255}, // Transistor - Hot Pink
    {205, 90, 106},  // Oscillator - Cadet Blue
    {128, 0, 128},   // Connector - Purple
    {0, 255, 255},   // Switch/Button - Yellow
    {169, 169, 169}, // Other - Dark Gray
    {255, 255, 0}    // Marker - Cyan
};

} // namespace

bool moveToOkFolder(const std::filesystem::path& sourcePath) {
    const std::filesystem::path okFolder = sourcePath.parent_path() / "ok";
    const std::filesystem::path destination = okFolder / sourcePath.filename();
    std::error_code error;

    std::filesystem::create_directories(okFolder, error);
    if (error) {
        std::cerr << "ERRORE: impossibile creare la cartella " << okFolder
                  << ": " << error.message() << std::endl;
        return false;
    }
    if (std::filesystem::exists(destination)) {
        std::cerr << "ERRORE: esiste già un file in " << destination
                  << "; l'originale non verrà spostato." << std::endl;
        return false;
    }

    std::filesystem::rename(sourcePath, destination, error);
    if (error) {
        std::cerr << "ERRORE: impossibile spostare " << sourcePath
                  << " in " << destination << ": " << error.message() << std::endl;
        return false;
    }
    return true;
}

float calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    cv::Rect2f inter = a & b;
    float inter_area = inter.area();
    if (inter_area <= 0.0f) return 0.0f;
    float union_area = a.area() + b.area() - inter_area;
    return (union_area > 0.0f) ? (inter_area / union_area) : 0.0f;
}

float calculateOverlap(const cv::Rect2f& a, const cv::Rect2f& b) {
    cv::Rect2f inter = a & b;
    float inter_area = inter.area();
    if (inter_area <= 0.0f) return 0.0f;
    float min_area = std::min(a.area(), b.area());
    return (min_area > 0.0f) ? (inter_area / min_area) : 0.0f;
}

std::vector<std::string> getDefaultClasses() {
    return {
        "Resistor",
        "Capacitor",
        "Electrolytic Capacitor",
        "IC",
        "Inductor",
        "Diode",
        "LED",
        "Transistor",
        "Oscillator",
        "Connector",
        "Switch/Button",
        "Other",
        "Marker"
    };
}

std::string getShortLabel(const std::string& full_label) {
    auto it = SHORT_MAP.find(full_label);
    if (it != SHORT_MAP.end()) {
        return it->second;
    }
    return (full_label.size() >= 3) ? full_label.substr(0, 3) : full_label;
}

std::map<std::string, cv::Scalar> getPalette(const std::vector<std::string>& labels) {
    std::map<std::string, cv::Scalar> palette;
    for (size_t i = 0; i < labels.size(); ++i) {
        palette[labels[i]] = BASE_PALETTE[i % BASE_PALETTE.size()];
    }
    palette["?"] = cv::Scalar(180, 180, 180);
    return palette;
}

cv::Mat renderSideLegend(
    int targetHeight,
    const std::vector<std::pair<std::string, int>>& label_counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats,
    int panelWidth) {

    int line_h = 40;
    int needed_h = static_cast<int>((label_counts.size() + extra_stats.size() + 5) * line_h);
    int panel_h = std::max(targetHeight, needed_h);
    int panel_w = panelWidth;

    cv::Mat panel(panel_h, panel_w, CV_8UC3, cv::Scalar(22, 22, 22));

    // Intestazione
    cv::rectangle(panel, cv::Rect(0, 0, panel_w, 60), cv::Scalar(35, 35, 35), cv::FILLED);
    cv::putText(panel, title, cv::Point(16, 40), cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // Totale
    std::string total_str = "Totale: " + std::to_string(total_count);
    cv::putText(panel, total_str, cv::Point(16, 88), cv::FONT_HERSHEY_SIMPLEX, 0.68, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    cv::line(panel, cv::Point(16, 102), cv::Point(panel_w - 16, 102), cv::Scalar(70, 70, 70), 1);

    int cur_y = 132;

    // Statistiche extra (se presenti)
    if (!extra_stats.empty()) {
        for (const auto& [stat_name, stat_val] : extra_stats) {
            cv::Scalar stat_color(200, 200, 200);
            if (stat_name.find("FUSED") != std::string::npos || stat_name.find("EXACT") != std::string::npos) {
                stat_color = cv::Scalar(50, 205, 50); // Verde
            } else if (stat_name.find("CORRECTED") != std::string::npos) {
                stat_color = cv::Scalar(0, 215, 255); // Giallo
            } else if (stat_name.find("RECOVERED") != std::string::npos) {
                stat_color = cv::Scalar(255, 215, 0); // Ciano
            } else if (stat_name.find("DL") != std::string::npos) {
                stat_color = cv::Scalar(0, 165, 255);
            } else if (stat_name.find("CV") != std::string::npos) {
                stat_color = cv::Scalar(255, 140, 0);
            }

            cv::circle(panel, cv::Point(24, cur_y - 5), 5, stat_color, -1, cv::LINE_AA);
            std::string row_str = stat_name + ": " + std::to_string(stat_val);
            cv::putText(panel, row_str, cv::Point(38, cur_y), cv::FONT_HERSHEY_SIMPLEX, 0.54, stat_color, 1, cv::LINE_AA);
            cur_y += 28;
        }
        cv::line(panel, cv::Point(16, cur_y), cv::Point(panel_w - 16, cur_y), cv::Scalar(70, 70, 70), 1);
        cur_y += 28;
    }

    // Elenco Classi
    for (const auto& [label, count] : label_counts) {
        if (count <= 0 && cur_y > panel_h - 20) continue;

        cv::Scalar color(180, 180, 180);
        auto it = palette.find(label);
        if (it != palette.end()) {
            color = it->second;
        }

        // Quadratino colorato della classe
        cv::rectangle(panel, cv::Rect(18, cur_y - 16, 18, 18), color, cv::FILLED);
        cv::rectangle(panel, cv::Rect(18, cur_y - 16, 18, 18), cv::Scalar(255, 255, 255), 1);

        std::string label_text = label + "  (" + std::to_string(count) + ")";
        cv::putText(panel, label_text, cv::Point(46, cur_y - 2), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);

        cur_y += 32;
    }

    return panel;
}

cv::Mat renderSideLegend(
    int targetHeight,
    const std::map<std::string, int>& counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats,
    int panelWidth) {

    std::vector<std::pair<std::string, int>> sorted_counts;
    for (const auto& p : counts) {
        if (p.second > 0) sorted_counts.push_back(p);
    }
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    return renderSideLegend(targetHeight, sorted_counts, total_count, palette, title, extra_stats, panelWidth);
}

} // namespace common
} // namespace pcb

