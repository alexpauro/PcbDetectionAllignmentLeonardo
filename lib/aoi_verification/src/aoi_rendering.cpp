/**
 * @file aoi_rendering.cpp
 * @brief Funzioni di visualizzazione e sovrimpressione grafica dei risultati di verifica AOI.
 */

#include "aoi_verification.hpp"
#include "aoi_internal.hpp"
#include "common_utils.hpp"

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>

namespace pcb {

cv::Mat AoiVerificationProcessor::createSideLegend(
    int targetHeight,
    const std::map<std::string, int>& counts,
    int totalComps,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extraStats) {

    // Delega alla libreria centralizzata pcb::common
    return pcb::common::renderSideLegend(targetHeight, counts, totalComps, palette, title, extraStats, 340);
}

cv::Mat AoiVerificationProcessor::renderBestAlignmentOverlay(
    const cv::Mat& pcbImage,
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    const AoiAlignmentTransform& transform) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();

    // NOTA: I componenti extra NON vengono disegnati come richiesto!
    // Disegna solo i punti teorici AOI e i collegamenti ai componenti associati
    int matchedCount = 0;
    for (const auto& a : aoiComps) {
        cv::Point2f nom = transform.aoiToPixel(a.x_mm, a.y_mm);

        cv::Scalar nomCol = COLOR_EXACT_MATCH;
        if (!a.matched) {
            nomCol = cv::Scalar(0, 0, 255);
        }

        cv::drawMarker(canvas, nom, nomCol, cv::MARKER_CROSS, 8, 1, cv::LINE_AA);

        if (a.matched && a.matched_detection_id > 0) {
            matchedCount++;
            for (const auto& d : detections) {
                if (d.id == a.matched_detection_id) {
                    // Disegna solo il box del componente associato
                    cv::rectangle(canvas, d.box, cv::Scalar(80, 140, 255), 1, cv::LINE_AA);
                    cv::line(canvas, nom, d.center, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                    break;
                }
            }
        }

        cv::putText(canvas, a.ref_des, cv::Point(static_cast<int>(nom.x + 3), static_cast<int>(nom.y - 3)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.28, nomCol, 1, cv::LINE_AA);
    }

    // Banner superiore scuro
    cv::Rect bannerRect(0, 0, canvas.cols, 42);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.85, banner, 0.15, 0.0, banner);

    std::string headerTxt = "Corrispondenza Coordinate AOI vs Immagine Reale (Correzione Distorsione Ottica): " +
        std::to_string(matchedCount) + "/" + std::to_string(aoiComps.size()) + " (" +
        transform.orientation_name + ", Offset medio: " +
        std::to_string(transform.avg_error_mm).substr(0, 5) + " mm)";
    cv::putText(canvas, headerTxt, cv::Point(14, 27), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    return canvas;
}

cv::Mat AoiVerificationProcessor::renderVerifiedTypesOverlay(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& verifiedComps) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();
    std::map<std::string, int> counts;
    int drawnCount = 0;

    for (const auto& c : verifiedComps) {
        // COMPONENTI EXTRA NON RAPPRESENTARLI!
        if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) {
            continue;
        }

        drawnCount++;
        counts[c.label]++;

        cv::Scalar col = cv::Scalar(180, 180, 180);
        auto it = CLASS_COLORS.find(c.label);
        if (it != CLASS_COLORS.end()) col = it->second;

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(2, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
        }
        cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        // REQUISITO UTENTE: Nelle immagini di output riporta solo il codice componente (es. R301 non R301-RESISTOR)
        std::string txt = (!c.ref_des.empty() && c.ref_des != "N/A") ? c.ref_des : c.label;
        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 6));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 14);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(18, 18, 18), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect bannerRect(0, 0, canvas.cols, 38);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, banner, 0.18, 0.0, banner);
    std::string btext = "Verifica AOI: Riconoscimento DL + Sovrapposizione CV (Extra esclusi)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // Leggenda laterale
    cv::Mat legend = createSideLegend(canvas.rows, counts, drawnCount,
                                      CLASS_COLORS, "Classi Verificate", {});

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

cv::Mat AoiVerificationProcessor::renderVerifiedElementsOverlay(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& verifiedComps) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();
    int nExact = 0, nCorrected = 0, nRecovered = 0;

    for (const auto& c : verifiedComps) {
        // COMPONENTI EXTRA NON RAPPRESENTARLI!
        if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) {
            continue;
        }

        cv::Scalar col;
        std::string stStr;

        if (c.status == AoiMatchStatus::MATCH_EXACT_CLASS) {
            col = COLOR_EXACT_MATCH;
            stStr = "OK";
            nExact++;
        } else if (c.status == AoiMatchStatus::CLASS_CORRECTED) {
            col = COLOR_CLASS_CORRECTED;
            stStr = "CORRETTO";
            nCorrected++;
        } else {
            col = COLOR_RECOVERED;
            stStr = "RECUPERATO";
            nRecovered++;
        }

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(2, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
        }
        cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        // REQUISITO UTENTE: Nelle immagini di output riporta solo il codice componente (es. R301)
        std::string txt = (!c.ref_des.empty() && c.ref_des != "N/A") ? c.ref_des : ("#" + std::to_string(c.id));
        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.34, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 6));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 14);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(18, 18, 18), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.34, col, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect bannerRect(0, 0, canvas.cols, 38);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, banner, 0.18, 0.0, banner);
    std::string btext = "Verifica Elementi AOI: Verde=OK, Arancione=Corretto, Ciano=Recuperato (Extra esclusi)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> extraStats = {
        {"Match Esatti (OK)", nExact},
        {"Classi Corrette",   nCorrected},
        {"Recuperati da AOI", nRecovered}
    };

    std::map<std::string, cv::Scalar> statusPalette = {
        {"Match Esatti (OK)", COLOR_EXACT_MATCH},
        {"Classi Corrette",   COLOR_CLASS_CORRECTED},
        {"Recuperati da AOI", COLOR_RECOVERED}
    };

    int totalNominalRepresented = nExact + nCorrected + nRecovered;
    cv::Mat legend = createSideLegend(canvas.rows, {}, totalNominalRepresented,
                                      statusPalette, "Stato Riconoscimento", extraStats);

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

} // namespace pcb

