#include "component_segmentation.hpp"
#include "cv_component_detector.hpp"
#include "common_utils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace pcb {

ComponentSegmentationProcessor::ComponentSegmentationProcessor() = default;

// =========================================================================
// MAPPATURA COORDINATE CON CORREZIONE DELLA DISTORSIONE OTTICA
// =========================================================================

cv::Point2d ComponentSegmentationProcessor::pixelToAoi(
    const cv::Point2f& pt_px,
    const AoiAlignmentTransform& transform) {

    if (!transform.valid) {
        return cv::Point2d(0.0, 0.0);
    }

    // 1. Modello di distorsione ottica e prospettica tramite inversa dell'omografia 3x3
    if (transform.has_distortion_model && !transform.homography_3x3.empty()) {
        cv::Mat H_inv = transform.homography_inv_3x3;
        if (H_inv.empty()) {
            cv::invert(transform.homography_3x3, H_inv);
        }
        if (!H_inv.empty()) {
            std::vector<cv::Point2f> src = { pt_px };
            std::vector<cv::Point2f> dst;
            cv::perspectiveTransform(src, dst, H_inv);
            if (!dst.empty()) {
                return cv::Point2d(static_cast<double>(dst[0].x), static_cast<double>(dst[0].y));
            }
        }
    }

    // 2. Modello lineare rigido base inverso (fallback se omografia non attiva)
    const double s = (transform.scale_px_per_mm > 1e-5) ? transform.scale_px_per_mm : 1.0;
    double xr = (pt_px.x - transform.tx) / s;
    double yr = (pt_px.y - transform.ty) / s;

    if (std::abs(transform.residual_angle_deg) > 1e-4) {
        double rad = -transform.residual_angle_deg * CV_PI / 180.0;
        double cos_r = std::cos(rad);
        double sin_r = std::sin(rad);
        double x_unrot = cos_r * xr - sin_r * yr;
        double y_unrot = sin_r * xr + cos_r * yr;
        xr = x_unrot;
        yr = y_unrot;
    }

    double det = transform.mxx * transform.myy - transform.mxy * transform.myx;
    if (std::abs(det) < 1e-6) det = 1.0;

    double aoi_x = (transform.myy * xr - transform.mxy * yr) / det;
    double aoi_y = (-transform.myx * xr + transform.mxx * yr) / det;

    return cv::Point2d(aoi_x, aoi_y);
}

cv::Point2f ComponentSegmentationProcessor::aoiToPixel(
    const cv::Point2d& pt_mm,
    const AoiAlignmentTransform& transform) {
    return transform.aoiToPixel(pt_mm.x, pt_mm.y);
}

double ComponentSegmentationProcessor::normalizeAngleDiff(double realAngleDeg, double nominalAngleDeg) {
    double diff = realAngleDeg - nominalAngleDeg;

    // Riduci in [-180, 180)
    while (diff <= -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;

    // Simmetria del package rettangolare a 180°
    if (diff > 90.0) diff -= 180.0;
    else if (diff < -90.0) diff += 180.0;

    // Se l'orientamento è invertito di 90° (per via dell'ordine w/h di minAreaRect), riduci a [-45, 45]
    if (diff > 45.0) diff -= 90.0;
    else if (diff < -45.0) diff += 90.0;

    return diff;
}

// =========================================================================
// PIPELINE PRINCIPALE DI SEGMENTAZIONE E RIFINITURA BORDI
// =========================================================================

ComponentSegmentationResult ComponentSegmentationProcessor::process(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& inputComponents,
    const AoiAlignmentTransform& transform,
    const ComponentSegmentationConfig& config,
    const cv::Mat& precomputedComponentsMask) {

    ComponentSegmentationResult result;
    result.success = false;
    result.summary.total_input_components = static_cast<int>(inputComponents.size());

    if (pcbImage.empty() || inputComponents.empty()) {
        std::cerr << "[SEGMENTATION] Immagine PCB vuota o nessun componente in ingresso." << std::endl;
        return result;
    }

    const double s = (transform.scale_px_per_mm > 1e-5) ? transform.scale_px_per_mm : 1.0;
    const int imgW = pcbImage.cols;
    const int imgH = pcbImage.rows;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "=== AVVIO SEGMENTAZIONE PRECISA DEI COMPONENTI PCB ===" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "[SEGMENTATION] Totale componenti in ingresso: " << inputComponents.size() << std::endl;

    // 1. Estrazione o utilizzo della maschera dei componenti mediante CvComponentDetector
    cv::Mat componentsMask;
    if (!precomputedComponentsMask.empty() && precomputedComponentsMask.size() == pcbImage.size()) {
        componentsMask = precomputedComponentsMask.clone();
        std::cout << "[SEGMENTATION] Utilizzo maschera componenti multi-esposizione precalcolata da CvComponentDetector." << std::endl;
    } else {
        std::cout << "[SEGMENTATION] Esecuzione algoritmo CvComponentDetector con tutti i suoi filtri..." << std::endl;
        CvComponentDetector cvDetector;
        CvDetectionConfig cvCfg;
        cvCfg.show_windows = false;
        CvDetectionResult cvRes = cvDetector.detect(pcbImage, cvCfg);
        componentsMask = cvRes.components_mask.clone();
    }

    // Filtra i componenti escludendo solo eventuali UNMATCHED extra non appartenenti alla distinta AOI
    std::vector<VerifiedPcbComponent> validInputs;
    for (const auto& comp : inputComponents) {
        if (comp.status == AoiMatchStatus::UNMATCHED_DETECTION) continue;
        validInputs.push_back(comp);
    }

    // =========================================================================
    // REQUISITO: "parti dai componenti più grandi ai componenti più piccoli"
    // =========================================================================
    auto getRealisticArea = [&](const VerifiedPcbComponent& c) -> float {
        float nomA = 0.0f;
        if (c.nominal_width_mm > 0.05 && c.nominal_height_mm > 0.05) {
            nomA = static_cast<float>(c.nominal_width_mm * c.nominal_height_mm * s * s);
        }
        float boxA = c.box.width * c.box.height;
        if (nomA > 1.0f) {
            if (boxA > nomA * 2.5f || boxA < nomA * 0.25f) return nomA;
            return std::max(nomA, boxA);
        }
        return std::max(boxA, c.area);
    };

    std::sort(validInputs.begin(), validInputs.end(), [&](const VerifiedPcbComponent& a, const VerifiedPcbComponent& b) {
        return getRealisticArea(a) > getRealisticArea(b);
    });

    std::cout << "[SEGMENTATION] Componenti ordinati per area decrescente (da max a min)." << std::endl;

    std::vector<SegmentedComponent> processedComps;
    std::map<std::string, std::vector<cv::Size2f>> partSizesPxMap;

    for (const auto& inComp : validInputs) {
        SegmentedComponent seg;
        seg.id = inComp.id;
        seg.ref_des = inComp.ref_des;
        seg.label = inComp.label;
        seg.part_number = inComp.aoi_part_number;
        seg.outline = inComp.aoi_outline;
        seg.nominal_x_mm = (inComp.nominal_center_px.x > 0) ? (inComp.center.x / s) : 0.0;
        seg.nominal_center_px = inComp.nominal_center_px;
        seg.notes = inComp.notes;

        // Determina il centro di riferimento del componente trovato
        cv::Point2f center = inComp.center;
        if (center.x <= 1.0f && center.y <= 1.0f) {
            center = inComp.nominal_center_px;
        }

        // Dimensioni attese della box
        float nomW_px = static_cast<float>(inComp.nominal_width_mm * s);
        float nomH_px = static_cast<float>(inComp.nominal_height_mm * s);
        float boxW = inComp.box.width;
        float boxH = inComp.box.height;

        if (nomW_px > 3.0f && nomH_px > 3.0f) {
            if (boxW > nomW_px * 2.0f || boxH > nomH_px * 2.0f || boxW <= 4.0f || boxH <= 4.0f) {
                boxW = nomW_px;
                boxH = nomH_px;
            }
        } else {
            boxW = std::max(boxW, 10.0f);
            boxH = std::max(boxH, 10.0f);
        }

        // =========================================================================
        // REQUISITO: "prendi la box del componente trovato (con un certo margine di
        // tolleranza) e applichi gli stessi algoritmi per trovare il bordo esatto"
        // =========================================================================
        std::vector<cv::Point> collectedPoints;
        int expansionLevel = 0;

        // Tentativo 1: Margine iniziale di tolleranza
        float margin1 = std::max(5.0f, std::max(boxW, boxH) * config.initial_margin_ratio);
        int rx1 = std::max(0, static_cast<int>(std::floor(center.x - boxW * 0.5f - margin1)));
        int ry1 = std::max(0, static_cast<int>(std::floor(center.y - boxH * 0.5f - margin1)));
        int rw1 = std::min(imgW - rx1, static_cast<int>(std::ceil(boxW + 2.0f * margin1)));
        int rh1 = std::min(imgH - ry1, static_cast<int>(std::ceil(boxH + 2.0f * margin1)));

        if (rw1 > 3 && rh1 > 3) {
            cv::Rect roiRect(rx1, ry1, rw1, rh1);
            cv::Mat roiMask = componentsMask(roiRect);
            std::vector<std::vector<cv::Point>> roiContours;
            cv::findContours(roiMask, roiContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

            cv::Point2f cInRoi(center.x - rx1, center.y - ry1);
            float innerMargin = std::max(boxW, boxH) * 0.65f + 4.0f;

            for (const auto& c : roiContours) {
                double a = cv::contourArea(c);
                if (a < config.min_contour_area) continue;

                cv::Rect b = cv::boundingRect(c);
                cv::Point2f bc(b.x + b.width * 0.5f, b.y + b.height * 0.5f);
                if (cv::norm(bc - cInRoi) <= innerMargin) {
                    for (const auto& pt : c) {
                        collectedPoints.push_back(cv::Point(pt.x + rx1, pt.y + ry1));
                    }
                }
            }
            if (!collectedPoints.empty()) {
                expansionLevel = 0;
            }
        }

        // Tentativo 2: Casella espansa se necessario
        if (collectedPoints.empty()) {
            float margin2 = std::max(10.0f, std::max(boxW, boxH) * config.expanded_margin_ratio);
            int rx2 = std::max(0, static_cast<int>(std::floor(center.x - boxW * 0.5f - margin2)));
            int ry2 = std::max(0, static_cast<int>(std::floor(center.y - boxH * 0.5f - margin2)));
            int rw2 = std::min(imgW - rx2, static_cast<int>(std::ceil(boxW + 2.0f * margin2)));
            int rh2 = std::min(imgH - ry2, static_cast<int>(std::ceil(boxH + 2.0f * margin2)));

            if (rw2 > 3 && rh2 > 3) {
                cv::Rect roiRect(rx2, ry2, rw2, rh2);
                cv::Mat roiMask = componentsMask(roiRect);
                std::vector<std::vector<cv::Point>> roiContours;
                cv::findContours(roiMask, roiContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

                cv::Point2f cInRoi(center.x - rx2, center.y - ry2);
                float innerMargin = std::max(boxW, boxH) * 0.90f + 6.0f;

                for (const auto& c : roiContours) {
                    double a = cv::contourArea(c);
                    if (a < config.min_contour_area) continue;

                    cv::Rect b = cv::boundingRect(c);
                    cv::Point2f bc(b.x + b.width * 0.5f, b.y + b.height * 0.5f);
                    if (cv::norm(bc - cInRoi) <= innerMargin) {
                        for (const auto& pt : c) {
                            collectedPoints.push_back(cv::Point(pt.x + rx2, pt.y + ry2));
                        }
                    }
                }
                if (!collectedPoints.empty()) {
                    expansionLevel = 1;
                }
            }
        }

        // =========================================================================
        // REQUISITO: "se i componenti erano stati individuati significa che qualcosa c'è"
        // I componenti NON vengono eliminati! Se il contorno CV locale è isolato, si calcola
        // la box aderente; altrimenti si preserva il box originario trovato rifinito con le piazzole.
        // =========================================================================
        cv::RotatedRect tightBox;
        std::vector<cv::Point> finalContour;

        if (!collectedPoints.empty()) {
            std::vector<cv::Point> hull;
            cv::convexHull(collectedPoints, hull);
            tightBox = cv::minAreaRect(hull);
            finalContour = hull;
            seg.found = true;
            seg.search_expansion_level = expansionLevel;
            if (expansionLevel == 0) result.summary.found_in_initial_roi++;
            else result.summary.found_in_expanded_roi++;
            result.summary.successfully_segmented++;
        } else {
            // Mantiene il riquadro originario validato
            tightBox = inComp.rotated_box;
            if (tightBox.size.width <= 0.0f || tightBox.size.height <= 0.0f) {
                tightBox = cv::RotatedRect(center, cv::Size2f(boxW, boxH), 0.0f);
            }
            cv::Point2f pts[4];
            tightBox.points(pts);
            for (int j = 0; j < 4; ++j) {
                finalContour.push_back(cv::Point(static_cast<int>(std::round(pts[j].x)),
                                                 static_cast<int>(std::round(pts[j].y))));
            }
            seg.found = true;
            seg.search_expansion_level = 2;
            result.summary.preserved_original_box++;
        }

        // Assicura orientamento canonico iniziale (w >= h)
        if (tightBox.size.width < tightBox.size.height) {
            std::swap(tightBox.size.width, tightBox.size.height);
            tightBox.angle += 90.0f;
        }

        seg.contour = finalContour;
        seg.rotated_box = tightBox;
        seg.bounding_box = tightBox.boundingRect2f();
        seg.pixel_center = tightBox.center;
        seg.box_width_px = tightBox.size.width;
        seg.box_height_px = tightBox.size.height;
        seg.pixel_angle_deg = tightBox.angle;
        seg.area_px = static_cast<float>(cv::contourArea(finalContour));
        seg.box_width_mm = seg.box_width_px / s;
        seg.box_height_mm = seg.box_height_px / s;

        if (!seg.part_number.empty()) {
            partSizesPxMap[seg.part_number].push_back(tightBox.size);
        }

        processedComps.push_back(seg);
    }

    std::cout << "[SEGMENTATION] Rifinitura bordi completata: " << processedComps.size()
              << " componenti confermati e modellati su PCB." << std::endl;

    // =========================================================================
    // REQUISITO: "ricorda che componenti con stesso codice produttore hanno
    //             lo stesso package e quindi devono avere lo stesso ingombro"
    // =========================================================================
    if (config.harmonize_part_number_sizes) {
        std::cout << "[SEGMENTATION] Armonizzazione ingombri package per medesimo codice produttore..." << std::endl;

        for (const auto& entry : partSizesPxMap) {
            const std::string& partNumber = entry.first;
            const auto& sizes = entry.second;
            if (sizes.empty()) continue;

            std::vector<float> widths, heights;
            for (const auto& sz : sizes) {
                widths.push_back(std::max(sz.width, sz.height));
                heights.push_back(std::min(sz.width, sz.height));
            }
            std::sort(widths.begin(), widths.end());
            std::sort(heights.begin(), heights.end());

            float medianW = widths[widths.size() / 2];
            float medianH = heights[heights.size() / 2];

            result.summary.canonical_part_sizes_px[partNumber] = cv::Size2f(medianW, medianH);
            result.summary.canonical_part_sizes_mm[partNumber] = cv::Size2d(medianW / s, medianH / s);
        }

        // Applica l'ingombro armonizzato a ciascun componente con medesimo part_number
        for (auto& comp : processedComps) {
            if (comp.part_number.empty()) continue;
            auto it = result.summary.canonical_part_sizes_px.find(comp.part_number);
            if (it != result.summary.canonical_part_sizes_px.end()) {
                cv::Size2f canonSize = it->second;
                bool keepSwap = (comp.rotated_box.size.width < comp.rotated_box.size.height);
                if (keepSwap) {
                    comp.rotated_box.size = cv::Size2f(canonSize.height, canonSize.width);
                } else {
                    comp.rotated_box.size = canonSize;
                }
                comp.box_width_px = comp.rotated_box.size.width;
                comp.box_height_px = comp.rotated_box.size.height;
                comp.box_width_mm = comp.box_width_px / s;
                comp.box_height_mm = comp.box_height_px / s;
                comp.bounding_box = comp.rotated_box.boundingRect2f();
            }
        }
    }

    // =========================================================================
    // REQUISITO: Calcolo reale posizione in mm (con distorsione) ed errori
    // "il componente è circa nella posizione nominale, può essere traslato di
    // pochi decimi di mm (1-2-3) e ruotato di massimo qualche grado"
    // =========================================================================
    double sumErrDistMm = 0.0;
    double sumErrDistPx = 0.0;
    double sumErrRotDeg = 0.0;
    int validErrorCount = 0;

    for (auto& comp : processedComps) {
        for (const auto& orig : validInputs) {
            if (orig.ref_des == comp.ref_des) {
                cv::Point2d nomMm = pixelToAoi(orig.nominal_center_px, transform);
                comp.nominal_x_mm = nomMm.x;
                comp.nominal_y_mm = nomMm.y;
                comp.nominal_center_px = orig.nominal_center_px;
                comp.nominal_rot_deg = orig.rotated_box.angle;
                break;
            }
        }

        // Calcola reale posizione in mm tramite inversa dell'omografia (distorsione ottica compensata)
        cv::Point2d realMm = pixelToAoi(comp.pixel_center, transform);
        comp.real_x_mm = realMm.x;
        comp.real_y_mm = realMm.y;

        comp.error_x_mm = comp.real_x_mm - comp.nominal_x_mm;
        comp.error_y_mm = comp.real_y_mm - comp.nominal_y_mm;
        comp.error_dist_mm = std::hypot(comp.error_x_mm, comp.error_y_mm);
        comp.error_dist_px = comp.error_dist_mm * s;

        comp.error_rot_deg = normalizeAngleDiff(comp.pixel_angle_deg, comp.nominal_rot_deg);
        comp.real_rot_deg = comp.nominal_rot_deg + comp.error_rot_deg;

        sumErrDistMm += comp.error_dist_mm;
        sumErrDistPx += comp.error_dist_px;
        sumErrRotDeg += std::abs(comp.error_rot_deg);
        validErrorCount++;

        result.summary.class_distribution[comp.label]++;
    }

    if (validErrorCount > 0) {
        result.summary.mean_error_dist_mm = sumErrDistMm / validErrorCount;
        result.summary.mean_error_dist_px = sumErrDistPx / validErrorCount;
        result.summary.mean_error_rot_deg = sumErrRotDeg / validErrorCount;
    }

    result.segmented_components = processedComps;
    result.cataloged_mask = componentsMask;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "=== RIEPILOGO FINALE RIFINITURA BORDI E INSEGUIMENTO ===" << std::endl;
    std::cout << "  - Totale componenti elaborati:     " << result.summary.total_input_components << std::endl;
    std::cout << "  - Bordi CV esattamente rifiniti:   " << result.summary.successfully_segmented << std::endl;
    std::cout << "    * Margine iniziale tolleranza:   " << result.summary.found_in_initial_roi << std::endl;
    std::cout << "    * Margine espanso tolleranza:    " << result.summary.found_in_expanded_roi << std::endl;
    std::cout << "  - Box originari validati preservati: " << result.summary.preserved_original_box << std::endl;
    std::cout << "  - Scostamento medio reale/nominale: "
              << std::fixed << std::setprecision(3) << result.summary.mean_error_dist_mm << " mm ("
              << std::setprecision(2) << result.summary.mean_error_dist_px << " px)" << std::endl;
    std::cout << "  - Errore medio rotazione residua:   "
              << std::fixed << std::setprecision(2) << result.summary.mean_error_rot_deg << "°" << std::endl;
    std::cout << "========================================================" << std::endl;

    // =========================================================================
    // RENDERING GRAFICO
    // =========================================================================
    auto palette = common::getPalette(common::getDefaultClasses());

    // 1. Immagine Contorni Reali (bordi sottili e colorati)
    cv::Mat contoursImg = pcbImage.clone();
    for (const auto& comp : processedComps) {
        cv::Scalar color(0, 255, 0);
        auto it = palette.find(comp.label);
        if (it != palette.end()) color = it->second;

        std::vector<std::vector<cv::Point>> cList = { comp.contour };
        cv::drawContours(contoursImg, cList, -1, color, config.contour_thickness, cv::LINE_AA);
        cv::circle(contoursImg, comp.pixel_center, 2, color, -1, cv::LINE_AA);

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(comp.ref_des, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        cv::Point textOrg(static_cast<int>(comp.pixel_center.x - textSize.width * 0.5f),
                          static_cast<int>(comp.pixel_center.y - 4));
        textOrg.x = std::max(2, std::min(imgW - textSize.width - 2, textOrg.x));
        textOrg.y = std::max(textSize.height + 2, std::min(imgH - 2, textOrg.y));

        cv::putText(contoursImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(contoursImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> extraStats = {
        {"Confermati", static_cast<int>(processedComps.size())},
        {"Rifiniti CV", result.summary.successfully_segmented},
        {"Preservati", result.summary.preserved_original_box}
    };
    cv::Mat sideLegend1 = common::renderSideLegend(
        contoursImg.rows,
        result.summary.class_distribution,
        static_cast<int>(processedComps.size()),
        palette,
        "Contorni Segmentati",
        extraStats,
        340);
    cv::Mat combinedContoursVis;
    cv::hconcat(contoursImg, sideLegend1, combinedContoursVis);
    result.contours_vis = combinedContoursVis;

    // 2. Immagine Rettangoli Minimi Aderenti Senza Spazi
    cv::Mat boxesImg = pcbImage.clone();
    for (const auto& comp : processedComps) {
        cv::Scalar color(0, 255, 0);
        auto it = palette.find(comp.label);
        if (it != palette.end()) color = it->second;

        cv::Point2f pts[4];
        comp.rotated_box.points(pts);
        for (int j = 0; j < 4; ++j) {
            cv::line(boxesImg, pts[j], pts[(j + 1) % 4], color, 1, cv::LINE_AA);
        }
        cv::circle(boxesImg, comp.pixel_center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(comp.ref_des, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        cv::Point textOrg(static_cast<int>(comp.pixel_center.x - textSize.width * 0.5f),
                          static_cast<int>(comp.pixel_center.y - 4));
        textOrg.x = std::max(2, std::min(imgW - textSize.width - 2, textOrg.x));
        textOrg.y = std::max(textSize.height + 2, std::min(imgH - 2, textOrg.y));

        cv::putText(boxesImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(boxesImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    cv::Mat sideLegend2 = common::renderSideLegend(
        boxesImg.rows,
        result.summary.class_distribution,
        static_cast<int>(processedComps.size()),
        palette,
        "Rettangoli Aderenti",
        extraStats,
        340);
    cv::Mat combinedBoxesVis;
    cv::hconcat(boxesImg, sideLegend2, combinedBoxesVis);
    result.tight_boxes_vis = combinedBoxesVis;

    result.success = true;
    return result;
}

// =========================================================================
// SALVATAGGIO DEI RISULTATI SU DISCO
// =========================================================================

bool ComponentSegmentationProcessor::saveResults(
    const std::string& outputDir,
    const std::string& fileStem,
    const ComponentSegmentationResult& result) {

    fs::create_directories(outputDir);

    std::string csvPath = (fs::path(outputDir) / (fileStem + "_precise_segmented_components.csv")).string();
    std::ofstream csv(csvPath);
    if (!csv.is_open()) {
        std::cerr << "[SEGMENTATION] Impossibile creare il file CSV: " << csvPath << std::endl;
        return false;
    }

    csv << "id,componente_codice,tipologia_classe,codice_produttore,package_outline,"
        << "box_width_px,box_height_px,box_width_mm,box_height_mm,"
        << "pixel_center_x,pixel_center_y,pixel_angle_deg,area_px,"
        << "reale_pos_x_mm,reale_pos_y_mm,reale_rot_deg,"
        << "nominale_x_mm,nominale_y_mm,nominale_rot_deg,"
        << "errore_pos_x_mm,errore_pos_y_mm,errore_dist_mm,errore_dist_px,errore_rot_deg,"
        << "search_expansion_level,notes\n";

    for (const auto& c : result.segmented_components) {
        csv << c.id << ","
            << "\"" << c.ref_des << "\","
            << "\"" << c.label << "\","
            << "\"" << c.part_number << "\","
            << "\"" << c.outline << "\","
            << std::fixed << std::setprecision(2)
            << c.box_width_px << ","
            << c.box_height_px << ","
            << std::setprecision(3)
            << c.box_width_mm << ","
            << c.box_height_mm << ","
            << std::setprecision(2)
            << c.pixel_center.x << ","
            << c.pixel_center.y << ","
            << c.pixel_angle_deg << ","
            << c.area_px << ","
            << std::setprecision(4)
            << c.real_x_mm << ","
            << c.real_y_mm << ","
            << std::setprecision(2)
            << c.real_rot_deg << ","
            << std::setprecision(4)
            << c.nominal_x_mm << ","
            << c.nominal_y_mm << ","
            << std::setprecision(2)
            << c.nominal_rot_deg << ","
            << std::setprecision(4)
            << c.error_x_mm << ","
            << c.error_y_mm << ","
            << c.error_dist_mm << ","
            << std::setprecision(2)
            << c.error_dist_px << ","
            << c.error_rot_deg << ","
            << c.search_expansion_level << ","
            << "\"" << c.notes << "\"\n";
    }
    csv.close();
    std::cout << "[SEGMENTATION] Salvato CSV componenti segmentati: " << csvPath << std::endl;

    std::string jsonPath = (fs::path(outputDir) / (fileStem + "_precise_segmented_summary.json")).string();
    std::ofstream json(jsonPath);
    if (json.is_open()) {
        json << "{\n"
             << "  \"board_name\": \"" << result.board_name << "\",\n"
             << "  \"side\": \"" << result.side << "\",\n"
             << "  \"total_input_components\": " << result.summary.total_input_components << ",\n"
             << "  \"successfully_segmented\": " << result.summary.successfully_segmented << ",\n"
             << "  \"found_in_initial_roi\": " << result.summary.found_in_initial_roi << ",\n"
             << "  \"found_in_expanded_roi\": " << result.summary.found_in_expanded_roi << ",\n"
             << "  \"preserved_original_box\": " << result.summary.preserved_original_box << ",\n"
             << "  \"mean_error_dist_mm\": " << std::fixed << std::setprecision(4) << result.summary.mean_error_dist_mm << ",\n"
             << "  \"mean_error_dist_px\": " << std::fixed << std::setprecision(2) << result.summary.mean_error_dist_px << ",\n"
             << "  \"mean_error_rot_deg\": " << std::fixed << std::setprecision(2) << result.summary.mean_error_rot_deg << ",\n"
             << "  \"canonical_part_number_sizes_mm\": {\n";

        size_t partIdx = 0;
        for (const auto& p : result.summary.canonical_part_sizes_mm) {
            json << "    \"" << p.first << "\": {\"width_mm\": "
                 << std::fixed << std::setprecision(3) << p.second.width
                 << ", \"height_mm\": " << p.second.height << "}";
            if (++partIdx < result.summary.canonical_part_sizes_mm.size()) json << ",";
            json << "\n";
        }
        json << "  },\n";

        json << "  \"class_distribution\": {\n";
        size_t cIdx = 0;
        for (const auto& c : result.summary.class_distribution) {
            json << "    \"" << c.first << "\": " << c.second;
            if (++cIdx < result.summary.class_distribution.size()) json << ",";
            json << "\n";
        }
        json << "  }\n";
        json << "}\n";
        json.close();
        std::cout << "[SEGMENTATION] Salvato JSON riepilogo: " << jsonPath << std::endl;
    }

    if (!result.contours_vis.empty()) {
        std::string contoursPath = (fs::path(outputDir) / (fileStem + "_segmented_contours.jpg")).string();
        cv::imwrite(contoursPath, result.contours_vis);
        std::cout << "[SEGMENTATION] Salvata immagine contorni sottili e colorati: " << contoursPath << std::endl;
    }

    if (!result.tight_boxes_vis.empty()) {
        std::string boxesPath = (fs::path(outputDir) / (fileStem + "_precise_bounding_boxes.jpg")).string();
        cv::imwrite(boxesPath, result.tight_boxes_vis);
        std::cout << "[SEGMENTATION] Salvata immagine rettangoli aderenti: " << boxesPath << std::endl;
    }

    return true;
}

// =========================================================================
// ELABORAZIONE AD ALTO LIVELLO CON FUSIONE MULTI-ESPOSIZIONE
// =========================================================================

ComponentSegmentationResult ComponentSegmentationProcessor::processBoardSide(
    const std::string& boardName,
    const std::string& side,
    const std::string& undistortedDir,
    const std::string& aoiVerificationDir,
    const std::string& paramsDir,
    const std::string& outputBaseDir,
    const ComponentSegmentationConfig& config) {

    ComponentSegmentationResult res;
    res.board_name = boardName;
    res.side = side;

    fs::path aoiSideDir = fs::path(aoiVerificationDir) / boardName / side;
    if (!fs::exists(aoiSideDir)) {
        std::cerr << "[SEGMENTATION] Cartella AOI verification non trovata: " << aoiSideDir << std::endl;
        return res;
    }

    // 1. Cerca il file CSV verificato prodotto da AOI verification
    std::string verifiedCsvPath;
    for (const auto& entry : fs::directory_iterator(aoiSideDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string name = entry.path().filename().string();
            if (name.find("_aoi_verified_components.csv") != std::string::npos) {
                verifiedCsvPath = entry.path().string();
                break;
            }
        }
    }

    if (verifiedCsvPath.empty()) {
        std::cerr << "[SEGMENTATION] File CSV AOI verified non trovato in: " << aoiSideDir << std::endl;
        return res;
    }

    std::string fileStem = fs::path(verifiedCsvPath).stem().string();
    size_t pos = fileStem.find("_aoi_verified_components");
    if (pos != std::string::npos) {
        fileStem = fileStem.substr(0, pos);
    }
    res.reference_image_name = fileStem;

    // 2. Carica l'immagine PCB di riferimento non distorta
    fs::path imgPath = fs::path(undistortedDir) / boardName / side / (fileStem + ".bmp");
    if (!fs::exists(imgPath)) {
        imgPath = fs::path(undistortedDir) / boardName / side / (fileStem + ".jpg");
    }
    if (!fs::exists(imgPath)) {
        fs::path sideDir = fs::path(undistortedDir) / boardName / side;
        if (fs::exists(sideDir)) {
            for (const auto& entry : fs::directory_iterator(sideDir)) {
                if (entry.is_regular_file() && (entry.path().extension() == ".bmp" || entry.path().extension() == ".jpg")) {
                    imgPath = entry.path();
                    break;
                }
            }
        }
    }

    cv::Mat pcbImage = cv::imread(imgPath.string());
    if (pcbImage.empty()) {
        std::cerr << "[SEGMENTATION] Impossibile caricare immagine PCB: " << imgPath << std::endl;
        return res;
    }

    // =========================================================================
    // REQUISITO: "il cv_component_detection con tutti i suoi filtri e con tutte
    // le immagini ad esposizioni diverse funzionava molto meglio!"
    // Esecuzione multi-esposizione con fusione delle maschere CV
    // =========================================================================
    fs::path sideDir = fs::path(undistortedDir) / boardName / side;
    std::vector<fs::path> exposureFiles;
    if (fs::exists(sideDir)) {
        for (const auto& entry : fs::directory_iterator(sideDir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".bmp" || ext == ".jpg") {
                    exposureFiles.push_back(entry.path());
                }
            }
        }
    }
    std::sort(exposureFiles.begin(), exposureFiles.end());

    cv::Mat fusedComponentsMask = cv::Mat::zeros(pcbImage.size(), CV_8UC1);
    CvComponentDetector cvDetector;
    CvDetectionConfig cvCfg;
    cvCfg.show_windows = false;

    if (!exposureFiles.empty()) {
        std::cout << "[SEGMENTATION] Esecuzione CvComponentDetector su " << exposureFiles.size()
                  << " immagini a esposizione diversa..." << std::endl;
        for (const auto& expPath : exposureFiles) {
            cv::Mat expImg = cv::imread(expPath.string());
            if (expImg.empty() || expImg.size() != pcbImage.size()) continue;

            CvDetectionResult cvRes = cvDetector.detect(expImg, cvCfg);
            cv::bitwise_or(fusedComponentsMask, cvRes.components_mask, fusedComponentsMask);
        }
        std::cout << "[SEGMENTATION] Fusione multi-esposizione completata con successo." << std::endl;
    } else {
        CvDetectionResult cvRes = cvDetector.detect(pcbImage, cvCfg);
        fusedComponentsMask = cvRes.components_mask.clone();
    }

    // 3. Carica i parametri di trasformazione e modello di distorsione YAML
    fs::path yamlPath = fs::path(paramsDir) / ("aoi_alignment_" + boardName + "_" + side + ".yml");
    AoiAlignmentTransform transform;
    if (fs::exists(yamlPath)) {
        cv::FileStorage fs(yamlPath.string(), cv::FileStorage::READ);
        if (fs.isOpened()) {
            transform.valid = true;
            fs["orientation_name"] >> transform.orientation_name;
            int hasDist = 0;
            fs["has_distortion_model"] >> hasDist;
            transform.has_distortion_model = (hasDist != 0);
            fs["homography_3x3"] >> transform.homography_3x3;
            if (!transform.homography_3x3.empty()) {
                cv::invert(transform.homography_3x3, transform.homography_inv_3x3);
            }
            fs["tx"] >> transform.tx;
            fs["ty"] >> transform.ty;
            fs["scale_px_per_mm"] >> transform.scale_px_per_mm;
            fs["inlier_count"] >> transform.inlier_count;
            fs["inlier_ratio"] >> transform.inlier_ratio;
            fs["avg_error_px"] >> transform.avg_error_px;
            fs["avg_error_mm"] >> transform.avg_error_mm;
            fs.release();
            std::cout << "[SEGMENTATION] Caricati parametri allineamento e omografia da: " << yamlPath << std::endl;
        }
    }

    // 4. Parser del CSV dei componenti verificati AOI
    std::vector<VerifiedPcbComponent> inputComps;
    std::ifstream csvFile(verifiedCsvPath);
    if (!csvFile.is_open()) {
        std::cerr << "[SEGMENTATION] Impossibile aprire CSV: " << verifiedCsvPath << std::endl;
        return res;
    }

    std::string headerLine;
    std::getline(csvFile, headerLine);

    std::string line;
    while (std::getline(csvFile, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> cols;

        bool inQuotes = false;
        std::string cur;
        for (char ch : line) {
            if (ch == '\"') inQuotes = !inQuotes;
            else if (ch == ',' && !inQuotes) {
                cols.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        cols.push_back(cur);

        if (cols.size() < 24) continue;

        VerifiedPcbComponent vc;
        vc.id = std::stoi(cols[0]);
        vc.ref_des = cols[1];
        std::string statusStr = cols[2];
        if (statusStr == "MATCH_EXACT") vc.status = AoiMatchStatus::MATCH_EXACT_CLASS;
        else if (statusStr == "CLASS_CORRECTED") vc.status = AoiMatchStatus::CLASS_CORRECTED;
        else if (statusStr == "RECOVERED_BY_AOI") vc.status = AoiMatchStatus::RECOVERED_BY_AOI;
        else vc.status = AoiMatchStatus::UNMATCHED_DETECTION;

        vc.label = cols[3];
        vc.original_label = cols[4];
        vc.confidence = std::stof(cols[5]);
        vc.source_name = cols[6];
        vc.center.x = std::stof(cols[7]);
        vc.center.y = std::stof(cols[8]);
        float w = std::stof(cols[9]);
        float h = std::stof(cols[10]);
        float ang = std::stof(cols[11]);
        vc.box = cv::Rect2f(vc.center.x - w * 0.5f, vc.center.y - h * 0.5f, w, h);
        vc.rotated_box = cv::RotatedRect(vc.center, cv::Size2f(w, h), ang);
        vc.radius = std::stof(cols[12]);
        vc.area = std::stof(cols[13]);
        vc.nominal_width_mm = std::stod(cols[16]);
        vc.nominal_height_mm = std::stod(cols[17]);
        vc.nominal_center_px.x = std::stof(cols[18]);
        vc.nominal_center_px.y = std::stof(cols[19]);
        vc.offset_error_px = std::stod(cols[20]);
        vc.offset_error_mm = std::stod(cols[21]);
        vc.aoi_part_number = cols[22];
        vc.aoi_outline = cols[23];
        if (cols.size() > 24) vc.notes = cols[24];

        inputComps.push_back(vc);
    }
    csvFile.close();

    std::cout << "[SEGMENTATION] Caricati " << inputComps.size() << " componenti verificati da: " << verifiedCsvPath << std::endl;

    // 5. Esegui la segmentazione e rifinitura geometrica usando la maschera multi-esposizione
    res = process(pcbImage, inputComps, transform, config, fusedComponentsMask);
    res.board_name = boardName;
    res.side = side;
    res.reference_image_name = fileStem;

    // 6. Salva tutti i risultati su disco
    fs::path outSideDir = fs::path(outputBaseDir) / boardName / side;
    saveResults(outSideDir.string(), fileStem, res);

    if (config.show_windows && !res.contours_vis.empty()) {
        std::string winName = "Contorni Segmentati: " + boardName + " [" + side + "]";
        cv::namedWindow(winName, cv::WINDOW_NORMAL);
        cv::resizeWindow(winName, 1400, 950);
        cv::imshow(winName, res.contours_vis);
        std::cout << "\nPremi un tasto sulla finestra OpenCV dei contorni per continuare..." << std::endl;
        cv::waitKey(0);
        cv::destroyWindow(winName);
    }

    return res;
}

} // namespace pcb
