#include "output_internal.hpp"
#include <algorithm>

namespace pcb {

// =========================================================================
// STEP 3: Fusione finale completa (DL + CV) & Risoluzione Sovrapposizioni
// =========================================================================
std::vector<FinalComponent> OutputProcessor::resolveEnclosingAndOverlaps(
    const std::vector<FinalComponent>& comps,
    float enclosure_thresh) {

    if (comps.size() <= 1) return comps;

    size_t n = comps.size();
    std::vector<bool> keep(n, true);

    // Controlliamo tutte le coppie di componenti sovrapposti
    for (size_t i = 0; i < n; ++i) {
        if (!keep[i]) continue;

        for (size_t j = i + 1; j < n; ++j) {
            if (!keep[j]) continue;

            const auto& a = comps[i];
            const auto& b = comps[j];

            // Intersezione tra bounding box
            float x1 = std::max(a.box.x, b.box.x);
            float y1 = std::max(a.box.y, b.box.y);
            float x2 = std::min(a.box.x + a.box.width, b.box.x + b.box.width);
            float y2 = std::min(a.box.y + a.box.height, b.box.y + b.box.height);

            float inter_w = std::max(0.0f, x2 - x1);
            float inter_h = std::max(0.0f, y2 - y1);
            float inter_area = inter_w * inter_h;
            if (inter_area <= 0.0f) continue;

            float area_a = std::max(1.0f, a.box.width * a.box.height);
            float area_b = std::max(1.0f, b.box.width * b.box.height);

            float overlap_a = inter_area / area_a;
            float overlap_b = inter_area / area_b;

            // Determina componente più piccolo vs più grande
            size_t idx_smaller = (area_a <= area_b) ? i : j;
            size_t idx_larger  = (area_a <= area_b) ? j : i;
            float overlap_smaller = (area_a <= area_b) ? overlap_a : overlap_b;
            float area_ratio = (area_a <= area_b) ? (area_b / area_a) : (area_a / area_b);

            bool center_inside = comps[idx_larger].box.contains(comps[idx_smaller].center);

            // Requisito: escludi componenti grandi che inglobano componenti piccoli (DL ha priorità)
            bool is_completely_enclosed = (overlap_smaller >= enclosure_thresh) ||
                                          (center_inside && overlap_smaller >= 0.70f && area_ratio >= 1.5f);

            if (is_completely_enclosed) {
                bool larger_is_dl  = (comps[idx_larger].source != ComponentSource::CV_ONLY);
                bool smaller_is_dl = (comps[idx_smaller].source != ComponentSource::CV_ONLY);

                if (larger_is_dl && !smaller_is_dl) {
                    keep[idx_smaller] = false;
                } else if (!larger_is_dl && smaller_is_dl) {
                    keep[idx_larger] = false;
                } else {
                    keep[idx_larger] = false;
                }
            }
        }
    }

    std::vector<FinalComponent> resolved;
    resolved.reserve(comps.size());
    int progressive_id = 1;
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            FinalComponent fc = comps[i];
            fc.id = progressive_id++;
            resolved.push_back(fc);
        }
    }
    return resolved;
}

std::vector<FinalComponent> OutputProcessor::fuseDlAndCv(
    const std::vector<FusedDlComponent>& merged_dl,
    const std::vector<ClassifiedCvComponent>& classified_cv,
    float match_iou_threshold,
    float tiny_cv_max_area,
    float tiny_cv_max_dim) {

    std::vector<FinalComponent> final_list;
    std::vector<bool> cv_used(classified_cv.size(), false);

    // 1) Elabora ciascun componente DL fuso
    for (const auto& dl : merged_dl) {
        float best_iou = 0.0f;
        int best_cv_idx = -1;

        for (size_t j = 0; j < classified_cv.size(); ++j) {
            if (cv_used[j]) continue;
            cv::Rect2f cv_box;
            if (classified_cv[j].cv_geom.shape == ShapeType::CIRCLE) {
                float r = classified_cv[j].cv_geom.radius;
                cv_box = cv::Rect2f(classified_cv[j].cv_geom.center.x - r,
                                    classified_cv[j].cv_geom.center.y - r,
                                    std::max(1.0f, 2.0f * r),
                                    std::max(1.0f, 2.0f * r));
            } else {
                cv_box = classified_cv[j].cv_geom.rotated_box.boundingRect2f();
            }

            float iou = calculateIoU(dl.box, cv_box);
            float ov = calculateOverlap(dl.box, cv_box);
            cv::Point2f dl_center(dl.box.x + dl.box.width * 0.5f, dl.box.y + dl.box.height * 0.5f);
            bool contains = dl.box.contains(classified_cv[j].cv_geom.center) || cv_box.contains(dl_center);

            float score = iou;
            if (ov >= 0.35f) score = std::max(score, ov * 0.85f);
            if (contains) score = std::max(score, 0.40f);

            if (score > best_iou) {
                best_iou = score;
                best_cv_idx = static_cast<int>(j);
            }
        }

        FinalComponent fc;
        fc.shape = ShapeType::RECTANGLE;
        fc.box = dl.box;
        fc.center = cv::Point2f(dl.box.x + dl.box.width * 0.5f, dl.box.y + dl.box.height * 0.5f);
        fc.rotated_box = cv::RotatedRect(fc.center, dl.box.size(), 0.0f);
        fc.radius = 0.0f;
        fc.area = dl.box.width * dl.box.height;
        fc.label = dl.label;
        fc.confidence = (dl.label != "?") ? dl.label_conf : dl.box_conf;

        if (best_iou >= match_iou_threshold && best_cv_idx >= 0) {
            fc.source = ComponentSource::FUSED_DL_CV;
            const auto& cv_ref = classified_cv[best_cv_idx];
            if (fc.label == "?" && cv_ref.label != "?") {
                fc.label = cv_ref.label;
                fc.confidence = cv_ref.label_conf;
            }
            fc.notes = "Riconosciuto da DL (" + std::to_string(dl.vote_count) + " voti) e confermato da CV";
        } else {
            fc.source = ComponentSource::DL_ONLY;
            fc.notes = "Solo DL (" + std::to_string(dl.vote_count) + " voti su " +
                       std::to_string(dl.total_observations) + " foto)";
        }

        // Rimuoviamo qualsiasi componente CV che si trova sotto a questo componente DL
        for (size_t j = 0; j < classified_cv.size(); ++j) {
            if (cv_used[j]) continue;
            const auto& cv_item = classified_cv[j];
            cv::Rect2f cv_box;
            if (cv_item.cv_geom.shape == ShapeType::CIRCLE) {
                float r = cv_item.cv_geom.radius;
                cv_box = cv::Rect2f(cv_item.cv_geom.center.x - r, cv_item.cv_geom.center.y - r, 2.0f * r, 2.0f * r);
            } else {
                cv_box = cv_item.cv_geom.rotated_box.boundingRect2f();
            }

            float x1 = std::max(dl.box.x, cv_box.x);
            float y1 = std::max(dl.box.y, cv_box.y);
            float x2 = std::min(dl.box.x + dl.box.width, cv_box.x + cv_box.width);
            float y2 = std::min(dl.box.y + dl.box.height, cv_box.y + cv_box.height);

            float inter_w = std::max(0.0f, x2 - x1);
            float inter_h = std::max(0.0f, y2 - y1);
            float inter_area = inter_w * inter_h;

            if (inter_area > 0.0f) {
                float cv_area = std::max(1.0f, cv_box.width * cv_box.height);
                float dl_area = std::max(1.0f, dl.box.width * dl.box.height);
                float cv_overlap = inter_area / cv_area;
                float dl_overlap = inter_area / dl_area;
                float iou = inter_area / (cv_area + dl_area - inter_area);

                bool cv_center_in_dl = dl.box.contains(cv_item.cv_geom.center);
                bool dl_center_in_cv = cv_box.contains(fc.center);

                if (cv_center_in_dl || dl_center_in_cv || cv_overlap >= 0.20f || dl_overlap >= 0.20f || iou >= 0.15f) {
                    cv_used[j] = true;
                }
            }
        }

        final_list.push_back(fc);
    }

    // 2) Aggiungi tutti i componenti CV non ancora abbinati (CV_ONLY)
    for (size_t j = 0; j < classified_cv.size(); ++j) {
        if (!cv_used[j]) {
            const auto& cv_ref = classified_cv[j];

            if (cv_ref.label == "?" || cv_ref.label.empty()) {
                continue;
            }

            if (isTinyUnclassified(cv_ref.cv_geom, cv_ref.label, tiny_cv_max_area, tiny_cv_max_dim)) {
                continue;
            }

            FinalComponent fc;
            fc.source = ComponentSource::CV_ONLY;
            fc.shape = cv_ref.cv_geom.shape;
            fc.center = cv_ref.cv_geom.center;
            fc.radius = cv_ref.cv_geom.radius;
            fc.rotated_box = cv_ref.cv_geom.rotated_box;
            fc.area = cv_ref.cv_geom.area;

            if (fc.shape == ShapeType::CIRCLE) {
                float r = fc.radius;
                fc.box = cv::Rect2f(fc.center.x - r, fc.center.y - r, 2.0f * r, 2.0f * r);
            } else {
                fc.box = fc.rotated_box.boundingRect2f();
            }

            fc.label = cv_ref.label;
            fc.confidence = cv_ref.label_conf;
            fc.notes = std::string("Solo CV (") + (fc.shape == ShapeType::CIRCLE ? "Cerchio" : "Rettangolo ruotato") +
                       ", Metodo: " + cv_ref.classification_method + ")";

            final_list.push_back(fc);
        }
    }

    std::vector<FinalComponent> non_overlapping_list = resolveEnclosingAndOverlaps(final_list);

    // Ordina da cima a fondo, poi da sinistra a destra
    std::sort(non_overlapping_list.begin(), non_overlapping_list.end(), [](const auto& a, const auto& b) {
        if (std::abs(a.center.y - b.center.y) > 20.0f) {
            return a.center.y < b.center.y;
        }
        return a.center.x < b.center.x;
    });

    for (size_t i = 0; i < non_overlapping_list.size(); ++i) {
        non_overlapping_list[i].id = static_cast<int>(i + 1);
    }

    return non_overlapping_list;
}

} // namespace pcb

