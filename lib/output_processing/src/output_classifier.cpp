#include "output_internal.hpp"
#include <algorithm>

namespace pcb {

// =========================================================================
// STEP 2: Riconoscimento componenti CV tramite pesi DL
// =========================================================================
std::vector<ClassifiedCvComponent> OutputProcessor::classifyCvComponentsWithDl(
    const std::vector<CvComponent>& cv_components,
    const std::vector<FusedDlComponent>& merged_dl,
    const cv::Mat& best_image,
    ComponentDetector* dl_detector,
    float match_iou_threshold,
    float min_conf,
    bool enable_patch_inference) {

    std::vector<ClassifiedCvComponent> classified;
    classified.reserve(cv_components.size());

    int prog_id = 1;
    for (const auto& cv_c : cv_components) {
        ClassifiedCvComponent item;
        item.id = prog_id++;
        item.cv_geom = cv_c;

        cv::Rect2f cv_box;
        if (cv_c.shape == ShapeType::CIRCLE) {
            cv_box = cv::Rect2f(cv_c.center.x - cv_c.radius,
                                cv_c.center.y - cv_c.radius,
                                std::max(1.0f, 2.0f * cv_c.radius),
                                std::max(1.0f, 2.0f * cv_c.radius));
        } else {
            cv_box = cv_c.rotated_box.boundingRect2f();
        }

        // 1) Matching spaziale con i rilevamenti DL multi-esposizione
        float best_iou = 0.0f;
        int best_dl_idx = -1;

        for (size_t i = 0; i < merged_dl.size(); ++i) {
            float iou = calculateIoU(cv_box, merged_dl[i].box);
            float ov = calculateOverlap(cv_box, merged_dl[i].box);
            cv::Point2f dl_center(merged_dl[i].box.x + merged_dl[i].box.width * 0.5f,
                                  merged_dl[i].box.y + merged_dl[i].box.height * 0.5f);
            bool contains = merged_dl[i].box.contains(cv_c.center) || cv_box.contains(dl_center);

            float score = iou;
            if (ov >= 0.35f) score = std::max(score, ov * 0.85f);
            if (contains) score = std::max(score, 0.40f);

            if (score > best_iou) {
                best_iou = score;
                best_dl_idx = static_cast<int>(i);
            }
        }

        if (best_iou >= match_iou_threshold && best_dl_idx >= 0) {
            item.matched_with_dl = true;
            item.label = merged_dl[best_dl_idx].label;
            item.label_conf = merged_dl[best_dl_idx].label_conf;
            item.class_id = merged_dl[best_dl_idx].class_id;
            item.classification_method = "DL_SPATIAL_MATCH";
        } else {
            // 2) Non abbinato a livello globale: tenta l'inferenza fine locale su patch (se abilitata e per componenti significativi)
            bool patch_classified = false;

            if (enable_patch_inference && dl_detector != nullptr && !best_image.empty() && cv_c.area >= 20.0f) {
                int pad = std::max(24, static_cast<int>(std::max(cv_box.width, cv_box.height) * 0.5f));
                int x0 = std::max(0, static_cast<int>(std::floor(cv_box.x)) - pad);
                int y0 = std::max(0, static_cast<int>(std::floor(cv_box.y)) - pad);
                int x1 = std::min(best_image.cols, static_cast<int>(std::ceil(cv_box.x + cv_box.width)) + pad);
                int y1 = std::min(best_image.rows, static_cast<int>(std::ceil(cv_box.y + cv_box.height)) + pad);

                int cw = x1 - x0;
                int ch = y1 - y0;

                if (cw > 10 && ch > 10) {
                    cv::Rect crop_roi(x0, y0, cw, ch);
                    cv::Mat crop = best_image(crop_roi);

                    DetectionConfig patch_cfg;
                    patch_cfg.conf_threshold = min_conf;
                    patch_cfg.nms_threshold = 0.45f;
                    patch_cfg.imgsz = 1024;
                    patch_cfg.tile_size = 0;
                    patch_cfg.label_display_mode = "none";

                    auto patch_res = dl_detector->detect(crop, patch_cfg, "fine");
                    float best_patch_iou = 0.0f;
                    std::string best_patch_label = "?";
                    float best_patch_conf = 0.0f;
                    int best_patch_cls_id = -1;

                    cv::Point2f local_cv_center(cv_c.center.x - x0, cv_c.center.y - y0);

                    for (const auto& pd : patch_res.detections) {
                        cv::Rect2f local_cv_box(cv_box.x - x0, cv_box.y - y0, cv_box.width, cv_box.height);
                        float iou = calculateIoU(local_cv_box, pd.box);
                        bool contains_center = pd.box.contains(local_cv_center);

                        if (iou > best_patch_iou || contains_center) {
                            if (pd.label != "?" && pd.label_conf >= min_conf) {
                                best_patch_iou = iou;
                                best_patch_label = pd.label;
                                best_patch_conf = pd.label_conf;
                                best_patch_cls_id = pd.class_id;
                            }
                        }
                    }

                    if (best_patch_label != "?") {
                        item.label = best_patch_label;
                        item.label_conf = best_patch_conf;
                        item.class_id = best_patch_cls_id;
                        item.classification_method = "DL_PATCH_INFERENCE";
                        patch_classified = true;
                    }
                }
            }

            if (!patch_classified) {
                item.label = "?";
                item.label_conf = 0.0f;
                item.class_id = -1;
                item.classification_method = "UNCLASSIFIED";
            }
        }

        classified.push_back(item);
    }

    return classified;
}

bool OutputProcessor::isTinyUnclassified(
    const CvComponent& cv_c,
    const std::string& label,
    float max_area,
    float max_dim) {

    // Se l'elemento è stato catalogato dal deep learning (non '?'), lo consideriamo sempre!
    if (label != "?") {
        return false;
    }

    float w = (cv_c.shape == ShapeType::CIRCLE) ? (cv_c.radius * 2.0f) : cv_c.rotated_box.size.width;
    float h = (cv_c.shape == ShapeType::CIRCLE) ? (cv_c.radius * 2.0f) : cv_c.rotated_box.size.height;
    float min_s = std::min(w, h);
    float max_s = std::max(w, h);

    // Se l'area è minuscola o la dimensione massima è microscopica (o spessore 1D), ed è '?', scartalo
    if (cv_c.area < max_area || max_s < max_dim || min_s < 3.5f) {
        return true;
    }
    return false;
}

} // namespace pcb

