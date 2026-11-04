#include "output_internal.hpp"
#include <algorithm>
#include <map>

namespace pcb {

ComponentVisualFeature extractVisualFeature(const cv::Mat& image, const FinalComponent& comp) {
    ComponentVisualFeature feat;
    feat.shape = comp.shape;
    if (image.empty()) return feat;

    const int target_w = 48;
    const int target_h = 48;
    cv::Mat patch_bgr;

    if (comp.shape == ShapeType::CIRCLE) {
        float r = std::max(2.0f, comp.radius);
        feat.width = r * 2.0f;
        feat.height = r * 2.0f;
        feat.area = (comp.area > 0.0f) ? comp.area : (static_cast<float>(CV_PI) * r * r);

        int x0 = std::max(0, static_cast<int>(std::floor(comp.center.x - r)));
        int y0 = std::max(0, static_cast<int>(std::floor(comp.center.y - r)));
        int x1 = std::min(image.cols, static_cast<int>(std::ceil(comp.center.x + r)));
        int y1 = std::min(image.rows, static_cast<int>(std::ceil(comp.center.y + r)));

        if (x1 <= x0 || y1 <= y0) return feat;
        cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
        cv::resize(image(roi), patch_bgr, cv::Size(target_w, target_h), 0, 0, cv::INTER_AREA);
    } else {
        // RECTANGLE
        float w = comp.rotated_box.size.width;
        float h = comp.rotated_box.size.height;
        if (w <= 0.0f || h <= 0.0f) {
            w = comp.box.width;
            h = comp.box.height;
        }
        feat.width = std::max(w, h);
        feat.height = std::min(w, h);
        feat.area = (comp.area > 0.0f) ? comp.area : (feat.width * feat.height);

        if (comp.rotated_box.size.width > 2.0f && comp.rotated_box.size.height > 2.0f) {
            cv::Point2f pts[4];
            comp.rotated_box.points(pts);
            float d01 = static_cast<float>(cv::norm(pts[0] - pts[1]));
            float d12 = static_cast<float>(cv::norm(pts[1] - pts[2]));

            cv::Point2f src[4];
            if (d01 >= d12) {
                src[0] = pts[0]; src[1] = pts[1]; src[2] = pts[2]; src[3] = pts[3];
            } else {
                src[0] = pts[1]; src[1] = pts[2]; src[2] = pts[3]; src[3] = pts[0];
            }
            cv::Point2f dst[4] = {
                cv::Point2f(0.0f, 0.0f),
                cv::Point2f(static_cast<float>(target_w), 0.0f),
                cv::Point2f(static_cast<float>(target_w), static_cast<float>(target_h)),
                cv::Point2f(0.0f, static_cast<float>(target_h))
            };
            cv::Mat M = cv::getPerspectiveTransform(src, dst);
            cv::warpPerspective(image, patch_bgr, M, cv::Size(target_w, target_h), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        } else {
            int x0 = std::max(0, static_cast<int>(std::floor(comp.box.x)));
            int y0 = std::max(0, static_cast<int>(std::floor(comp.box.y)));
            int x1 = std::min(image.cols, static_cast<int>(std::ceil(comp.box.x + comp.box.width)));
            int y1 = std::min(image.rows, static_cast<int>(std::ceil(comp.box.y + comp.box.height)));
            if (x1 <= x0 || y1 <= y0) return feat;
            cv::Mat crop = image(cv::Rect(x0, y0, x1 - x0, y1 - y0));
            if (crop.cols < crop.rows) {
                cv::rotate(crop, crop, cv::ROTATE_90_CLOCKWISE);
            }
            cv::resize(crop, patch_bgr, cv::Size(target_w, target_h), 0, 0, cv::INTER_AREA);
        }
    }

    if (patch_bgr.empty() || patch_bgr.cols != target_w || patch_bgr.rows != target_h) {
        return feat;
    }

    // 1) Grayscale patch normalizzato con leggero smoothing
    cv::cvtColor(patch_bgr, feat.patch_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(feat.patch_gray, feat.patch_gray, cv::Size(3, 3), 0.5);

    // 2) Istogramma HSV 2D (Hue e Saturation)
    cv::Mat hsv;
    cv::cvtColor(patch_bgr, hsv, cv::COLOR_BGR2HSV);
    int h_bins = 16, s_bins = 16;
    int histSize[] = {h_bins, s_bins};
    float h_ranges[] = {0.0f, 180.0f};
    float s_ranges[] = {0.0f, 256.0f};
    const float* ranges[] = {h_ranges, s_ranges};
    int channels[] = {0, 1};
    cv::calcHist(&hsv, 1, channels, cv::Mat(), feat.hist_hsv, 2, histSize, ranges, true, false);
    cv::normalize(feat.hist_hsv, feat.hist_hsv, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    feat.valid = true;
    return feat;
}

bool areFeaturesIdentical(const ComponentVisualFeature& f1,
                          const ComponentVisualFeature& f2,
                          float geom_tol,
                          float ncc_thresh,
                          float hist_thresh) {
    if (!f1.valid || !f2.valid) return false;
    if (f1.shape != f2.shape) return false;

    // 1. Controllo geometrico
    float max_w = std::max(f1.width, f2.width);
    float max_h = std::max(f1.height, f2.height);
    if (max_w <= 0.0f || max_h <= 0.0f) return false;

    float diff_w = std::abs(f1.width - f2.width) / max_w;
    float diff_h = std::abs(f1.height - f2.height) / max_h;
    if (diff_w > geom_tol || diff_h > geom_tol) {
        return false;
    }

    // Aspect ratio check
    float ar1 = f1.width / std::max(1.0f, f1.height);
    float ar2 = f2.width / std::max(1.0f, f2.height);
    float max_ar = std::max(ar1, ar2);
    if (std::abs(ar1 - ar2) / max_ar > geom_tol * 1.30f) {
        return false;
    }

    // 2. Controllo istogramma colore HSV
    double bhatt = cv::compareHist(f1.hist_hsv, f2.hist_hsv, cv::HISTCMP_BHATTACHARYYA);
    if (bhatt > hist_thresh) {
        return false;
    }

    // 3. Normalized Cross Correlation (NCC) standard (0°) e ruotato di 180°
    cv::Mat match0, match180;
    cv::matchTemplate(f1.patch_gray, f2.patch_gray, match0, cv::TM_CCOEFF_NORMED);
    float ncc0 = match0.at<float>(0, 0);

    cv::Mat f2_flipped;
    cv::flip(f2.patch_gray, f2_flipped, -1);
    cv::matchTemplate(f1.patch_gray, f2_flipped, match180, cv::TM_CCOEFF_NORMED);
    float ncc180 = match180.at<float>(0, 0);

    float best_ncc = std::max(ncc0, ncc180);
    if (best_ncc < ncc_thresh) {
        return false;
    }

    return true;
}

// =========================================================================
// Armonizzazione Componenti Identici via Computer Vision
// =========================================================================
std::vector<FinalComponent> OutputProcessor::harmonizeIdenticalComponents(
    const std::vector<FinalComponent>& comps,
    const cv::Mat& image,
    float geom_tol,
    float ncc_thresh,
    float hist_thresh,
    std::vector<SimilarityClusterInfo>* clusters_out) {

    if (comps.empty() || image.empty()) {
        return comps;
    }

    size_t n = comps.size();
    std::vector<FinalComponent> harmonized = comps;

    // 1) Pre-estrazione delle feature visuali per tutti i componenti
    std::vector<ComponentVisualFeature> features(n);
    for (size_t i = 0; i < n; ++i) {
        features[i] = extractVisualFeature(image, comps[i]);
    }

    // 2) Disjoint Set Union (DSU) per partizionamento in cluster di equivalenza
    std::vector<int> parent(n);
    for (size_t i = 0; i < n; ++i) parent[i] = static_cast<int>(i);

    auto find_root = [&](auto& self, int i) -> int {
        return (parent[i] == i) ? i : (parent[i] = self(self, parent[i]));
    };

    auto unite = [&](int i, int j) {
        int ri = find_root(find_root, i);
        int rj = find_root(find_root, j);
        if (ri != rj) parent[ri] = rj;
    };

    // 3) Confronto pairwise ottimizzato
    for (size_t i = 0; i < n; ++i) {
        if (!features[i].valid) continue;
        for (size_t j = i + 1; j < n; ++j) {
            if (!features[j].valid) continue;
            if (comps[i].shape != comps[j].shape) continue;

            float max_area = std::max(features[i].area, features[j].area);
            if (max_area > 0.0f && std::abs(features[i].area - features[j].area) / max_area > 0.35f) {
                continue;
            }

            if (areFeaturesIdentical(features[i], features[j], geom_tol, ncc_thresh, hist_thresh)) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }

    // 4) Raggruppamento indici per cluster
    std::map<int, std::vector<int>> cluster_groups;
    for (size_t i = 0; i < n; ++i) {
        cluster_groups[find_root(find_root, static_cast<int>(i))].push_back(static_cast<int>(i));
    }

    if (clusters_out) {
        clusters_out->clear();
    }

    int cluster_id_prog = 1;

    // 5) Risoluzione del consenso per ciascun cluster con almeno 2 elementi identici
    for (auto& [root, members] : cluster_groups) {
        if (members.size() <= 1) continue;

        std::map<std::string, int> class_counts;
        std::map<std::string, float> class_conf_sum;
        std::map<std::string, int> orig_labels;

        for (int idx : members) {
            const std::string& lbl = comps[idx].label;
            orig_labels[lbl]++;
            if (lbl != "?") {
                class_counts[lbl]++;
                class_conf_sum[lbl] += comps[idx].confidence;
            }
        }

        if (class_counts.empty()) continue;

        std::string winning_label = "?";
        int max_count = 0;
        float max_conf_sum = 0.0f;
        for (const auto& [lbl, cnt] : class_counts) {
            float conf_sum = class_conf_sum[lbl];
            if (cnt > max_count || (cnt == max_count && conf_sum > max_conf_sum)) {
                max_count = cnt;
                max_conf_sum = conf_sum;
                winning_label = lbl;
            }
        }

        float consensus_conf = max_conf_sum / static_cast<float>(max_count);
        if (max_count >= 2) {
            consensus_conf = std::min(1.0f, std::max(consensus_conf, 0.75f));
        }

        int harmonized_in_cluster = 0;
        for (int idx : members) {
            if (harmonized[idx].label != winning_label) {
                std::string old_lbl = harmonized[idx].label;
                harmonized[idx].label = winning_label;
                harmonized[idx].confidence = consensus_conf;
                harmonized[idx].notes += " [CV-Similarity Harmonized: '" + old_lbl +
                                         "' -> '" + winning_label + "' (Cluster #" +
                                         std::to_string(cluster_id_prog) + ")]";
                harmonized_in_cluster++;
            }
        }

        if (clusters_out) {
            SimilarityClusterInfo cinfo;
            cinfo.cluster_id = cluster_id_prog;
            cinfo.winning_label = winning_label;
            cinfo.consensus_confidence = consensus_conf;
            cinfo.member_count = static_cast<int>(members.size());
            cinfo.harmonized_count = harmonized_in_cluster;
            cinfo.original_labels = orig_labels;
            for (int idx : members) {
                cinfo.member_ids.push_back(harmonized[idx].id);
            }
            clusters_out->push_back(cinfo);
        }
        cluster_id_prog++;
    }

    return harmonized;
}

// =========================================================================
// Ricerca Globale di Componenti Simili su Tutta la Scheda (Fase 2)
// =========================================================================
std::vector<FinalComponent> OutputProcessor::searchSimilarComponentsOnBoard(
    const std::vector<FinalComponent>& existing_comps,
    const cv::Mat& image,
    const std::vector<SimilarityClusterInfo>& clusters,
    float search_ncc_thresh,
    float search_hist_thresh,
    float max_existing_iou) {

    std::vector<FinalComponent> discovered;
    if (existing_comps.empty() || image.empty() || clusters.empty()) {
        return discovered;
    }

    cv::Mat board_gray;
    cv::cvtColor(image, board_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Rect2f> occupied_boxes;
    occupied_boxes.reserve(existing_comps.size() + 100);
    for (const auto& ec : existing_comps) {
        occupied_boxes.push_back(ec.box);
    }

    int next_id = static_cast<int>(existing_comps.size() + 1);

    for (const auto& cl : clusters) {
        if (cl.winning_label == "?" || cl.member_count < 1) continue;

        const FinalComponent* rep_comp = nullptr;
        for (const auto& ec : existing_comps) {
            if (ec.label == cl.winning_label &&
                std::find(cl.member_ids.begin(), cl.member_ids.end(), ec.id) != cl.member_ids.end()) {
                if (!rep_comp || ec.confidence > rep_comp->confidence) {
                    rep_comp = &ec;
                }
            }
        }
        if (!rep_comp) continue;

        int tw = static_cast<int>(std::round(rep_comp->box.width));
        int th = static_cast<int>(std::round(rep_comp->box.height));
        int tx = static_cast<int>(std::round(rep_comp->box.x));
        int ty = static_cast<int>(std::round(rep_comp->box.y));

        if (tw < 8 || th < 8 || tw > 250 || th > 250) continue;
        if (tx < 0 || ty < 0 || tx + tw > image.cols || ty + th > image.rows) continue;

        cv::Rect t_roi(tx, ty, tw, th);
        cv::Mat t_crop = image(t_roi);
        cv::Mat t_gray;
        cv::cvtColor(t_crop, t_gray, cv::COLOR_BGR2GRAY);

        cv::Mat t_hsv;
        cv::cvtColor(t_crop, t_hsv, cv::COLOR_BGR2HSV);
        int h_bins = 16, s_bins = 16;
        int histSize[] = {h_bins, s_bins};
        float h_ranges[] = {0.0f, 180.0f};
        float s_ranges[] = {0.0f, 256.0f};
        const float* ranges[] = {h_ranges, s_ranges};
        int channels[] = {0, 1};
        cv::Mat t_hist;
        cv::calcHist(&t_hsv, 1, channels, cv::Mat(), t_hist, 2, histSize, ranges, true, false);
        cv::normalize(t_hist, t_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        std::vector<std::pair<cv::Mat, float>> templates_to_match;
        templates_to_match.push_back({t_gray, 0.0f});

        float aspect_ratio = static_cast<float>(std::max(tw, th)) / static_cast<float>(std::min(tw, th));
        if (aspect_ratio >= 1.15f) {
            cv::Mat t_rot;
            cv::rotate(t_gray, t_rot, cv::ROTATE_90_CLOCKWISE);
            templates_to_match.push_back({t_rot, 90.0f});
        }

        for (const auto& [cur_t, cur_angle] : templates_to_match) {
            if (cur_t.cols >= board_gray.cols || cur_t.rows >= board_gray.rows) continue;

            cv::Mat match_res;
            cv::matchTemplate(board_gray, cur_t, match_res, cv::TM_CCOEFF_NORMED);

            cv::Mat dilated;
            cv::dilate(match_res, dilated, cv::Mat(), cv::Point(-1, -1), 2);
            cv::Mat is_peak = (match_res == dilated) & (match_res >= search_ncc_thresh);

            std::vector<cv::Point> peak_pts;
            cv::findNonZero(is_peak, peak_pts);

            for (const auto& pt : peak_pts) {
                float score = match_res.at<float>(pt.y, pt.x);
                cv::Rect2f cand_box(static_cast<float>(pt.x), static_cast<float>(pt.y),
                                    static_cast<float>(cur_t.cols), static_cast<float>(cur_t.rows));

                if (cand_box.x < 0 || cand_box.y < 0 ||
                    cand_box.x + cand_box.width > image.cols ||
                    cand_box.y + cand_box.height > image.rows) {
                    continue;
                }

                bool overlaps_existing = false;
                cv::Point2f cand_center(cand_box.x + cand_box.width * 0.5f, cand_box.y + cand_box.height * 0.5f);
                for (const auto& ob : occupied_boxes) {
                    float iou = calculateIoU(cand_box, ob);
                    float ov = calculateOverlap(cand_box, ob);
                    if (iou > max_existing_iou || ov > 0.35f || ob.contains(cand_center)) {
                        overlaps_existing = true;
                        break;
                    }
                }
                if (overlaps_existing) continue;

                cv::Rect cand_roi(static_cast<int>(cand_box.x), static_cast<int>(cand_box.y),
                                  static_cast<int>(cand_box.width), static_cast<int>(cand_box.height));
                cv::Mat cand_crop = image(cand_roi);
                cv::Mat cand_hsv;
                cv::cvtColor(cand_crop, cand_hsv, cv::COLOR_BGR2HSV);
                cv::Mat cand_hist;
                cv::calcHist(&cand_hsv, 1, channels, cv::Mat(), cand_hist, 2, histSize, ranges, true, false);
                cv::normalize(cand_hist, cand_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

                double bhatt = cv::compareHist(t_hist, cand_hist, cv::HISTCMP_BHATTACHARYYA);
                if (bhatt > search_hist_thresh) {
                    continue;
                }

                cv::Scalar mean_val = cv::mean(cand_crop);
                if (mean_val[0] < 15.0 && mean_val[1] < 15.0 && mean_val[2] < 15.0) {
                    continue;
                }

                FinalComponent fc_new;
                fc_new.id = next_id++;
                fc_new.source = ComponentSource::SIMILARITY_DISCOVERED;
                fc_new.label = cl.winning_label;
                fc_new.confidence = std::min(0.92f, std::max(0.70f, score));
                fc_new.box = cand_box;
                fc_new.center = cand_center;
                fc_new.shape = rep_comp->shape;
                fc_new.rotated_box = cv::RotatedRect(cand_center, cand_box.size(), cur_angle);
                fc_new.radius = (fc_new.shape == ShapeType::CIRCLE) ? (cand_box.width * 0.5f) : 0.0f;
                fc_new.area = cand_box.width * cand_box.height;
                fc_new.notes = "Nuovo componente scoperto via ricerca visiva su scheda (Cluster #" +
                               std::to_string(cl.cluster_id) + ", Score NCC: " +
                               std::to_string(score).substr(0, 4) + ")";

                discovered.push_back(fc_new);
                occupied_boxes.push_back(cand_box);
            }
        }
    }

    return discovered;
}

} // namespace pcb

