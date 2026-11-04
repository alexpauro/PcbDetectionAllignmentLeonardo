#include "output_processor.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

namespace pcb {

namespace {

// Helper per conversione stringa in minuscolo
std::string toLower(const std::string& str) {
    std::string s = str;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

// Colori fissi per la visualizzazione geometrica di Step 3
const cv::Scalar COLOR_FUSED(0, 255, 0);          // Verde brillante: FUSED_DL_CV
const cv::Scalar COLOR_DL_ONLY(255, 200, 0);      // Ciano/Azzurro: DL_ONLY
const cv::Scalar COLOR_CV_ONLY(255, 0, 255);      // Magenta/Viola: CV_ONLY
const cv::Scalar COLOR_SIMILARITY(0, 165, 255);   // Arancione brillante: SIMILARITY_DISCOVERED
const cv::Scalar COLOR_ELEMENT(120, 255, 0);      // Verde chiaro Step 1

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

    // 2. Controllo istogramma colore HSV (distanza Bhattacharyya: 0 = identico, 1 = completamente diverso)
    double bhatt = cv::compareHist(f1.hist_hsv, f2.hist_hsv, cv::HISTCMP_BHATTACHARYYA);
    if (bhatt > hist_thresh) {
        return false;
    }

    // 3. Normalized Cross Correlation (NCC) con orientamento standard (0°) e ruotato di 180°
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

} // namespace

std::string OutputProcessor::sourceToString(ComponentSource src) {
    switch (src) {
        case ComponentSource::FUSED_DL_CV: return "FUSED_DL_CV";
        case ComponentSource::DL_ONLY: return "DL_ONLY";
        case ComponentSource::CV_ONLY: return "CV_ONLY";
        case ComponentSource::SIMILARITY_DISCOVERED: return "SIMILARITY_DISCOVERED";
        default: return "UNKNOWN";
    }
}

float OutputProcessor::calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float inter_w = std::max(0.0f, x2 - x1);
    float inter_h = std::max(0.0f, y2 - y1);
    float inter_area = inter_w * inter_h;

    float area_a = a.width * a.height;
    float area_b = b.width * b.height;
    float union_area = area_a + area_b - inter_area;

    return (union_area > 0.0f) ? (inter_area / union_area) : 0.0f;
}

float OutputProcessor::calculateOverlap(const cv::Rect2f& a, const cv::Rect2f& b) {
    float x1 = std::max(a.x, b.x);
    float y1 = std::max(a.y, b.y);
    float x2 = std::min(a.x + a.width, b.x + b.width);
    float y2 = std::min(a.y + a.height, b.y + b.height);

    float inter_w = std::max(0.0f, x2 - x1);
    float inter_h = std::max(0.0f, y2 - y1);
    float inter_area = inter_w * inter_h;

    float min_area = std::min(a.width * a.height, b.width * b.height);
    return (min_area > 0.0f) ? (inter_area / min_area) : 0.0f;
}

std::vector<ComponentDetection> OutputProcessor::loadDlDetectionsCsv(const std::string& csv_path) {
    std::vector<ComponentDetection> dets;
    if (!fs::exists(csv_path)) return dets;

    std::ifstream file(csv_path);
    if (!file.is_open()) return dets;

    std::string line;
    // Salta l'intestazione
    if (!std::getline(file, line)) return dets;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 8) {
            try {
                ComponentDetection det;
                det.id = std::stoi(tokens[0]);
                det.label = tokens[1];
                det.box_conf = std::stof(tokens[2]);
                det.label_conf = std::stof(tokens[3]);
                float x0 = std::stof(tokens[4]);
                float y0 = std::stof(tokens[5]);
                float x1 = std::stof(tokens[6]);
                float y1 = std::stof(tokens[7]);
                det.box = cv::Rect2f(x0, y0, std::max(1.0f, x1 - x0), std::max(1.0f, y1 - y0));

                const auto& default_classes = ComponentDetector::getDefaultClasses();
                auto it = std::find(default_classes.begin(), default_classes.end(), det.label);
                det.class_id = (it != default_classes.end()) ? static_cast<int>(std::distance(default_classes.begin(), it)) : -1;

                dets.push_back(det);
            } catch (...) {
                continue;
            }
        }
    }
    return dets;
}

std::vector<CvComponent> OutputProcessor::loadCvComponentsCsv(const std::string& csv_path) {
    std::vector<CvComponent> comps;
    if (!fs::exists(csv_path)) return comps;

    std::ifstream file(csv_path);
    if (!file.is_open()) return comps;

    std::string line;
    // Salta intestazione
    if (!std::getline(file, line)) return comps;

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 10) {
            try {
                CvComponent c;
                c.id = std::stoi(tokens[0]);
                std::string shape_str = toLower(tokens[1]);
                c.shape = (shape_str == "circle") ? ShapeType::CIRCLE : ShapeType::RECTANGLE;
                c.center.x = std::stof(tokens[2]);
                c.center.y = std::stof(tokens[3]);
                float w = std::stof(tokens[4]);
                float h = std::stof(tokens[5]);
                float angle = std::stof(tokens[6]);
                c.radius = std::stof(tokens[7]);
                c.area = std::stof(tokens[8]);
                c.circularity = std::stof(tokens[9]);

                c.rotated_box = cv::RotatedRect(c.center, cv::Size2f(w, h), angle);
                comps.push_back(c);
            } catch (...) {
                continue;
            }
        }
    }
    return comps;
}

std::map<std::string, std::vector<ComponentDetection>> OutputProcessor::loadAllDlDetectionsInDir(const std::string& dir) {
    std::map<std::string, std::vector<ComponentDetection>> all_dets;
    if (!fs::exists(dir)) return all_dets;

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            std::string suffix = "_detections.csv";
            if (filename.size() > suffix.size() &&
                filename.substr(filename.size() - suffix.size()) == suffix) {
                std::string stem = filename.substr(0, filename.size() - suffix.size());
                all_dets[stem] = loadDlDetectionsCsv(entry.path().string());
            }
        }
    }
    return all_dets;
}

// =========================================================================
// STEP 1: Fusione multi-esposizione DL
// =========================================================================
std::vector<FusedDlComponent> OutputProcessor::mergeMultiExposureDl(
    const std::map<std::string, std::vector<ComponentDetection>>& all_image_detections,
    const std::string& best_image_stem,
    float iou_threshold) {

    if (all_image_detections.empty()) return {};

    // Struttura ausiliaria per accumulare le osservazioni dello stesso componente
    struct DlCluster {
        cv::Rect2f anchor_box;
        std::vector<MultiExposureObservation> observations;
    };

    std::vector<DlCluster> clusters;

    // Ordiniamo le immagini in modo che l'immagine di riferimento con più componenti venga elaborata per prima
    std::vector<std::pair<std::string, size_t>> ordered_images;
    for (const auto& [stem, dets] : all_image_detections) {
        ordered_images.emplace_back(stem, dets.size());
    }
    std::sort(ordered_images.begin(), ordered_images.end(),
              [&best_image_stem](const auto& a, const auto& b) {
                  if (a.first == best_image_stem) return true;
                  if (b.first == best_image_stem) return false;
                  return a.second > b.second;
              });

    for (const auto& [stem, count] : ordered_images) {
        const auto& dets = all_image_detections.at(stem);
        for (const auto& det : dets) {
            float best_iou = 0.0f;
            int best_cluster_idx = -1;

            for (size_t c = 0; c < clusters.size(); ++c) {
                float iou = calculateIoU(det.box, clusters[c].anchor_box);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_cluster_idx = static_cast<int>(c);
                }
            }

            // Se l'IoU supera la soglia, abbina al cluster
            bool matched = false;
            if (best_iou >= iou_threshold && best_cluster_idx >= 0) {
                matched = true;
            } else {
                // Controllo supplementare: centro contenuto e alto overlap
                for (size_t c = 0; c < clusters.size(); ++c) {
                    const auto& ab = clusters[c].anchor_box;
                    cv::Point2f det_center(det.box.x + det.box.width * 0.5f, det.box.y + det.box.height * 0.5f);
                    cv::Point2f cl_center(ab.x + ab.width * 0.5f, ab.y + ab.height * 0.5f);

                    bool center_in_cluster = ab.contains(det_center);
                    bool cluster_center_in_det = det.box.contains(cl_center);

                    if ((center_in_cluster || cluster_center_in_det) && calculateOverlap(det.box, ab) >= 0.50f) {
                        best_cluster_idx = static_cast<int>(c);
                        matched = true;
                        break;
                    }
                }
            }

            if (matched && best_cluster_idx >= 0) {
                clusters[best_cluster_idx].observations.push_back({stem, det});
            } else {
                // Crea un nuovo cluster
                DlCluster new_cl;
                new_cl.anchor_box = det.box;
                new_cl.observations.push_back({stem, det});
                clusters.push_back(new_cl);
            }
        }
    }

    std::vector<FusedDlComponent> merged_results;
    merged_results.reserve(clusters.size());
    int progressive_id = 1;

    for (const auto& cl : clusters) {
        if (cl.observations.empty()) continue;

        FusedDlComponent fused;
        fused.id = progressive_id++;
        fused.total_observations = static_cast<int>(cl.observations.size());

        // Raccogliamo i voti di classe
        std::map<std::string, int> vote_map;
        std::map<std::string, float> conf_sum_map;
        std::map<std::string, float> conf_max_map;
        std::map<std::string, const MultiExposureObservation*> best_obs_per_label;

        int known_votes_count = 0;
        float max_box_conf = 0.0f;

        for (const auto& obs : cl.observations) {
            const std::string& lbl = obs.detection.label;
            vote_map[lbl]++;
            conf_sum_map[lbl] += obs.detection.label_conf;
            if (obs.detection.label_conf >= conf_max_map[lbl]) {
                conf_max_map[lbl] = obs.detection.label_conf;
                best_obs_per_label[lbl] = &obs;
            }

            if (lbl != "?") {
                known_votes_count++;
            }
            if (obs.detection.box_conf > max_box_conf) {
                max_box_conf = obs.detection.box_conf;
            }
        }

        fused.votes = vote_map;
        fused.box_conf = max_box_conf;

        // =====================================================================
        // REGOLE DI RISOLUZIONE:
        // 1) Priorità assoluta ai componenti catalogati SENZA punto di domanda
        // 2) Se catalogato con classi diverse: priorità alla classe con più voti
        // 3) In caso di parità di voti: vince la classe con confidenza maggiore
        // =====================================================================
        std::string winning_label = "?";
        int max_votes = 0;
        float winning_conf = 0.0f;

        if (known_votes_count > 0) {
            // Consideriamo SOLTANTO le classi note (escludiamo "?")
            for (const auto& [lbl, count] : vote_map) {
                if (lbl == "?") continue;

                float avg_conf = conf_sum_map[lbl] / count;
                if (count > max_votes) {
                    max_votes = count;
                    winning_label = lbl;
                    winning_conf = conf_max_map[lbl];
                } else if (count == max_votes) {
                    // Tie-break: vince chi ha confidenza maggiore
                    if (avg_conf > (conf_sum_map[winning_label] / max_votes)) {
                        winning_label = lbl;
                        winning_conf = conf_max_map[lbl];
                    }
                }
            }
        } else {
            // Tutte le foto hanno catalogato come "?"
            winning_label = "?";
            max_votes = vote_map["?"];
            winning_conf = 0.0f;
        }

        fused.label = winning_label;
        fused.vote_count = max_votes;
        fused.label_conf = winning_conf;

        // Determina la bounding box rappresentativa
        const MultiExposureObservation* chosen_obs = nullptr;
        if (best_obs_per_label.find(winning_label) != best_obs_per_label.end()) {
            chosen_obs = best_obs_per_label[winning_label];
        } else if (!cl.observations.empty()) {
            chosen_obs = &cl.observations[0];
        }

        if (chosen_obs) {
            fused.box = chosen_obs->detection.box;
            fused.class_id = chosen_obs->detection.class_id;
            fused.best_source_image = chosen_obs->image_stem;
        } else {
            fused.box = cl.anchor_box;
            fused.class_id = -1;
            fused.best_source_image = best_image_stem;
        }

        // Se l'immagine di riferimento 'best_image_stem' conteneva questo componente, usiamo preferenzialmente la sua box
        for (const auto& obs : cl.observations) {
            if (obs.image_stem == best_image_stem && obs.detection.label == winning_label) {
                fused.box = obs.detection.box;
                break;
            }
        }

        merged_results.push_back(fused);
    }

    return merged_results;
}

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

// =========================================================================
// STEP 3: Fusione finale completa (DL + CV)
// =========================================================================
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

            // REQUISITO UTENTE:
            // "inoltre possono esserci componenti molto vicini, escludi solo i componenti che includono
            // completamente componenti più piccoli (togli quello più grande, ricorda che il dl ha sempre ragione, il cv no)"
            bool is_completely_enclosed = (overlap_smaller >= enclosure_thresh) ||
                                          (center_inside && overlap_smaller >= 0.70f && area_ratio >= 1.5f);

            if (is_completely_enclosed) {
                bool larger_is_dl  = (comps[idx_larger].source != ComponentSource::CV_ONLY);
                bool smaller_is_dl = (comps[idx_smaller].source != ComponentSource::CV_ONLY);

                if (larger_is_dl && !smaller_is_dl) {
                    // DL ha sempre ragione, CV no -> togli CV (più piccolo)
                    keep[idx_smaller] = false;
                } else if (!larger_is_dl && smaller_is_dl) {
                    // DL ha sempre ragione, CV no -> togli CV (più grande)
                    keep[idx_larger] = false;
                } else {
                    // Entrambi della stessa fonte (entrambi DL o entrambi CV): togli quello più grande
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
        // REQUISITO UTENTE:
        // "quello riconosciuto da dl è sempre più accurato, quindi mantieni ciò che ha riconosciuto."
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
            // Se DL non ha una classe specifica ('?') ma CV ne ha una riconosciuta dai pesi DL, usala
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

        // REQUISITO UTENTE:
        // "se ciò che dl riconosce è sopra a cv, allora togli cv."
        // Rimuoviamo qualsiasi componente CV che si trova sotto o si sovrappone a questo componente DL
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

                // Se DL è sopra a CV (inclusione di centro o sovrapposizione significativa), togli CV
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

            // REQUISITO UTENTE:
            // "i componenti non catalogati in cv non considerarli nella fusione"
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

    // REQUISITO UTENTE:
    // "se ci sono componenti più grandi che racchiudono componenti più piccoli allora c'è un errore:
    // non ci sono componenti sovrapposti, possono essere molto vicini ma non sovrapposti:
    // tieni gli elementi con maggiore confidenza e più piccoli"
    std::vector<FinalComponent> non_overlapping_list = resolveEnclosingAndOverlaps(final_list);

    // Ordina da cima a fondo, poi da sinistra a destra per leggibilità
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

        // Se tutti i componenti del cluster sono '?', non possiamo assegnare una classe certa
        if (class_counts.empty()) continue;

        // Trova la classe vincente (maggioranza di rilevamenti, con confidenza come tie-break)
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

    // Bounding box occupate (esistenti + nuove scoperte) per prevenire sovrapposizioni
    std::vector<cv::Rect2f> occupied_boxes;
    occupied_boxes.reserve(existing_comps.size() + 100);
    for (const auto& ec : existing_comps) {
        occupied_boxes.push_back(ec.box);
    }

    int next_id = static_cast<int>(existing_comps.size() + 1);

    for (const auto& cl : clusters) {
        if (cl.winning_label == "?" || cl.member_count < 1) continue;

        // Trova il componente rappresentativo del cluster (massima confidenza con etichetta vincente)
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

        // Istogramma colore HSV 2D di riferimento
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

        // Template da testare: orizzontale e (se rettangolare allungato) verticale ruotato di 90°
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

            // Trova picchi locali (massimi locali >= search_ncc_thresh)
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

                // Controllo 1: sovrapposizione con componenti già noti o già scoperti
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

                // Controllo 2: verifica coerenza colore con istogramma HSV
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
                    continue; // Colore troppo differente da quello del componente
                }

                // Controllo 3: luminosità media (non su bordo nero esterno al PCB)
                cv::Scalar mean_val = cv::mean(cand_crop);
                if (mean_val[0] < 15.0 && mean_val[1] < 15.0 && mean_val[2] < 15.0) {
                    continue;
                }

                // Match valido trovato! Creiamo il nuovo componente
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

// =========================================================================
// Pannello Legenda Comune
// =========================================================================
cv::Mat OutputProcessor::createSideLegendPanel(
    const std::vector<std::pair<std::string, int>>& label_counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    int target_height,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats) {

    int line_h = 44;
    int needed_h = static_cast<int>((label_counts.size() + extra_stats.size() + 5) * line_h);
    int panel_h = std::max(target_height, needed_h);
    int panel_w = 440;

    cv::Mat panel(panel_h, panel_w, CV_8UC3, cv::Scalar(22, 22, 22));

    // Intestazione
    cv::rectangle(panel, cv::Rect(0, 0, panel_w, 64), cv::Scalar(35, 35, 35), cv::FILLED);
    cv::putText(panel, title, cv::Point(16, 42), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // Totale
    std::string total_str = "Totale Componenti: " + std::to_string(total_count);
    cv::putText(panel, total_str, cv::Point(16, 92), cv::FONT_HERSHEY_SIMPLEX, 0.72, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    cv::line(panel, cv::Point(16, 108), cv::Point(panel_w - 16, 108), cv::Scalar(70, 70, 70), 1);

    int cur_y = 140;

    // Se ci sono statistiche extra (es. FUSED, DL_ONLY, CV_ONLY)
    if (!extra_stats.empty()) {
        for (const auto& [stat_name, stat_val] : extra_stats) {
            cv::Scalar stat_color(200, 200, 200);
            if (stat_name.find("FUSED") != std::string::npos) stat_color = COLOR_FUSED;
            else if (stat_name.find("DL") != std::string::npos) stat_color = COLOR_DL_ONLY;
            else if (stat_name.find("CV") != std::string::npos) stat_color = COLOR_CV_ONLY;

            cv::circle(panel, cv::Point(26, cur_y - 6), 6, stat_color, -1, cv::LINE_AA);
            std::string row_str = stat_name + ": " + std::to_string(stat_val);
            cv::putText(panel, row_str, cv::Point(44, cur_y), cv::FONT_HERSHEY_SIMPLEX, 0.58, stat_color, 1, cv::LINE_AA);
            cur_y += 32;
        }
        cv::line(panel, cv::Point(16, cur_y), cv::Point(panel_w - 16, cur_y), cv::Scalar(70, 70, 70), 1);
        cur_y += 32;
    }

    // Elenco Classi
    for (const auto& [label, count] : label_counts) {
        cv::Scalar color(180, 180, 180);
        auto it = palette.find(label);
        if (it != palette.end()) {
            color = it->second;
        }

        // Quadratino colorato della classe
        cv::rectangle(panel, cv::Rect(20, cur_y - 18, 20, 20), color, cv::FILLED);
        cv::rectangle(panel, cv::Rect(20, cur_y - 18, 20, 20), cv::Scalar(255, 255, 255), 1);

        std::string label_text = label + "  (" + std::to_string(count) + ")";
        cv::putText(panel, label_text, cv::Point(52, cur_y - 3), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);

        cur_y += 36;
    }

    return panel;
}

// =========================================================================
// Rendering Grafico Passaggi Intermedi e Finali
// =========================================================================
cv::Mat OutputProcessor::renderStep1Elements(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        cv::Rect r(static_cast<int>(std::round(c.box.x)),
                    static_cast<int>(std::round(c.box.y)),
                    static_cast<int>(std::round(c.box.width)),
                    static_cast<int>(std::round(c.box.height)));
        r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (r.width > 0 && r.height > 0) {
            cv::rectangle(canvas, r, COLOR_ELEMENT, 2, cv::LINE_AA);
            std::string id_str = "#" + std::to_string(c.id);
            cv::putText(canvas, id_str, cv::Point(r.x, std::max(12, r.y - 4)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.42, COLOR_ELEMENT, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);

    std::string text = "Step 1: Fusione Multi-Esposizione DL (Posizioni) | Totale Componenti Unificati: " +
                       std::to_string(comps.size());
    cv::putText(canvas, text, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    return canvas;
}

cv::Mat OutputProcessor::renderStep1Types(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::vector<std::string> all_labels;
    std::map<std::string, int> counts;
    for (const auto& c : comps) {
        counts[c.label]++;
        all_labels.push_back(c.label);
    }

    auto palette = ComponentDetector::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Rect r(static_cast<int>(std::round(c.box.x)),
                    static_cast<int>(std::round(c.box.y)),
                    static_cast<int>(std::round(c.box.width)),
                    static_cast<int>(std::round(c.box.height)));
        r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (r.width <= 0 || r.height <= 0) continue;

        cv::Scalar col = palette[c.label];
        cv::rectangle(canvas, r, col, 2, cv::LINE_AA);

        std::string short_l = ComponentDetector::getShortLabel(c.label);
        std::string txt = "#" + std::to_string(c.id) + " " + short_l;

        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseLine);
        cv::Point pt(r.x, std::max(sz.height + 2, r.y - 4));
        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(20, 20, 20), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string btext = "Step 1: Fusione Multi-Esposizione DL (Classificazione con Priorita' no-? e Voti)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b){ return a.second > b.second; });

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows, "Classi DL Fuse");

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

cv::Mat OutputProcessor::renderStep2CvElements(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        if (c.cv_geom.shape == ShapeType::CIRCLE) {
            int rad = std::max(1, static_cast<int>(std::round(c.cv_geom.radius)));
            cv::circle(canvas, c.cv_geom.center, rad, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            cv::circle(canvas, c.cv_geom.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.cv_geom.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.cv_geom.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }
    }

    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string text = "Step 2: Componenti Computer Vision (Cerchi e Rettangoli Ruotati Aderenti) | Totale=" +
                       std::to_string(comps.size());
    cv::putText(canvas, text, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    return canvas;
}

cv::Mat OutputProcessor::renderStep2CvTypes(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::vector<std::string> all_labels;
    std::map<std::string, int> counts;
    for (const auto& c : comps) {
        counts[c.label]++;
        all_labels.push_back(c.label);
    }

    auto palette = ComponentDetector::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Scalar col = palette[c.label];

        if (c.cv_geom.shape == ShapeType::CIRCLE) {
            int rad = std::max(1, static_cast<int>(std::round(c.cv_geom.radius)));
            cv::circle(canvas, c.cv_geom.center, rad, col, 2, cv::LINE_AA);
            cv::circle(canvas, c.cv_geom.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.cv_geom.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.cv_geom.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }

        std::string short_l = ComponentDetector::getShortLabel(c.label);
        std::string txt = "#" + std::to_string(c.id) + " " + short_l;

        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.40, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.cv_geom.center.x - sz.width / 2),
                     static_cast<int>(c.cv_geom.center.y - 8));
        if (pt.y < 12) pt.y = static_cast<int>(c.cv_geom.center.y + 16);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(20, 20, 20), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv::LINE_AA);
        }
    }

    // Banner
    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string btext = "Step 2: Componenti CV Riconosciuti tramite Pesi Deep Learning";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b){ return a.second > b.second; });

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows, "Classi CV (DL)");

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

cv::Mat OutputProcessor::renderStep3FinalElements(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(1, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, COLOR_ELEMENT, 2, cv::LINE_AA);
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], COLOR_ELEMENT, 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }

        std::string id_str = "#" + std::to_string(c.id);
        cv::putText(canvas, id_str, cv::Point(static_cast<int>(c.center.x - 8), static_cast<int>(c.center.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, COLOR_ELEMENT, 1, cv::LINE_AA);
    }

    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string text = "Step 3 (Finale): Tutti i Componenti Identificati (WHERE) | Totale=" + std::to_string(comps.size());
    cv::putText(canvas, text, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    return canvas;
}

cv::Mat OutputProcessor::renderStep3FinalTypes(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::vector<std::string> all_labels;
    std::map<std::string, int> counts;
    int n_fused = 0, n_dl = 0, n_cv = 0;

    for (const auto& c : comps) {
        counts[c.label]++;
        all_labels.push_back(c.label);
        if (c.source == ComponentSource::FUSED_DL_CV) n_fused++;
        else if (c.source == ComponentSource::DL_ONLY) n_dl++;
        else if (c.source == ComponentSource::CV_ONLY) n_cv++;
    }

    auto palette = ComponentDetector::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Scalar col = palette[c.label];

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(1, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }

        std::string short_l = ComponentDetector::getShortLabel(c.label);
        std::string txt = "#" + std::to_string(c.id) + " " + short_l;

        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 8));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 16);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(20, 20, 20), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv::LINE_AA);
        }
    }

    // Banner
    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string btext = "Step 3 (Finale Completo): Fusione Deep Learning Multi-Esposizione + Computer Vision";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.62, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b){ return a.second > b.second; });

    std::vector<std::pair<std::string, int>> extra_stats = {
        {"FUSED (DL + CV)", n_fused},
        {"DL ONLY", n_dl},
        {"CV ONLY (DL classified)", n_cv}
    };

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows,
                                           "Riepilogo Finale", extra_stats);

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

cv::Mat OutputProcessor::renderStep3Geometric(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    int n_fused = 0, n_dl = 0, n_cv = 0, n_sim = 0;

    for (const auto& c : comps) {
        cv::Scalar col;
        if (c.source == ComponentSource::FUSED_DL_CV) {
            col = COLOR_FUSED;
            n_fused++;
        } else if (c.source == ComponentSource::DL_ONLY) {
            col = COLOR_DL_ONLY;
            n_dl++;
        } else if (c.source == ComponentSource::SIMILARITY_DISCOVERED) {
            col = COLOR_SIMILARITY;
            n_sim++;
        } else {
            col = COLOR_CV_ONLY;
            n_cv++;
        }

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(1, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }

        std::string txt = "#" + std::to_string(c.id) + " (" + sourceToString(c.source) + ")";
        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.38, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 8));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 16);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(20, 20, 20), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv::LINE_AA);
        }
    }

    // Banner
    cv::Rect banner_rect(0, 0, canvas.cols, 36);
    cv::Mat overlay = canvas(banner_rect);
    cv::Mat dark(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, overlay, 0.18, 0.0, overlay);
    std::string btext = "Step 3 (Geometrico): Origine Rilevamento (Verde=Fuso DL+CV, Azzurro=Solo DL, Magenta=Solo CV, Arancio=Ricerca Simili)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> stats = {
        {"FUSED (DL + CV)", n_fused},
        {"DL ONLY", n_dl},
        {"CV ONLY", n_cv},
        {"RICERCA SIMILI", n_sim}
    };

    std::map<std::string, cv::Scalar> pal = {
        {"FUSED (DL + CV)", COLOR_FUSED},
        {"DL ONLY", COLOR_DL_ONLY},
        {"CV ONLY", COLOR_CV_ONLY},
        {"RICERCA SIMILI", COLOR_SIMILARITY}
    };

    cv::Mat legend = createSideLegendPanel(stats, static_cast<int>(comps.size()), pal, canvas.rows, "Fonti Rilevamento");

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

// =========================================================================
// Salvataggio di tutti i passaggi intermedi
// =========================================================================
bool OutputProcessor::saveAllStepResults(
    const OutputProcessingResult& res,
    const std::string& output_dir) {

    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[OutputProcessor] ERRORE: Impossibile creare cartella " << output_dir << ": " << ec.message() << std::endl;
        return false;
    }

    fs::path dir(output_dir);
    std::vector<int> jpg_params = {cv::IMWRITE_JPEG_QUALITY, 93};

    std::cout << "\n[OutputProcessor] Salvataggio passaggi intermedi in: " << output_dir << std::endl;

    // -------------------------------------------------------------------------
    // STEP 1: Fusione Multi-Esposizione DL
    // -------------------------------------------------------------------------
    if (!res.step1_elements_vis.empty()) {
        std::string path = (dir / "step1_merged_dl_elements.jpg").string();
        cv::imwrite(path, res.step1_elements_vis, jpg_params);
        std::cout << "  -> [Step 1] Immagine elementi DL: " << path << std::endl;
    }

    if (!res.step1_types_vis.empty()) {
        std::string path = (dir / "step1_merged_dl_types.jpg").string();
        cv::imwrite(path, res.step1_types_vis, jpg_params);
        std::cout << "  -> [Step 1] Immagine tipi DL: " << path << std::endl;
    }

    // CSV Step 1
    {
        std::string csv_path = (dir / "step1_merged_dl_detections.csv").string();
        std::ofstream csv(csv_path);
        if (csv.is_open()) {
            csv << "id,label,vote_count,total_photos,box_conf,label_conf,x0,y0,x1,y1,best_photo\n";
            for (const auto& c : res.step1_dl_merged) {
                int x0 = static_cast<int>(std::round(c.box.x));
                int y0 = static_cast<int>(std::round(c.box.y));
                int x1 = static_cast<int>(std::round(c.box.x + c.box.width));
                int y1 = static_cast<int>(std::round(c.box.y + c.box.height));
                csv << c.id << "," << c.label << "," << c.vote_count << "," << c.total_observations << ","
                    << std::fixed << std::setprecision(3) << c.box_conf << "," << c.label_conf << ","
                    << x0 << "," << y0 << "," << x1 << "," << y1 << "," << c.best_source_image << "\n";
            }
            std::cout << "  -> [Step 1] Report CSV: " << csv_path << std::endl;
        }
    }

    // JSON Step 1
    {
        std::string json_path = (dir / "step1_merged_dl_summary.json").string();
        std::ofstream jf(json_path);
        if (jf.is_open()) {
            jf << "{\n";
            jf << "  \"step\": 1,\n";
            jf << "  \"description\": \"Fusione multi-esposizione rilevamenti Deep Learning\",\n";
            jf << "  \"best_reference_image\": \"" << res.best_image_stem << "\",\n";
            jf << "  \"total_merged_components\": " << res.step1_dl_merged.size() << ",\n";
            jf << "  \"counts\": {\n";
            size_t idx = 0;
            for (auto it = res.step1_counts.begin(); it != res.step1_counts.end(); ++it, ++idx) {
                jf << "    \"" << it->first << "\": " << it->second;
                if (idx + 1 < res.step1_counts.size()) jf << ",";
                jf << "\n";
            }
            jf << "  }\n";
            jf << "}\n";
            std::cout << "  -> [Step 1] Riepilogo JSON: " << json_path << std::endl;
        }
    }

    // -------------------------------------------------------------------------
    // STEP 2: Componenti CV Classificati tramite Pesi DL
    // -------------------------------------------------------------------------
    if (!res.step2_cv_elements_vis.empty()) {
        std::string path = (dir / "step2_cv_dl_elements.jpg").string();
        cv::imwrite(path, res.step2_cv_elements_vis, jpg_params);
        std::cout << "  -> [Step 2] Immagine elementi CV: " << path << std::endl;
    }

    if (!res.step2_cv_types_vis.empty()) {
        std::string path = (dir / "step2_cv_dl_classified.jpg").string();
        cv::imwrite(path, res.step2_cv_types_vis, jpg_params);
        std::cout << "  -> [Step 2] Immagine CV classificati DL: " << path << std::endl;
    }

    // CSV Step 2
    {
        std::string csv_path = (dir / "step2_cv_dl_detections.csv").string();
        std::ofstream csv(csv_path);
        if (csv.is_open()) {
            csv << "id,shape,label,confidence,center_x,center_y,width,height,angle_deg,radius,area,method,matched_dl\n";
            for (const auto& c : res.step2_cv_classified) {
                csv << c.id << ","
                    << (c.cv_geom.shape == ShapeType::CIRCLE ? "circle" : "rectangle") << ","
                    << c.label << ","
                    << std::fixed << std::setprecision(3) << c.label_conf << ","
                    << std::fixed << std::setprecision(2)
                    << c.cv_geom.center.x << "," << c.cv_geom.center.y << ","
                    << c.cv_geom.rotated_box.size.width << "," << c.cv_geom.rotated_box.size.height << ","
                    << c.cv_geom.rotated_box.angle << "," << c.cv_geom.radius << "," << c.cv_geom.area << ","
                    << c.classification_method << "," << (c.matched_with_dl ? "true" : "false") << "\n";
            }
            std::cout << "  -> [Step 2] Report CSV: " << csv_path << std::endl;
        }
    }

    // JSON Step 2
    {
        std::string json_path = (dir / "step2_cv_dl_summary.json").string();
        std::ofstream jf(json_path);
        if (jf.is_open()) {
            jf << "{\n";
            jf << "  \"step\": 2,\n";
            jf << "  \"description\": \"Componenti Computer Vision classificati con pesi Deep Learning\",\n";
            jf << "  \"total_cv_components\": " << res.step2_cv_classified.size() << ",\n";
            jf << "  \"counts\": {\n";
            size_t idx = 0;
            for (auto it = res.step2_counts.begin(); it != res.step2_counts.end(); ++it, ++idx) {
                jf << "    \"" << it->first << "\": " << it->second;
                if (idx + 1 < res.step2_counts.size()) jf << ",";
                jf << "\n";
            }
            jf << "  }\n";
            jf << "}\n";
            std::cout << "  -> [Step 2] Riepilogo JSON: " << json_path << std::endl;
        }
    }

    // -------------------------------------------------------------------------
    // STEP 3: Risultato Finale Unificato Completo
    // -------------------------------------------------------------------------
    if (!res.step3_final_elements_vis.empty()) {
        std::string path = (dir / "step3_final_fused_elements.jpg").string();
        cv::imwrite(path, res.step3_final_elements_vis, jpg_params);
        std::cout << "  -> [Step 3] Immagine finale elementi: " << path << std::endl;
    }

    if (!res.step3_final_types_vis.empty()) {
        std::string path = (dir / "step3_final_fused_types.jpg").string();
        cv::imwrite(path, res.step3_final_types_vis, jpg_params);
        std::cout << "  -> [Step 3] Immagine finale tipi con legenda: " << path << std::endl;
    }

    if (!res.step3_final_geometric_vis.empty()) {
        std::string path = (dir / "step3_final_fused_geometric.jpg").string();
        cv::imwrite(path, res.step3_final_geometric_vis, jpg_params);
        std::cout << "  -> [Step 3] Immagine finale geometrica per fonti: " << path << std::endl;
    }

    // CSV Finale
    {
        std::string csv_path = (dir / "final_fused_components.csv").string();
        std::ofstream csv(csv_path);
        if (csv.is_open()) {
            csv << "id,source,label,confidence,shape,center_x,center_y,width,height,angle_deg,radius,area,x0,y0,x1,y1,notes\n";
            for (const auto& c : res.step3_final_components) {
                int x0 = static_cast<int>(std::round(c.box.x));
                int y0 = static_cast<int>(std::round(c.box.y));
                int x1 = static_cast<int>(std::round(c.box.x + c.box.width));
                int y1 = static_cast<int>(std::round(c.box.y + c.box.height));

                csv << c.id << ","
                    << sourceToString(c.source) << ","
                    << c.label << ","
                    << std::fixed << std::setprecision(3) << c.confidence << ","
                    << (c.shape == ShapeType::CIRCLE ? "circle" : "rectangle") << ","
                    << std::fixed << std::setprecision(2)
                    << c.center.x << "," << c.center.y << ","
                    << c.rotated_box.size.width << "," << c.rotated_box.size.height << ","
                    << c.rotated_box.angle << "," << c.radius << "," << c.area << ","
                    << x0 << "," << y0 << "," << x1 << "," << y1 << ",\""
                    << c.notes << "\"\n";
            }
            std::cout << "  -> [Step 3] Report CSV Finale Completo: " << csv_path << std::endl;
        }
    }

    // JSON Finale
    {
        std::string json_path = (dir / "final_fused_summary.json").string();
        std::ofstream jf(json_path);
        if (jf.is_open()) {
            jf << "{\n";
            jf << "  \"board\": \"" << res.board_name << "\",\n";
            jf << "  \"side\": \"" << res.side << "\",\n";
            jf << "  \"best_reference_image\": \"" << res.best_image_stem << "\",\n";
            jf << "  \"total_final_components\": " << res.step3_final_components.size() << ",\n";
            jf << "  \"sources_breakdown\": {\n";
            size_t sidx = 0;
            for (auto it = res.source_counts.begin(); it != res.source_counts.end(); ++it, ++sidx) {
                jf << "    \"" << it->first << "\": " << it->second;
                if (sidx + 1 < res.source_counts.size()) jf << ",";
                jf << "\n";
            }
            jf << "  },\n";
            jf << "  \"class_counts\": {\n";
            size_t cidx = 0;
            for (auto it = res.step3_final_counts.begin(); it != res.step3_final_counts.end(); ++it, ++cidx) {
                jf << "    \"" << it->first << "\": " << it->second;
                if (cidx + 1 < res.step3_final_counts.size()) jf << ",";
                jf << "\n";
            }
            jf << "  },\n";
            jf << "  \"similarity_harmonization\": {\n";
            jf << "    \"clusters_count\": " << res.similarity_clusters.size() << ",\n";
            jf << "    \"harmonized_components_count\": " << res.harmonized_components_count << ",\n";
            jf << "    \"discovered_components_count\": " << res.discovered_components_count << ",\n";
            jf << "    \"clusters\": [\n";
            for (size_t ci = 0; ci < res.similarity_clusters.size(); ++ci) {
                const auto& cl = res.similarity_clusters[ci];
                jf << "      {\n";
                jf << "        \"cluster_id\": " << cl.cluster_id << ",\n";
                jf << "        \"winning_label\": \"" << cl.winning_label << "\",\n";
                jf << "        \"consensus_confidence\": " << std::fixed << std::setprecision(3) << cl.consensus_confidence << ",\n";
                jf << "        \"member_count\": " << cl.member_count << ",\n";
                jf << "        \"harmonized_count\": " << cl.harmonized_count << ",\n";
                jf << "        \"original_distribution\": {";
                size_t od_i = 0;
                for (const auto& [lbl, cnt] : cl.original_labels) {
                    jf << "\"" << lbl << "\": " << cnt;
                    if (++od_i < cl.original_labels.size()) jf << ", ";
                }
                jf << "},\n";
                jf << "        \"member_ids\": [";
                for (size_t mi = 0; mi < cl.member_ids.size(); ++mi) {
                    jf << cl.member_ids[mi];
                    if (mi + 1 < cl.member_ids.size()) jf << ", ";
                }
                jf << "]\n";
                jf << "      }";
                if (ci + 1 < res.similarity_clusters.size()) jf << ",";
                jf << "\n";
            }
            jf << "    ]\n";
            jf << "  }\n";
            jf << "}\n";
            std::cout << "  -> [Step 3] Riepilogo JSON Finale Completo: " << json_path << std::endl;
        }
    }

    return true;
}

// =========================================================================
// Pipeline Completa Lato Scheda
// =========================================================================
OutputProcessingResult OutputProcessor::processBoardSide(
    const std::string& board_name,
    const std::string& side,
    const std::string& images_dir,
    const std::string& dl_output_dir,
    const std::string& cv_output_dir,
    ComponentDetector* dl_detector,
    CvComponentDetector* cv_detector,
    const OutputProcessingConfig& config) {

    OutputProcessingResult res;
    res.board_name = board_name;
    res.side = side;

    std::cout << "\n========================================================" << std::endl;
    std::cout << " ELABORAZIONE OUTPUT E FUSIONE MULTI-ESPOSIZIONE        " << std::endl;
    std::cout << " Scheda: " << board_name << " | Lato: " << side << std::endl;
    std::cout << "========================================================" << std::endl;

    // 1) Raccogliamo tutte le immagini disponibili per questo lato
    std::vector<std::string> valid_exts = {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"};
    std::vector<fs::path> image_paths;
    if (fs::exists(images_dir)) {
        for (const auto& entry : fs::directory_iterator(images_dir)) {
            if (entry.is_regular_file()) {
                std::string ext = toLower(entry.path().extension().string());
                if (std::find(valid_exts.begin(), valid_exts.end(), ext) != valid_exts.end()) {
                    image_paths.push_back(entry.path());
                }
            }
        }
    }
    std::sort(image_paths.begin(), image_paths.end());

    // 2) Carichiamo o eseguiamo i rilevamenti DL per ciascuna foto
    std::map<std::string, std::vector<ComponentDetection>> all_dl_dets;
    if (fs::exists(dl_output_dir)) {
        all_dl_dets = loadAllDlDetectionsInDir(dl_output_dir);
    }

    // Se mancano rilevamenti DL e abbiamo il detector, li eseguiamo al volo
    if (all_dl_dets.empty() && dl_detector != nullptr && !image_paths.empty()) {
        std::cout << "[OutputProcessor] Rilevamenti DL non trovati su disco, esecuzione al volo..." << std::endl;
        DetectionConfig dcfg;
        for (const auto& ip : image_paths) {
            cv::Mat img = cv::imread(ip.string());
            if (!img.empty()) {
                auto det_res = dl_detector->detect(img, dcfg, "both");
                all_dl_dets[ip.stem().string()] = det_res.detections;
            }
        }
    }

    if (all_dl_dets.empty() && image_paths.empty()) {
        std::cerr << "[OutputProcessor] ERRORE: Nessuna immagine o dato trovato per "
                  << board_name << " [" << side << "]." << std::endl;
        return res;
    }

    // 3) Selezioniamo la foto in cui sono stati rilevati PIÙ componenti (consigliata)
    std::string recommended_stem;
    size_t max_dets_found = 0;
    for (const auto& [stem, dets] : all_dl_dets) {
        if (dets.size() >= max_dets_found) {
            max_dets_found = dets.size();
            recommended_stem = stem;
        }
    }

    if (recommended_stem.empty() && !image_paths.empty()) {
        recommended_stem = image_paths[0].stem().string();
    }

    std::cout << "\n-> Foto consigliata (piu' componenti rilevati): " << recommended_stem
              << " (" << max_dets_found << " componenti DL)" << std::endl;

    std::string chosen_stem = recommended_stem;

    // REQUISITO UTENTE:
    // "come immagine base per l'output finale chiedi all'utente quale usare tra quelle disponibili di quel lato
    // (a livello di riconoscimento non cambia, cambia solo l'aspetto grafico)"
    if (!config.preferred_base_image.empty()) {
        chosen_stem = config.preferred_base_image;
        std::cout << "-> Utilizzo immagine base predefinita da configurazione: " << chosen_stem << std::endl;
    } else if (config.interactive_image_selection && image_paths.size() > 1) {
        std::cout << "\n========================================================" << std::endl;
        std::cout << " SCELTA IMMAGINE BASE PER IL RENDERING GRAFICO FINALE   " << std::endl;
        std::cout << " Scheda: " << board_name << " | Lato: " << side << std::endl;
        std::cout << "========================================================" << std::endl;
        std::cout << "Tutte le foto sono state scattate dalla medesima posizione del robot." << std::endl;
        std::cout << "A livello di riconoscimento non cambia, cambia solo l'aspetto grafico finale." << std::endl;
        std::cout << "Foto disponibili per questo lato:" << std::endl;

        for (size_t i = 0; i < image_paths.size(); ++i) {
            std::string st = image_paths[i].stem().string();
            size_t n_dets = (all_dl_dets.find(st) != all_dl_dets.end()) ? all_dl_dets[st].size() : 0;
            std::cout << "  " << (i + 1) << ") " << image_paths[i].filename().string()
                      << " (" << n_dets << " rilevamenti DL)";
            if (st == recommended_stem) {
                std::cout << "  <-- [CONSIGLIATA - PIÙ COMPONENTI]";
            }
            std::cout << std::endl;
        }

        std::cout << "Quale foto vuoi utilizzare come sfondo grafico? (Inserisci 1-"
                  << image_paths.size() << " oppure premi INVIO per la consigliata [" << recommended_stem << "]): ";

        std::string user_choice;
        if (std::getline(std::cin, user_choice)) {
            user_choice.erase(0, user_choice.find_first_not_of(" \t\r\n"));
            user_choice.erase(user_choice.find_last_not_of(" \t\r\n") + 1);
            if (!user_choice.empty()) {
                try {
                    int c_idx = std::stoi(user_choice);
                    if (c_idx >= 1 && c_idx <= static_cast<int>(image_paths.size())) {
                        chosen_stem = image_paths[c_idx - 1].stem().string();
                    }
                } catch (...) {
                    for (const auto& ip : image_paths) {
                        if (ip.stem().string() == user_choice || ip.filename().string() == user_choice) {
                            chosen_stem = ip.stem().string();
                            break;
                        }
                    }
                }
            }
        }
        std::cout << "-> Immagine base selezionata per il rendering: " << chosen_stem << std::endl;
    }

    res.best_image_stem = chosen_stem;

    // Carichiamo l'immagine base selezionata
    for (const auto& ip : image_paths) {
        if (ip.stem().string() == chosen_stem) {
            res.best_image_path = ip;
            res.best_image = cv::imread(ip.string());
            break;
        }
    }

    if (res.best_image.empty() && !image_paths.empty()) {
        res.best_image_path = image_paths[0];
        res.best_image = cv::imread(image_paths[0].string());
    }

    // -------------------------------------------------------------------------
    // ESECUZIONE STEP 1: Fusione multi-esposizione DL
    // -------------------------------------------------------------------------
    if (!all_dl_dets.empty() || config.detection_mode == 2 || config.detection_mode == 3) {
        std::cout << "\n--- STEP 1: Fusione rilevamenti DL multi-esposizione ---" << std::endl;
        res.step1_dl_merged = mergeMultiExposureDl(all_dl_dets, recommended_stem, config.dl_merge_iou_thresh);
        for (const auto& c : res.step1_dl_merged) {
            res.step1_counts[c.label]++;
        }

        std::cout << "   Componenti DL unificati totali: " << res.step1_dl_merged.size() << std::endl;
        std::cout << "   Dettaglio classi (priorita' no-? e voti di maggioranza):" << std::endl;
        for (const auto& [lbl, cnt] : res.step1_counts) {
            std::cout << "     - " << lbl << ": " << cnt << std::endl;
        }

        if (!res.best_image.empty()) {
            res.step1_elements_vis = renderStep1Elements(res.best_image, res.step1_dl_merged);
            res.step1_types_vis = renderStep1Types(res.best_image, res.step1_dl_merged);
        }
    }

    // -------------------------------------------------------------------------
    // ESECUZIONE STEP 2: Riconoscimento componenti CV tramite pesi DL (se mode 1 o 3)
    // -------------------------------------------------------------------------
    if (config.detection_mode == 1 || config.detection_mode == 3) {
        std::cout << "\n--- STEP 2: Riconoscimento componenti CV con pesi Deep Learning ---" << std::endl;
        std::vector<CvComponent> cv_comps;

        // Tentiamo di caricare i componenti CV della foto migliore o della prima disponibile
        std::string cv_csv_path = (fs::path(cv_output_dir) / (recommended_stem + "_components_cv.csv")).string();
        if (fs::exists(cv_csv_path)) {
            cv_comps = loadCvComponentsCsv(cv_csv_path);
            std::cout << "   Caricati " << cv_comps.size() << " componenti CV da " << cv_csv_path << std::endl;
        } else if (cv_detector != nullptr && !res.best_image.empty()) {
            std::cout << "   Esecuzione CV Component Detector al volo sull'immagine di riferimento..." << std::endl;
            CvDetectionConfig cv_cfg;
            cv_cfg.show_windows = false;
            auto cv_res = cv_detector->detect(res.best_image, cv_cfg);
            cv_comps = cv_res.components;
        } else {
            // Cerca qualsiasi file CV disponibile nella cartella
            if (fs::exists(cv_output_dir)) {
                for (const auto& entry : fs::directory_iterator(cv_output_dir)) {
                    if (entry.path().extension() == ".csv") {
                        cv_comps = loadCvComponentsCsv(entry.path().string());
                        if (!cv_comps.empty()) {
                            std::cout << "   Caricati " << cv_comps.size() << " componenti CV da "
                                      << entry.path().filename().string() << std::endl;
                            break;
                        }
                    }
                }
            }
        }

        res.step2_cv_classified = classifyCvComponentsWithDl(
            cv_comps,
            res.step1_dl_merged,
            res.best_image,
            dl_detector,
            config.cv_dl_match_iou_thresh,
            config.min_conf_for_label,
            config.enable_patch_inference);

        for (const auto& c : res.step2_cv_classified) {
            res.step2_counts[c.label]++;
        }

        std::cout << "   Componenti CV classificati totali: " << res.step2_cv_classified.size() << std::endl;
        std::cout << "   Dettaglio classi CV riconosciute con pesi DL:" << std::endl;
        for (const auto& [lbl, cnt] : res.step2_counts) {
            std::cout << "     - " << lbl << ": " << cnt << std::endl;
        }

        if (!res.best_image.empty()) {
            res.step2_cv_elements_vis = renderStep2CvElements(res.best_image, res.step2_cv_classified);
            res.step2_cv_types_vis = renderStep2CvTypes(res.best_image, res.step2_cv_classified);
        }
    }

    // -------------------------------------------------------------------------
    // ESECUZIONE STEP 3: Elaborazione Componenti Finali per Modalità
    // -------------------------------------------------------------------------
    std::cout << "\n--- STEP 3: Elaborazione Componenti Finali ---" << std::endl;
    if (config.detection_mode == 1) {
        std::cout << "   [Modalità 1: Solo Computer Vision Classica catalogata]" << std::endl;
        for (const auto& cv_ref : res.step2_cv_classified) {
            // REQUISITO UTENTE: i componenti non catalogati in CV non considerarli nella fusione
            if (cv_ref.label == "?" || cv_ref.label.empty()) continue;

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
            fc.notes = "Solo CV (Metodo: " + cv_ref.classification_method + ")";
            res.step3_final_components.push_back(fc);
        }
        res.step3_final_components = resolveEnclosingAndOverlaps(res.step3_final_components, config.enclosure_overlap_thresh);
    } else if (config.detection_mode == 2) {
        std::cout << "   [Modalità 2: Solo Deep Learning YOLO]" << std::endl;
        for (const auto& dl : res.step1_dl_merged) {
            FinalComponent fc;
            fc.source = ComponentSource::DL_ONLY;
            fc.shape = ShapeType::RECTANGLE;
            fc.box = dl.box;
            fc.center = cv::Point2f(dl.box.x + dl.box.width * 0.5f, dl.box.y + dl.box.height * 0.5f);
            fc.rotated_box = cv::RotatedRect(fc.center, dl.box.size(), 0.0f);
            fc.radius = 0.0f;
            fc.area = dl.box.width * dl.box.height;
            fc.label = dl.label;
            fc.confidence = (dl.label != "?") ? dl.label_conf : dl.box_conf;
            fc.notes = "Solo DL (" + std::to_string(dl.vote_count) + " voti su " +
                       std::to_string(dl.total_observations) + " foto)";
            res.step3_final_components.push_back(fc);
        }
        res.step3_final_components = resolveEnclosingAndOverlaps(res.step3_final_components, config.enclosure_overlap_thresh);
    } else {
        std::cout << "   [Modalità 3: Entrambi DL + CV Classica]" << std::endl;
        res.step3_final_components = fuseDlAndCv(
            res.step1_dl_merged,
            res.step2_cv_classified,
            config.cv_dl_match_iou_thresh,
            config.tiny_cv_max_area,
            config.tiny_cv_max_dim);
    }

    // -------------------------------------------------------------------------
    // ARMONIZZAZIONE E RICERCA GLOBALE DI COMPONENTI SIMILI (TUTTE LE 3 MODALITÀ)
    // -------------------------------------------------------------------------
    if (config.enable_similarity_harmonization && !res.best_image.empty()) {
        std::cout << "\n--- FASE 1: Armonizzazione Componenti Identici Riconosciuti ---" << std::endl;
        res.step3_final_components = harmonizeIdenticalComponents(
            res.step3_final_components,
            res.best_image,
            config.similarity_geom_tol,
            config.similarity_ncc_thresh,
            config.similarity_hist_thresh,
            &res.similarity_clusters);

        for (const auto& cinfo : res.similarity_clusters) {
            res.harmonized_components_count += cinfo.harmonized_count;
        }

        std::cout << "   Cluster di componenti identici identificati: " << res.similarity_clusters.size() << std::endl;
        std::cout << "   Componenti ri-classificati/uniformati: " << res.harmonized_components_count << std::endl;
        for (const auto& cinfo : res.similarity_clusters) {
            if (cinfo.harmonized_count > 0) {
                std::cout << "     * Cluster #" << cinfo.cluster_id << " (" << cinfo.member_count << " componenti identici): "
                          << "Classe armonizzata: '" << cinfo.winning_label << "' (Conf: "
                          << std::fixed << std::setprecision(2) << cinfo.consensus_confidence << "). "
                          << cinfo.harmonized_count << " corretti/promossi da: ";
                for (const auto& [lbl, cnt] : cinfo.original_labels) {
                    if (lbl != cinfo.winning_label) {
                        std::cout << "[" << lbl << ": " << cnt << "] ";
                    }
                }
                std::cout << std::endl;
            }
        }

        // FASE 2: Ricerca visiva globale su tutta la scheda per scoprire nuovi componenti simili
        if (config.enable_whole_board_search && !res.similarity_clusters.empty()) {
            std::cout << "\n--- FASE 2: Ricerca Globale su tutta la Scheda di Porzioni Simili ---" << std::endl;
            auto newly_discovered = searchSimilarComponentsOnBoard(
                res.step3_final_components,
                res.best_image,
                res.similarity_clusters,
                config.search_ncc_thresh,
                config.search_hist_thresh,
                config.search_max_existing_iou);

            res.discovered_components_count = static_cast<int>(newly_discovered.size());
            std::cout << "   Nuovi componenti scoperti su scheda: " << res.discovered_components_count << std::endl;
            for (const auto& nd : newly_discovered) {
                std::cout << "     + Trovato nuovo componente: " << nd.label
                          << " ad alta similarita' a (" << std::round(nd.center.x) << ", " << std::round(nd.center.y)
                          << ") [Conf: " << std::fixed << std::setprecision(2) << nd.confidence << "]" << std::endl;
                res.step3_final_components.push_back(nd);
            }

            // Ri-ordina e ri-assegna ID progressivi
            std::sort(res.step3_final_components.begin(), res.step3_final_components.end(), [](const auto& a, const auto& b) {
                if (std::abs(a.center.y - b.center.y) > 20.0f) {
                    return a.center.y < b.center.y;
                }
                return a.center.x < b.center.x;
            });
            for (size_t i = 0; i < res.step3_final_components.size(); ++i) {
                res.step3_final_components[i].id = static_cast<int>(i + 1);
            }
        }
    }

    for (const auto& c : res.step3_final_components) {
        res.step3_final_counts[c.label]++;
        res.source_counts[sourceToString(c.source)]++;
    }

    std::cout << "   Totale componenti finali unificati: " << res.step3_final_components.size() << std::endl;
    std::cout << "   Ripartizione per fonte:" << std::endl;
    for (const auto& [src, cnt] : res.source_counts) {
        std::cout << "     - " << src << ": " << cnt << std::endl;
    }
    std::cout << "   Riepilogo finale classi:" << std::endl;
    for (const auto& [lbl, cnt] : res.step3_final_counts) {
        std::cout << "     - " << lbl << ": " << cnt << std::endl;
    }

    if (!res.best_image.empty()) {
        res.step3_final_elements_vis = renderStep3FinalElements(res.best_image, res.step3_final_components);
        res.step3_final_types_vis = renderStep3FinalTypes(res.best_image, res.step3_final_components);
        res.step3_final_geometric_vis = renderStep3Geometric(res.best_image, res.step3_final_components);
    }

    // -------------------------------------------------------------------------
    // Salvataggio di tutti i passaggi intermedi
    // -------------------------------------------------------------------------
    if (config.save_intermediate_images) {
        std::string target_out_dir = (fs::path(config.output_base_dir) / board_name / side).string();
        saveAllStepResults(res, target_out_dir);
    }

    return res;
}

} // namespace pcb
