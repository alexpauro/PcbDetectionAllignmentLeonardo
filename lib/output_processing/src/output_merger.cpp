#include "output_internal.hpp"
#include <algorithm>

namespace pcb {

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

} // namespace pcb

