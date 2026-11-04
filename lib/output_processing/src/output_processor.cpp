#include "output_processor.hpp"
#include "output_internal.hpp"
#include "common_utils.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

namespace pcb {

std::string OutputProcessor::sourceToString(ComponentSource src) {
    switch (src) {
        case ComponentSource::DL_ONLY: return "DL_ONLY";
        case ComponentSource::CV_ONLY: return "CV_ONLY";
        case ComponentSource::FUSED_DL_CV: return "FUSED_DL_CV";
        case ComponentSource::SIMILARITY_DISCOVERED: return "SIMILARITY_DISCOVERED";
        default: return "UNKNOWN";
    }
}

float OutputProcessor::calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    return pcb::common::calculateIoU(a, b);
}

float OutputProcessor::calculateOverlap(const cv::Rect2f& a, const cv::Rect2f& b) {
    return pcb::common::calculateOverlap(a, b);
}

std::vector<ComponentDetection> OutputProcessor::loadDlDetectionsCsv(const std::string& csv_path) {
    std::vector<ComponentDetection> detections;
    std::ifstream file(csv_path);
    if (!file.is_open()) return detections;

    std::string line;
    if (!std::getline(file, line)) return detections; // Salta intestazione

    std::string header = toLower(line);
    bool isStandardFmt = (header.find("x0") != std::string::npos && header.find("y0") != std::string::npos);

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> row;
        while (std::getline(ss, token, ',')) {
            size_t start = token.find_first_not_of(" \t\r\n");
            size_t end = token.find_last_not_of(" \t\r\n");
            row.push_back((start == std::string::npos) ? "" : token.substr(start, end - start + 1));
        }
        if (row.size() >= 8) {
            ComponentDetection d;
            try {
                d.id = std::stoi(row[0]);
                if (isStandardFmt || row[1].find_first_not_of("0123456789.-") != std::string::npos) {
                    // Formato standard ComponentDetector: id,label,box_conf,label_conf,x0,y0,x1,y1
                    d.label = row[1];
                    d.box_conf = std::stof(row[2]);
                    d.label_conf = std::stof(row[3]);
                    float x0 = std::stof(row[4]);
                    float y0 = std::stof(row[5]);
                    float x1 = std::stof(row[6]);
                    float y1 = std::stof(row[7]);
                    d.box = cv::Rect2f(x0, y0, std::max(0.0f, x1 - x0), std::max(0.0f, y1 - y0));
                    d.class_id = (row.size() >= 9) ? std::stoi(row[8]) : -1;
                } else {
                    // Formato alternativo con coordinate prima: id, x1, y1, x2, y2, label, box_conf, label_conf
                    float x1 = std::stof(row[1]);
                    float y1 = std::stof(row[2]);
                    float x2 = std::stof(row[3]);
                    float y2 = std::stof(row[4]);
                    d.box = cv::Rect2f(x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1));
                    d.label = row[5];
                    d.box_conf = std::stof(row[6]);
                    d.label_conf = std::stof(row[7]);
                    d.class_id = (row.size() >= 9) ? std::stoi(row[8]) : -1;
                }
                detections.push_back(d);
            } catch (...) {
                continue;
            }
        }
    }
    return detections;
}

std::vector<CvComponent> OutputProcessor::loadCvComponentsCsv(const std::string& csv_path) {
    std::vector<CvComponent> components;
    std::ifstream file(csv_path);
    if (!file.is_open()) return components;

    std::string line;
    if (!std::getline(file, line)) return components; // Salta intestazione

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> row;
        while (std::getline(ss, token, ',')) {
            size_t start = token.find_first_not_of(" \t\r\n");
            size_t end = token.find_last_not_of(" \t\r\n");
            row.push_back((start == std::string::npos) ? "" : token.substr(start, end - start + 1));
        }
        if (row.size() >= 10) {
            CvComponent c;
            try {
                c.id = std::stoi(row[0]);
                std::string shape_str = toLower(row[1]);
                c.shape = (shape_str == "circle") ? ShapeType::CIRCLE : ShapeType::RECTANGLE;
                c.center.x = std::stof(row[2]);
                c.center.y = std::stof(row[3]);
                float w = std::stof(row[4]);
                float h = std::stof(row[5]);
                float angle = std::stof(row[6]);
                c.rotated_box = cv::RotatedRect(c.center, cv::Size2f(w, h), angle);
                c.radius = std::stof(row[7]);
                c.area = std::stof(row[8]);
                c.circularity = std::stof(row[9]);
                c.perimeter = (row.size() >= 11) ? std::stof(row[10]) : (2.0f * (w + h));
                c.aspect_ratio = (row.size() >= 12) ? std::stof(row[11]) :
                                 ((std::max(w, h) > 1e-4f) ? (std::min(w, h) / std::max(w, h)) : 1.0f);
                components.push_back(c);
            } catch (...) {
                continue;
            }
        }
    }
    return components;
}

std::map<std::string, std::vector<ComponentDetection>> OutputProcessor::loadAllDlDetectionsInDir(const std::string& dir) {
    std::map<std::string, std::vector<ComponentDetection>> all_dets;
    if (!fs::exists(dir)) return all_dets;

    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
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

    // Scelta immagine base per l'output finale
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
    // ESECUZIONE STEP 2: Riconoscimento componenti CV tramite pesi DL
    // -------------------------------------------------------------------------
    if (config.detection_mode == 1 || config.detection_mode == 3) {
        std::cout << "\n--- STEP 2: Riconoscimento componenti CV con pesi Deep Learning ---" << std::endl;
        std::vector<CvComponent> cv_comps;

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
    // ARMONIZZAZIONE E RICERCA GLOBALE DI COMPONENTI SIMILI
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

    if (config.save_intermediate_images) {
        std::string target_out_dir = (fs::path(config.output_base_dir) / board_name / side).string();
        saveAllStepResults(res, target_out_dir);
    }

    return res;
}

} // namespace pcb
