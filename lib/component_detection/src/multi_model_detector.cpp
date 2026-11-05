#include "multi_model_detector.hpp"
#include "common_utils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <set>

namespace fs = std::filesystem;

namespace pcb {

MultiModelDetector::MultiModelDetector(
    const std::string& base_models_dir,
    const std::string& mapping_file_path)
    : base_models_dir_(base_models_dir),
      class_mapper_(mapping_file_path) {
}

void MultiModelDetector::addSearchDirectory(const std::string& dir) {
    if (!dir.empty() && fs::exists(dir)) {
        additional_search_dirs_.push_back(dir);
    }
}

bool MultiModelDetector::addExplicitWeightFile(const std::string& weights_path, const std::string& custom_model_name) {
    if (!fs::exists(weights_path)) {
        std::cerr << "[MultiModelDetector] File pesi esplicito non trovato: " << weights_path << std::endl;
        return false;
    }

    std::error_code ec;
    auto sz = fs::file_size(weights_path, ec);
    WeightFileInfo info;
    info.full_path = fs::absolute(weights_path).string();
    info.relative_path = fs::path(weights_path).filename().string();
    info.size_bytes = (ec) ? 0 : sz;
    info.model_name = custom_model_name.empty() ? fs::path(weights_path).stem().string() : custom_model_name;
    info.is_loadable = (info.size_bytes > 0);
    info.status_note = info.is_loadable ? "File esplicito pronto" : "File esplicito vuoto (0 byte)";
    discovered_files_.push_back(info);

    return info.is_loadable;
}

std::string MultiModelDetector::resolveBaseModelsDir() const {
    std::vector<std::string> candidates = {
        base_models_dir_,
        "/home/alexpauro/Tesi/PcbDetectionAllignmentLeonardo/pcb_component_detection",
        "../pcb_component_detection",
        "pcb_component_detection",
        "../../pcb_component_detection"
    };

    // Tentativo di risoluzione tramite percorso dell'eseguibile (/proc/self/exe su Linux)
    try {
        if (fs::exists("/proc/self/exe")) {
            fs::path exe_p = fs::canonical("/proc/self/exe");
            fs::path cand_a = exe_p.parent_path() / "pcb_component_detection";
            fs::path cand_b = exe_p.parent_path().parent_path() / "pcb_component_detection";
            fs::path cand_c = exe_p.parent_path().parent_path().parent_path() / "pcb_component_detection";
            candidates.push_back(cand_a.string());
            candidates.push_back(cand_b.string());
            candidates.push_back(cand_c.string());
        }
    } catch (...) {}

    for (const auto& c : candidates) {
        if (!c.empty() && fs::exists(c)) {
            std::error_code ec;
            fs::path can = fs::canonical(c, ec);
            if (!ec && fs::is_directory(can)) {
                return can.string();
            }
        }
    }

    return base_models_dir_;
}

int MultiModelDetector::scanAndLoadModels(bool interactive_mapping) {
    models_.clear();
    discovered_files_.clear();

    std::string root_dir = resolveBaseModelsDir();
    base_models_dir_ = root_dir;

    std::cout << "\n========================================================================" << std::endl;
    std::cout << " SCANSIONE AVANZATA PESI E ALGORITMI COMPONENT DETECTION" << std::endl;
    std::cout << " Cartella di ricerca principale: " << root_dir << std::endl;
    std::cout << "========================================================================" << std::endl;

    if (!fs::exists(root_dir)) {
        std::cerr << "[MultiModelDetector] ERRORE: Cartella modelli non trovata: " << root_dir << std::endl;
        return 0;
    }

    // Elenco directory da perlustrare (root principale + cartelle addizionali)
    std::vector<std::string> search_roots = { root_dir };
    for (const auto& ad : additional_search_dirs_) {
        if (fs::exists(ad) && std::find(search_roots.begin(), search_roots.end(), ad) == search_roots.end()) {
            search_roots.push_back(ad);
        }
    }

    // -------------------------------------------------------------------------
    // FASE 1: INVENTARIO COMPLETO DI TUTTI I FILE PESI SUL DISCO
    // -------------------------------------------------------------------------
    std::vector<std::string> target_exts = {".pt", ".onnx", ".engine", ".weights", ".bin"};

    for (const auto& sr : search_roots) {
        for (const auto& entry : fs::recursive_directory_iterator(sr, fs::directory_options::follow_directory_symlink)) {
            if (!entry.is_regular_file()) continue;

            std::string p_str = entry.path().string();
            // Salta solo cartelle interne di sistema o file all'interno della cartella .git/
            if (p_str.find("/.venv/") != std::string::npos ||
                p_str.find("/__pycache__/") != std::string::npos ||
                p_str.find("/.git/") != std::string::npos) {
                continue;
            }

            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

            if (std::find(target_exts.begin(), target_exts.end(), ext) != target_exts.end()) {
                std::string fname = entry.path().filename().string();
                if (fname.find("test_interpreter") != std::string::npos) continue;

                std::error_code ec;
                auto sz = fs::file_size(entry.path(), ec);
                uintmax_t file_bytes = (ec) ? 0 : sz;

                WeightFileInfo item;
                item.full_path = fs::absolute(entry.path()).string();

                // Calcolo percorso relativo leggibile
                try {
                    item.relative_path = fs::relative(entry.path(), sr).string();
                } catch (...) {
                    item.relative_path = entry.path().filename().string();
                }

                item.size_bytes = file_bytes;
                item.is_loadable = (file_bytes > 0);

                // Deduzione nome modello
                std::string parent_name = entry.path().parent_path().filename().string();
                if (parent_name == "weights" || parent_name == "model" || parent_name.empty()) {
                    item.model_name = entry.path().parent_path().parent_path().filename().string();
                } else {
                    item.model_name = parent_name;
                }
                if (item.model_name.empty()) {
                    item.model_name = entry.path().stem().string();
                }

                if (item.size_bytes == 0) {
                    item.status_note = "FILE VUOTO (0 byte): segnaposto Git o download incompleto";
                } else {
                    item.status_note = "Pronto per l'inferenza";
                }

                discovered_files_.push_back(item);
            }
        }
    }

    // -------------------------------------------------------------------------
    // FASE 2: REPORT COMPLETO IN CONSOLE DELL'INVENTARIO
    // -------------------------------------------------------------------------
    std::cout << "\n[Inventario Pesi Trovati]: " << discovered_files_.size() << " file identificati nel sistema:" << std::endl;
    for (size_t i = 0; i < discovered_files_.size(); ++i) {
        const auto& f = discovered_files_[i];
        double sz_mb = static_cast<double>(f.size_bytes) / (1024.0 * 1024.0);
        std::cout << " [" << i + 1 << "] " << f.relative_path << "\n";
        std::cout << "     Percorso:   " << f.full_path << "\n";
        std::cout << "     Modello:    " << f.model_name << "\n";
        if (f.size_bytes == 0) {
            std::cout << "     Stato:      [ATTENZIONE: 0 BYTE - NON CARICABILE]\n";
            std::cout << "     Dettaglio:  " << f.status_note << "\n";
            if (f.full_path.find("yolov5s_results15") != std::string::npos) {
                std::cout << "     --> NOTA: Il repository originale 'pcb-components-detection-recognition'\n";
                std::cout << "         ha caricato un file 'best.pt' vuoto a 0 byte su GitHub.\n";
                std::cout << "         I pesi addestrati effettivi sono disponibili nel Google Drive del progetto:\n";
                std::cout << "         https://drive.google.com/drive/folders/1odzD298ImaHI3NKozQNGIgKbflzgSWIn?usp=sharing\n";
                std::cout << "         Scaricando o inserendo un file pesi valido (>0 byte), verra' caricato automaticamente!\n";
            }
        } else {
            std::cout << "     Dimensione: " << std::fixed << std::setprecision(2) << sz_mb << " MB\n";
            std::cout << "     Stato:      [VALIDO E DISPONIBILE]\n";
        }
    }
    std::cout << "------------------------------------------------------------------------" << std::endl;

    // -------------------------------------------------------------------------
    // FASE 3: CARICAMENTO DEL MODELLO PRIMARIO PARSAVICTOR (2 STADI)
    // -------------------------------------------------------------------------
    std::string pv_any_path = "";
    std::string pv_fine_path = "";

    for (const auto& f : discovered_files_) {
        if (!f.is_loadable) continue;
        std::string fname = fs::path(f.full_path).filename().string();
        if (f.full_path.find("ParsaVictor") != std::string::npos) {
            if (fname == "any_best.onnx" || (fname == "any_best.pt" && pv_any_path.empty())) {
                pv_any_path = f.full_path;
            } else if (fname == "fine_best.onnx" || (fname == "fine_best.pt" && pv_fine_path.empty())) {
                pv_fine_path = f.full_path;
            }
        }
    }

    if (!pv_any_path.empty() && !pv_fine_path.empty()) {
        DiscoveredModelInfo pv_info;
        pv_info.name = "ParsaVictor";
        pv_info.directory = fs::path(pv_fine_path).parent_path().string();
        pv_info.weights_path = pv_fine_path;
        pv_info.is_parsa_victor = true;
        pv_info.is_two_stage = true;
        pv_info.any_weights_path = pv_any_path;
        pv_info.fine_weights_path = pv_fine_path;
        pv_info.imgsz = 1024;
        pv_info.priority_weight = 1.0f; // Massima priorità per gerarchia di progetto
        pv_info.raw_classes = ComponentDetector::getDefaultClasses();
        pv_info.mapped_classes = pv_info.raw_classes;
        pv_info.num_classes = static_cast<int>(pv_info.raw_classes.size());

        pv_info.two_stage_detector = std::make_shared<ComponentDetector>();
        bool any_ok = pv_info.two_stage_detector->loadAnyModel(pv_any_path, pv_info.imgsz);
        bool fine_ok = pv_info.two_stage_detector->loadFineModel(pv_fine_path, pv_info.raw_classes, pv_info.imgsz);

        if (any_ok && fine_ok) {
            pv_info.is_loaded = true;
            models_.push_back(pv_info);
            std::cout << "\n[OK] Modello di Riferimento 'ParsaVictor' caricato (Priorita': "
                      << pv_info.priority_weight << "x, 13 classi standard)." << std::endl;
        } else {
            std::cerr << "[ERRORE] Inizializzazione ParsaVictor fallita." << std::endl;
        }
    }

    // -------------------------------------------------------------------------
    // FASE 4: CARICAMENTO DEI MODELLI SECONDARI VALIDI (> 0 BYTE)
    // -------------------------------------------------------------------------
    std::set<std::string> parsa_paths;
    if (!pv_any_path.empty()) parsa_paths.insert(pv_any_path);
    if (!pv_fine_path.empty()) parsa_paths.insert(pv_fine_path);

    // Evita duplicati di pesi (.onnx e .pt della stessa rete)
    std::map<std::string, const WeightFileInfo*> valid_weights_map;
    for (const auto& f : discovered_files_) {
        if (!f.is_loadable) continue;
        if (parsa_paths.find(f.full_path) != parsa_paths.end()) continue;

        fs::path p(f.full_path);
        std::string key = (p.parent_path() / p.stem()).string();

        if (p.extension() == ".onnx") {
            valid_weights_map[key] = &f; // Priorità a .onnx
        } else if (valid_weights_map.find(key) == valid_weights_map.end()) {
            valid_weights_map[key] = &f;
        }
    }

    for (const auto& [k, f_ptr] : valid_weights_map) {
        const auto& f = *f_ptr;
        DiscoveredModelInfo sec_info;
        sec_info.name = f.model_name;
        sec_info.directory = fs::path(f.full_path).parent_path().string();
        sec_info.is_parsa_victor = false;
        sec_info.is_two_stage = false;
        sec_info.imgsz = 1024;
        sec_info.priority_weight = 1.0f; // Priorità secondaria rispetto a ParsaVictor

        std::cout << "\n-> Inizializzazione modello secondario: " << sec_info.name << std::endl;
        std::cout << "   File pesi: " << f.full_path << std::endl;

        std::string onnx_path = ComponentDetector::ensureOnnxModel(f.full_path, sec_info.imgsz);
        if (onnx_path.empty() || !fs::exists(onnx_path)) {
            std::cerr << "   [ERRORE] Impossibile convertire o caricare " << f.full_path << std::endl;
            continue;
        }
        sec_info.weights_path = onnx_path;

        // Estrazione delle classi del modello
        std::vector<std::string> raw_classes;
        fs::path wp(f.full_path);
        std::vector<fs::path> candidate_cls_files = {
            wp.parent_path() / (wp.stem().string() + "_classes.txt"),
            wp.parent_path() / "best_classes.txt",
            wp.parent_path() / "classes.txt",
            wp.parent_path().parent_path() / "classes.txt",
            wp.parent_path().parent_path() / "best_classes.txt"
        };

        for (const auto& cf : candidate_cls_files) {
            if (fs::exists(cf)) {
                std::ifstream cfile(cf);
                std::string line;
                while (std::getline(cfile, line)) {
                    size_t s = line.find_first_not_of(" \t\r\n");
                    size_t e = line.find_last_not_of(" \t\r\n");
                    if (s != std::string::npos) {
                        std::string cl = line.substr(s, e - s + 1);
                        if (!cl.empty() && cl[0] != '#') raw_classes.push_back(cl);
                    }
                }
                if (!raw_classes.empty()) break;
            }
        }

        // Se non trovato su file, tenta tramite script Python con ONNX metadata
        if (raw_classes.empty()) {
            std::vector<std::string> py_bins = {
                "pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/.venv/bin/python3",
                "../pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/.venv/bin/python3",
                "/home/alexpauro/Tesi/PcbDetectionAllignmentLeonardo/pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/.venv/bin/python3",
                "python3", "python"
            };
            std::string py = "python3";
            for (const auto& b : py_bins) { if (fs::exists(b)) { py = b; break; } }

            std::vector<std::string> exp_scripts = {
                "pcb_component_detection/export_onnx.py",
                "../pcb_component_detection/export_onnx.py",
                "/home/alexpauro/Tesi/PcbDetectionAllignmentLeonardo/pcb_component_detection/export_onnx.py"
            };
            std::string sc = "";
            for (const auto& s : exp_scripts) { if (fs::exists(s)) { sc = s; break; } }

            if (!sc.empty()) {
                std::string cmd = py + " " + sc + " --weights \"" + onnx_path + "\" --save-classes > /dev/null 2>&1";
                int sys_ret = std::system(cmd.c_str());
                (void)sys_ret;

                fs::path gen_cls = fs::path(onnx_path).parent_path() / (fs::path(onnx_path).stem().string() + "_classes.txt");
                if (fs::exists(gen_cls)) {
                    std::ifstream cfile(gen_cls);
                    std::string line;
                    while (std::getline(cfile, line)) {
                        size_t s = line.find_first_not_of(" \t\r\n");
                        size_t e = line.find_last_not_of(" \t\r\n");
                        if (s != std::string::npos) {
                            std::string cl = line.substr(s, e - s + 1);
                            if (!cl.empty() && cl[0] != '#') raw_classes.push_back(cl);
                        }
                    }
                }
            }
        }

        // Caricamento rete in OpenCV DNN
        try {
            sec_info.single_stage_net = cv::dnn::readNetFromONNX(onnx_path);
            sec_info.single_stage_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            sec_info.single_stage_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        } catch (const cv::Exception& e) {
            std::cerr << "   [ERRORE] Caricamento OpenCV DNN fallito per " << onnx_path << ": " << e.what() << std::endl;
            continue;
        }

        // Se le classi non erano note, determina il numero dalle uscite del modello
        if (raw_classes.empty()) {
            cv::Mat dummy = cv::Mat::zeros(cv::Size(sec_info.imgsz, sec_info.imgsz), CV_8UC3);
            cv::Mat blob;
            cv::dnn::blobFromImage(dummy, blob, 1.0 / 255.0, cv::Size(sec_info.imgsz, sec_info.imgsz), cv::Scalar(), true, false);
            sec_info.single_stage_net.setInput(blob);
            std::vector<cv::Mat> outs;
            sec_info.single_stage_net.forward(outs, sec_info.single_stage_net.getUnconnectedOutLayersNames());
            if (!outs.empty()) {
                int channels = (outs[0].dims == 3) ? outs[0].size[1] : outs[0].rows;
                if (channels > 4 && channels < 500) {
                    int c_count = channels - 4;
                    for (int i = 0; i < c_count; ++i) {
                        raw_classes.push_back("Class_" + std::to_string(i));
                    }
                }
            }
        }

        sec_info.raw_classes = raw_classes;
        sec_info.num_classes = static_cast<int>(raw_classes.size());

        // Unificazione delle classi verso ParsaVictor
        std::cout << "   Unificazione classi del modello (" << raw_classes.size() << " classi):" << std::endl;
        for (const auto& raw_c : raw_classes) {
            std::string mapped_c = class_mapper_.getOrMapClass(sec_info.name, raw_c, interactive_mapping);
            sec_info.mapped_classes.push_back(mapped_c);
            std::cout << "     - '" << raw_c << "' -> '" << mapped_c << "'" << std::endl;
        }

        sec_info.is_loaded = true;
        models_.push_back(sec_info);
        std::cout << "   [OK] Modello '" << sec_info.name << "' pronto per l'inferenza." << std::endl;
    }

    std::cout << "\n========================================================================" << std::endl;
    std::cout << " RIEPILOGO: " << models_.size() << " algoritmi/pesi DL attivi e pronti per l'ensemble." << std::endl;
    std::cout << "========================================================================\n" << std::endl;

    return static_cast<int>(models_.size());
}

void MultiModelDetector::inferSingleStageTile(
    cv::dnn::Net& net,
    const cv::Mat& tile,
    int offset_x,
    int offset_y,
    float conf_threshold,
    int imgsz,
    int num_classes,
    const std::vector<std::string>& mapped_classes,
    std::vector<cv::Rect2f>& out_boxes,
    std::vector<float>& out_scores,
    std::vector<int>& out_classes) {

    cv::Mat blob;
    cv::dnn::blobFromImage(tile, blob, 1.0 / 255.0, cv::Size(imgsz, imgsz), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return;

    cv::Mat out = outputs[0];
    int rows = (out.dims == 3) ? out.size[1] : out.rows;
    int cols = (out.dims == 3) ? out.size[2] : out.cols;

    cv::Mat data_mat(rows, cols, CV_32F, (float*)out.data);
    cv::Mat predictions;
    if (rows < cols && rows <= 5 + num_classes) {
        cv::transpose(data_mat, predictions);
    } else {
        predictions = data_mat;
    }

    int num_candidates = predictions.rows;
    int feature_len = predictions.cols;
    if (feature_len < 4) return;

    float scale_x = static_cast<float>(tile.cols) / static_cast<float>(imgsz);
    float scale_y = static_cast<float>(tile.rows) / static_cast<float>(imgsz);

    bool is_yolov5_format = (feature_len >= 5 + num_classes);

    for (int i = 0; i < num_candidates; ++i) {
        const float* row_ptr = predictions.ptr<float>(i);
        float cx = row_ptr[0];
        float cy = row_ptr[1];
        float w = row_ptr[2];
        float h = row_ptr[3];

        float best_score = 0.0f;
        int best_cls = -1;

        if (is_yolov5_format) {
            float obj_conf = row_ptr[4];
            if (num_classes == 1) {
                best_score = obj_conf;
                best_cls = 0;
            } else {
                for (int c = 0; c < num_classes; ++c) {
                    float s = obj_conf * row_ptr[5 + c];
                    if (s > best_score) {
                        best_score = s;
                        best_cls = c;
                    }
                }
            }
        } else { // YOLOv8 / YOLO11 / YOLO26
            if (num_classes == 1 && feature_len == 5) {
                best_score = row_ptr[4];
                best_cls = 0;
            } else {
                for (int c = 0; c < num_classes && (4 + c) < feature_len; ++c) {
                    float s = row_ptr[4 + c];
                    if (s > best_score) {
                        best_score = s;
                        best_cls = c;
                    }
                }
            }
        }

        if (best_score >= conf_threshold && best_cls >= 0) {
            if (best_cls < static_cast<int>(mapped_classes.size()) && mapped_classes[best_cls] == "?") {
                continue;
            }

            float x0 = (cx - 0.5f * w) * scale_x + offset_x;
            float y0 = (cy - 0.5f * h) * scale_y + offset_y;
            float box_w = w * scale_x;
            float box_h = h * scale_y;

            out_boxes.emplace_back(x0, y0, box_w, box_h);
            out_scores.push_back(best_score);
            out_classes.push_back(best_cls);
        }
    }
}

std::vector<ComponentDetection> MultiModelDetector::runSingleStageModel(
    DiscoveredModelInfo& model,
    const cv::Mat& img,
    const DetectionConfig& config) {

    int h = img.rows;
    int w = img.cols;

    std::vector<cv::Rect2f> all_boxes;
    std::vector<float> all_scores;
    std::vector<int> all_classes;

    if (config.tile_size <= 0 || std::max(h, w) <= config.tile_size) {
        inferSingleStageTile(model.single_stage_net, img, 0, 0, config.conf_threshold,
                             model.imgsz, model.num_classes, model.mapped_classes,
                             all_boxes, all_scores, all_classes);
    } else {
        int step = std::max(1, static_cast<int>(config.tile_size * (1.0f - config.overlap)));
        std::vector<int> xs, ys;
        for (int x = 0; x <= std::max(0, w - config.tile_size); x += step) xs.push_back(x);
        for (int y = 0; y <= std::max(0, h - config.tile_size); y += step) ys.push_back(y);

        if (xs.empty() || xs.back() + config.tile_size < w) xs.push_back(std::max(0, w - config.tile_size));
        if (ys.empty() || ys.back() + config.tile_size < h) ys.push_back(std::max(0, h - config.tile_size));

        for (int y : ys) {
            for (int x : xs) {
                int cur_tw = std::min(config.tile_size, w - x);
                int cur_th = std::min(config.tile_size, h - y);
                cv::Rect crop_roi(x, y, cur_tw, cur_th);
                cv::Mat crop = img(crop_roi);

                inferSingleStageTile(model.single_stage_net, crop, x, y, config.conf_threshold,
                                     model.imgsz, model.num_classes, model.mapped_classes,
                                     all_boxes, all_scores, all_classes);
            }
        }
    }

    if (all_boxes.empty()) return {};

    std::vector<cv::Rect> int_boxes;
    int_boxes.reserve(all_boxes.size());
    for (const auto& b : all_boxes) {
        int_boxes.emplace_back(static_cast<int>(std::round(b.x)),
                               static_cast<int>(std::round(b.y)),
                               static_cast<int>(std::round(b.width)),
                               static_cast<int>(std::round(b.height)));
    }

    std::vector<int> keep_indices;
    cv::dnn::NMSBoxes(int_boxes, all_scores, config.conf_threshold, config.nms_threshold, keep_indices, 1.0f, config.max_det);

    std::vector<ComponentDetection> results;
    results.reserve(keep_indices.size());
    int id = 1;

    for (int idx : keep_indices) {
        ComponentDetection det;
        det.id = id++;
        det.box = all_boxes[idx];
        det.box_conf = all_scores[idx];
        det.label_conf = all_scores[idx];
        det.class_id = all_classes[idx];

        if (det.class_id >= 0 && det.class_id < static_cast<int>(model.mapped_classes.size())) {
            det.label = model.mapped_classes[det.class_id];
        } else {
            det.label = "?";
        }

        results.push_back(det);
    }

    return results;
}

std::vector<ComponentDetection> MultiModelDetector::fuseModelDetections(
    const std::vector<std::pair<DiscoveredModelInfo*, std::vector<ComponentDetection>>>& all_dets,
    float match_iou_thresh) {

    if (all_dets.empty()) return {};
    if (all_dets.size() == 1) return all_dets[0].second;

    struct ModelObservation {
        DiscoveredModelInfo* model;
        ComponentDetection det;
        float reliability;
    };

    struct MultiModelCluster {
        cv::Rect2f anchor_box;
        bool has_parsa_victor = false;
        cv::Rect2f parsa_victor_box;
        std::vector<ModelObservation> obs_list;
    };

    std::vector<MultiModelCluster> clusters;

    // Ordina i modelli in modo che ParsaVictor venga elaborato per primo
    std::vector<std::pair<DiscoveredModelInfo*, std::vector<ComponentDetection>>> ordered_models = all_dets;
    std::sort(ordered_models.begin(), ordered_models.end(), [](const auto& a, const auto& b) {
        if (a.first->is_parsa_victor) return true;
        if (b.first->is_parsa_victor) return false;
        return a.first->priority_weight > b.first->priority_weight;
    });

    for (auto& [model, dets] : ordered_models) {
        for (const auto& det : dets) {
            float rel = det.label_conf * model->priority_weight;
            if (det.label == "?") {
                rel = 0.05f * model->priority_weight;
            }

            float best_iou = 0.0f;
            int best_cluster_idx = -1;

            for (size_t c = 0; c < clusters.size(); ++c) {
                float iou = pcb::common::calculateIoU(det.box, clusters[c].anchor_box);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_cluster_idx = static_cast<int>(c);
                }
            }

            bool matched = false;
            if (best_iou >= match_iou_thresh && best_cluster_idx >= 0) {
                matched = true;
            } else {
                for (size_t c = 0; c < clusters.size(); ++c) {
                    const auto& ab = clusters[c].anchor_box;
                    cv::Point2f det_c(det.box.x + det.box.width * 0.5f, det.box.y + det.box.height * 0.5f);
                    cv::Point2f cl_c(ab.x + ab.width * 0.5f, ab.y + ab.height * 0.5f);

                    if ((ab.contains(det_c) || det.box.contains(cl_c)) &&
                        pcb::common::calculateOverlap(det.box, ab) >= 0.50f) {
                        best_cluster_idx = static_cast<int>(c);
                        matched = true;
                        break;
                    }
                }
            }

            if (matched && best_cluster_idx >= 0) {
                clusters[best_cluster_idx].obs_list.push_back({model, det, rel});
                if (model->is_parsa_victor) {
                    clusters[best_cluster_idx].has_parsa_victor = true;
                    clusters[best_cluster_idx].parsa_victor_box = det.box;
                }
            } else {
                MultiModelCluster new_cl;
                new_cl.anchor_box = det.box;
                new_cl.has_parsa_victor = model->is_parsa_victor;
                if (model->is_parsa_victor) new_cl.parsa_victor_box = det.box;
                new_cl.obs_list.push_back({model, det, rel});
                clusters.push_back(new_cl);
            }
        }
    }

    std::vector<ComponentDetection> final_dets;
    final_dets.reserve(clusters.size());
    int progressive_id = 1;

    for (const auto& cl : clusters) {
        if (cl.obs_list.empty()) continue;

        ComponentDetection final_d;
        final_d.id = progressive_id++;

        float max_reliability = -1.0f;
        const ModelObservation* winning_obs = nullptr;

        for (const auto& obs : cl.obs_list) {
            if (obs.reliability > max_reliability) {
                max_reliability = obs.reliability;
                winning_obs = &obs;
            }
        }

        if (winning_obs) {
            final_d.label = winning_obs->det.label;
            final_d.label_conf = winning_obs->det.label_conf;
            final_d.class_id = winning_obs->det.class_id;
            final_d.box_conf = winning_obs->det.box_conf;
        }

        if (cl.has_parsa_victor) {
            final_d.box = cl.parsa_victor_box;
        } else if (winning_obs) {
            final_d.box = winning_obs->det.box;
        } else {
            final_d.box = cl.anchor_box;
        }

        final_dets.push_back(final_d);
    }

    return final_dets;
}

DetectionResult MultiModelDetector::detect(const cv::Mat& image, const DetectionConfig& config) {
    DetectionResult result;
    if (image.empty()) {
        std::cerr << "[MultiModelDetector] Immagine vuota passata a detect()." << std::endl;
        return result;
    }

    if (models_.empty()) {
        scanAndLoadModels(true);
    }

    if (models_.empty()) {
        std::cerr << "[MultiModelDetector] Nessun modello DL disponibile per l'inferenza." << std::endl;
        return result;
    }

    std::vector<std::pair<DiscoveredModelInfo*, std::vector<ComponentDetection>>> all_model_dets;

    for (auto& model : models_) {
        if (!model.is_loaded) continue;

        std::vector<ComponentDetection> dets;
        if (model.is_two_stage && model.two_stage_detector) {
            auto sub_res = model.two_stage_detector->detect(image, config, "both");
            dets = sub_res.detections;
        } else {
            dets = runSingleStageModel(model, image, config);
        }

        std::cout << "   - Modello [" << model.name << "]: trovati " << dets.size() << " componenti." << std::endl;
        all_model_dets.push_back({&model, dets});
    }

    result.detections = fuseModelDetections(all_model_dets, config.match_iou);

    for (const auto& det : result.detections) {
        result.counts[det.label]++;
    }

    result.elements_vis = ComponentDetector::drawElements(image, result.detections);
    auto [vis, _] = ComponentDetector::drawTypes(image, result.detections, config.label_display_mode, config.min_label_px);
    result.types_vis = vis;

    return result;
}

bool MultiModelDetector::saveResults(const std::string& output_dir,
                                     const std::string& file_stem,
                                     const cv::Mat& orig_img,
                                     const DetectionResult& result) {
    return ComponentDetector::saveResults(output_dir, file_stem, orig_img, result, "both");
}

} // namespace pcb
