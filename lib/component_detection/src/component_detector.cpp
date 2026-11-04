#include "component_detector.hpp"
#include "common_utils.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

namespace pcb {

namespace {

// Colore verde brillante per lo Step 1: WHERE
const cv::Scalar ELEMENT_COLOUR(120, 255, 0); // BGR

// Codici brevi per le classi note
const std::map<std::string, std::string> SHORT_MAP = {
    {"Resistor", "R"},
    {"Capacitor", "C"},
    {"Electrolytic Capacitor", "EC"},
    {"IC", "IC"},
    {"Inductor", "L"},
    {"Diode", "D"},
    {"LED", "LED"},
    {"Transistor", "Q"},
    {"Oscillator", "X"},
    {"Connector", "CN"},
    {"Switch/Button", "SW"},
    {"Other", "O"},
    {"Marker", "M"},
    {"component", ""},
    {"?", "?"}
};

// Conversione HSV (0..1, 0..1, 0..1) a cv::Scalar (BGR, 0..255)
cv::Scalar hsvToBgr(float h, float s, float v) {
    float r = 0.0f, g = 0.0f, b = 0.0f;
    int i = static_cast<int>(h * 6.0f);
    float f = (h * 6.0f) - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (i % 6) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return cv::Scalar(static_cast<int>(b * 255.0f),
                      static_cast<int>(g * 255.0f),
                      static_cast<int>(r * 255.0f));
}

// Ridimensionamento per visualizzazione pulita
std::pair<cv::Mat, float> makeCanvas(const cv::Mat& img, int target) {
    int h0 = img.rows;
    int w0 = img.cols;
    int max_dim = std::max(h0, w0);
    if (max_dim <= 0) return {img.clone(), 1.0f};

    float up = static_cast<float>(target) / static_cast<float>(max_dim);
    if (max_dim < target) {
        up = std::max(up, 1.0f);
    } else {
        up = std::min(1.0f, 4000.0f / static_cast<float>(max_dim));
    }

    if (std::abs(up - 1.0f) < 1e-4f) {
        return {img.clone(), 1.0f};
    }

    cv::Mat out;
    int interp = (up > 1.0f) ? cv::INTER_CUBIC : cv::INTER_AREA;
    cv::resize(img, out, cv::Size(), up, up, interp);
    return {out, up};
}

// Calcolo dinamico spessore linea e dimensione font
std::pair<int, double> getStyle(int h, int w) {
    int side = std::max(h, w);
    int lw = std::max(2, static_cast<int>(std::round(side / 900.0)));
    double fs = std::max(0.7, side / 2600.0);
    return {lw, fs};
}

// Creazione del pannello laterale della legenda
cv::Mat createLegend(const std::vector<std::pair<std::string, int>>& rows,
                     int total,
                     const std::map<std::string, cv::Scalar>& colors,
                     int target_height,
                     const std::string& title) {
    int ph = std::max(target_height, static_cast<int>(54 * (rows.size() + 3)));
    cv::Mat panel(ph, 420, CV_8UC3, cv::Scalar(28, 28, 28));

    cv::putText(panel, title, cv::Point(18, 44), cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::putText(panel, "Total: " + std::to_string(total), cv::Point(18, 92), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    for (size_t r = 0; r < rows.size(); ++r) {
        int y = 150 + static_cast<int>(r) * 48;
        const auto& key = rows[r].first;
        int count = rows[r].second;

        cv::Scalar col(200, 200, 200);
        auto it = colors.find(key);
        if (it != colors.end()) {
            col = it->second;
        }

        cv::rectangle(panel, cv::Point(18, y - 26), cv::Point(52, y + 8), col, -1);
        cv::putText(panel, key + ": " + std::to_string(count), cv::Point(66, y + 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.95, cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
    }

    return panel;
}

// Unione dell'immagine con la legenda a destra
cv::Mat joinWithLegend(const cv::Mat& image, const cv::Mat& legend) {
    int h = image.rows;
    int w = image.cols;
    int ph = legend.rows;

    cv::Mat left;
    if (ph > h) {
        cv::Mat pad(ph - h, w, image.type(), cv::Scalar(0, 0, 0));
        cv::vconcat(image, pad, left);
    } else {
        left = image;
    }

    cv::Mat result;
    cv::hconcat(left, legend, result);
    return result;
}

} // anonymous namespace

// ----------------------------------------------------------------------------
// ComponentDetector Implementation
// ----------------------------------------------------------------------------

ComponentDetector::ComponentDetector() {
    fine_classes_ = getDefaultClasses();
}

ComponentDetector::ComponentDetector(const std::string& any_weights_path,
                                     const std::string& fine_weights_path,
                                     const std::vector<std::string>& custom_classes) {
    if (!custom_classes.empty()) {
        fine_classes_ = custom_classes;
    } else {
        fine_classes_ = getDefaultClasses();
    }

    if (!any_weights_path.empty()) {
        loadAnyModel(any_weights_path);
    }
    if (!fine_weights_path.empty()) {
        loadFineModel(fine_weights_path, fine_classes_);
    }
}

std::vector<std::string> ComponentDetector::getDefaultClasses() {
    return pcb::common::getDefaultClasses();
}

std::string ComponentDetector::getShortLabel(const std::string& full_label) {
    return pcb::common::getShortLabel(full_label);
}

std::map<std::string, cv::Scalar> ComponentDetector::getPalette(const std::vector<std::string>& labels) {
    std::vector<std::string> uniq;
    for (const auto& l : labels) {
        if (std::find(uniq.begin(), uniq.end(), l) == uniq.end()) {
            uniq.push_back(l);
        }
    }
    std::sort(uniq.begin(), uniq.end(), [](const std::string& a, const std::string& b) {
        if (a == "?" && b != "?") return false;
        if (b == "?" && a != "?") return true;
        return a < b;
    });

    std::map<std::string, cv::Scalar> palette;
    for (size_t i = 0; i < uniq.size(); ++i) {
        if (uniq[i] == "?") {
            palette["?"] = cv::Scalar(210, 210, 210);
            continue;
        }
        float hue = std::fmod(static_cast<float>(i) * 0.618033988749895f, 1.0f);
        palette[uniq[i]] = hsvToBgr(hue, 0.9f, 1.0f);
    }
    return palette;
}

std::string ComponentDetector::ensureOnnxModel(const std::string& weights_path, int imgsz) {
    if (weights_path.empty()) return "";

    fs::path p(weights_path);
    if (p.extension() == ".onnx") {
        if (fs::exists(p)) return p.string();
        std::cerr << "[ComponentDetector] File ONNX non trovato: " << p << std::endl;
        return "";
    }

    if (p.extension() == ".pt") {
        fs::path onnx_p = p;
        onnx_p.replace_extension(".onnx");

        if (fs::exists(onnx_p)) {
            std::cout << "[ComponentDetector] Trovato modello ONNX già esportato: " << onnx_p << std::endl;
            return onnx_p.string();
        }

        std::cout << "[ComponentDetector] Pesi in formato PyTorch .pt rilevati. Avvio esportazione in ONNX: "
                  << onnx_p << "..." << std::endl;

        // Cerca python nell'ambiente virtuale locale o di sistema
        std::vector<std::string> python_candidates = {
            "pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/.venv/bin/python3",
            "../pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/.venv/bin/python3",
            "python3",
            "python"
        };

        std::string python_bin = "";
        for (const auto& py : python_candidates) {
            if (fs::exists(py)) {
                python_bin = py;
                break;
            }
        }
        if (python_bin.empty()) python_bin = "python3";

        // Cerca lo script di esportazione
        std::vector<std::string> script_candidates = {
            "pcb_component_detection/export_onnx.py",
            "../pcb_component_detection/export_onnx.py"
        };

        std::string script_path = "";
        for (const auto& sc : script_candidates) {
            if (fs::exists(sc)) {
                script_path = sc;
                break;
            }
        }

        if (!script_path.empty()) {
            std::ostringstream cmd;
            cmd << python_bin << " " << script_path << " --weights \"" << p.string() << "\" --imgsz " << imgsz
                << " --out \"" << onnx_p.string() << "\"";
            std::cout << "[ComponentDetector] Esecuzione comando: " << cmd.str() << std::endl;
            int ret = std::system(cmd.str().c_str());
            if (ret == 0 && fs::exists(onnx_p)) {
                std::cout << "[ComponentDetector] Esportazione completata con successo." << std::endl;
                return onnx_p.string();
            }
        }

        // Se lo script fallisce, prova direttamente il comando da riga di comando yolo
        std::ostringstream cmd2;
        cmd2 << python_bin << " -m ultralytics yolo export model=\"" << p.string()
             << "\" format=onnx imgsz=" << imgsz << " opset=12 dynamic=False";
        std::cout << "[ComponentDetector] Tentativo export tramite CLI: " << cmd2.str() << std::endl;
        std::system(cmd2.str().c_str());

        if (fs::exists(onnx_p)) {
            std::cout << "[ComponentDetector] Esportazione CLI riuscita." << std::endl;
            return onnx_p.string();
        }

        std::cerr << "[ComponentDetector] Impossibile esportare " << p << " in formato ONNX. "
                  << "Verifica la presenza di ultralytics o esporta manualmente il modello." << std::endl;
        return "";
    }

    return weights_path;
}

bool ComponentDetector::loadAnyModel(const std::string& weights_path, int imgsz) {
    std::string onnx_path = ensureOnnxModel(weights_path, imgsz);
    if (onnx_path.empty() || !fs::exists(onnx_path)) {
        std::cerr << "[ComponentDetector] ERRORE: Impossibile caricare il modello 'any' da: "
                  << weights_path << std::endl;
        has_any_ = false;
        return false;
    }

    try {
        std::cout << "[ComponentDetector] Caricamento rete ANY: " << onnx_path << " ..." << std::endl;
        net_any_ = cv::dnn::readNetFromONNX(onnx_path);
        net_any_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_any_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        has_any_ = true;
        std::cout << "[ComponentDetector] Modello ANY caricato con successo." << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[ComponentDetector] Errore durante il caricamento del modello ANY: " << e.what() << std::endl;
        has_any_ = false;
        return false;
    }
}

bool ComponentDetector::loadFineModel(const std::string& weights_path,
                                      const std::vector<std::string>& class_names,
                                      int imgsz) {
    if (!class_names.empty()) {
        fine_classes_ = class_names;
    }

    std::string onnx_path = ensureOnnxModel(weights_path, imgsz);
    if (onnx_path.empty() || !fs::exists(onnx_path)) {
        std::cerr << "[ComponentDetector] ERRORE: Impossibile caricare il modello 'fine' da: "
                  << weights_path << std::endl;
        has_fine_ = false;
        return false;
    }

    try {
        std::cout << "[ComponentDetector] Caricamento rete FINE: " << onnx_path << " ..." << std::endl;
        net_fine_ = cv::dnn::readNetFromONNX(onnx_path);
        net_fine_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_fine_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        has_fine_ = true;
        std::cout << "[ComponentDetector] Modello FINE caricato con successo ("
                  << fine_classes_.size() << " classi configurate)." << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[ComponentDetector] Errore durante il caricamento del modello FINE: " << e.what() << std::endl;
        has_fine_ = false;
        return false;
    }
}

float ComponentDetector::calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    return pcb::common::calculateIoU(a, b);
}

void ComponentDetector::inferTile(
    cv::dnn::Net& net,
    const cv::Mat& tile,
    int offset_x,
    int offset_y,
    float conf_threshold,
    int imgsz,
    int num_classes,
    const std::vector<std::string>& class_names,
    std::vector<cv::Rect2f>& out_boxes,
    std::vector<float>& out_scores,
    std::vector<int>& out_classes) {

    (void)class_names;
    cv::Mat blob;
    cv::dnn::blobFromImage(tile, blob, 1.0 / 255.0, cv::Size(imgsz, imgsz), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    if (outputs.empty()) return;

    // Output tipico YOLOv8 / YOLO26: [1, 4 + C, N] oppure [1, N, 4 + C]
    cv::Mat out = outputs[0];
    int dims = out.dims;
    int rows = 0;
    int cols = 0;

    if (dims == 3) {
        rows = out.size[1];
        cols = out.size[2];
    } else if (dims == 2) {
        rows = out.rows;
        cols = out.cols;
    } else {
        return;
    }

    cv::Mat data_mat(rows, cols, CV_32F, (float*)out.data);
    cv::Mat predictions;
    if (rows == 4 + num_classes && cols > rows) {
        cv::transpose(data_mat, predictions); // Diventa N x (4 + num_classes)
    } else {
        predictions = data_mat;
    }

    int num_candidates = predictions.rows;
    int feature_len = predictions.cols;
    if (feature_len < 4 + num_classes) return;

    float scale_x = static_cast<float>(tile.cols) / static_cast<float>(imgsz);
    float scale_y = static_cast<float>(tile.rows) / static_cast<float>(imgsz);

    for (int i = 0; i < num_candidates; ++i) {
        const float* row_ptr = predictions.ptr<float>(i);
        float cx = row_ptr[0];
        float cy = row_ptr[1];
        float w = row_ptr[2];
        float h = row_ptr[3];

        float best_score = 0.0f;
        int best_cls = 0;

        if (num_classes == 1) {
            best_score = row_ptr[4];
            best_cls = 0;
        } else {
            for (int c = 0; c < num_classes; ++c) {
                float cls_score = row_ptr[4 + c];
                if (cls_score > best_score) {
                    best_score = cls_score;
                    best_cls = c;
                }
            }
        }

        if (best_score >= conf_threshold) {
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

std::vector<ComponentDetection> ComponentDetector::runInference(
    cv::dnn::Net& net,
    const cv::Mat& img,
    float conf_threshold,
    float nms_threshold,
    int imgsz,
    int tile_size,
    float overlap,
    int max_det,
    int num_classes,
    const std::vector<std::string>& class_names,
    bool agnostic_nms) {

    int h = img.rows;
    int w = img.cols;

    std::vector<cv::Rect2f> all_boxes;
    std::vector<float> all_scores;
    std::vector<int> all_classes;

    if (tile_size <= 0 || std::max(h, w) <= tile_size) {
        // Inferenza sull'intera immagine ridimensionata
        inferTile(net, img, 0, 0, conf_threshold, imgsz, num_classes, class_names,
                  all_boxes, all_scores, all_classes);
    } else {
        // Tiling a finestra scorrevole per immagini ad alta risoluzione
        int step = std::max(1, static_cast<int>(tile_size * (1.0f - overlap)));
        std::vector<int> xs, ys;
        for (int x = 0; x <= std::max(0, w - tile_size); x += step) xs.push_back(x);
        for (int y = 0; y <= std::max(0, h - tile_size); y += step) ys.push_back(y);

        if (xs.empty() || xs.back() + tile_size < w) xs.push_back(std::max(0, w - tile_size));
        if (ys.empty() || ys.back() + tile_size < h) ys.push_back(std::max(0, h - tile_size));

        for (int y : ys) {
            for (int x : xs) {
                int cur_tw = std::min(tile_size, w - x);
                int cur_th = std::min(tile_size, h - y);
                cv::Rect crop_roi(x, y, cur_tw, cur_th);
                cv::Mat crop = img(crop_roi);

                inferTile(net, crop, x, y, conf_threshold, imgsz, num_classes, class_names,
                          all_boxes, all_scores, all_classes);
            }
        }
    }

    if (all_boxes.empty()) return {};

    // Applicazione di NMS
    std::vector<cv::Rect> int_boxes;
    int_boxes.reserve(all_boxes.size());
    for (const auto& b : all_boxes) {
        int_boxes.emplace_back(static_cast<int>(std::round(b.x)),
                               static_cast<int>(std::round(b.y)),
                               static_cast<int>(std::round(b.width)),
                               static_cast<int>(std::round(b.height)));
    }

    std::vector<int> keep_indices;
    if (agnostic_nms || num_classes == 1) {
        cv::dnn::NMSBoxes(int_boxes, all_scores, conf_threshold, nms_threshold, keep_indices, 1.0f, max_det);
    } else {
        // Batched NMS per classe
        std::map<int, std::vector<int>> per_class_idx;
        for (size_t i = 0; i < all_classes.size(); ++i) {
            per_class_idx[all_classes[i]].push_back(static_cast<int>(i));
        }

        for (const auto& [cls, indices] : per_class_idx) {
            std::vector<cv::Rect> cls_boxes;
            std::vector<float> cls_scores;
            for (int idx : indices) {
                cls_boxes.push_back(int_boxes[idx]);
                cls_scores.push_back(all_scores[idx]);
            }
            std::vector<int> cls_keep;
            cv::dnn::NMSBoxes(cls_boxes, cls_scores, conf_threshold, nms_threshold, cls_keep);
            for (int k : cls_keep) {
                keep_indices.push_back(indices[k]);
            }
        }
    }

    std::vector<ComponentDetection> results;
    results.reserve(keep_indices.size());
    int id = 1;
    for (int idx : keep_indices) {
        ComponentDetection det;
        det.id = id++;
        det.box = all_boxes[idx];
        det.box_conf = all_scores[idx];
        det.class_id = all_classes[idx];

        if (num_classes == 1) {
            det.label = "component";
            det.label_conf = all_scores[idx];
        } else if (det.class_id >= 0 && det.class_id < static_cast<int>(class_names.size())) {
            det.label = class_names[det.class_id];
            det.label_conf = all_scores[idx];
        } else {
            det.label = "?";
            det.label_conf = 0.0f;
        }

        results.push_back(det);
    }

    return results;
}

DetectionResult ComponentDetector::detect(const cv::Mat& image,
                                          const DetectionConfig& config,
                                          const std::string& mode) {
    DetectionResult result;
    if (image.empty()) {
        std::cerr << "[ComponentDetector] Immagine vuota passata a detect()." << std::endl;
        return result;
    }

    cv::Mat proc_img;
    float scale = config.scale;
    if (scale != 1.0f && scale > 0.0f) {
        int interp = (scale < 1.0f) ? cv::INTER_AREA : cv::INTER_LINEAR;
        cv::resize(image, proc_img, cv::Size(), scale, scale, interp);
    } else {
        proc_img = image;
        scale = 1.0f;
    }

    if (config.use_cuda) {
        if (has_any_) {
            net_any_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_any_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        }
        if (has_fine_) {
            net_fine_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net_fine_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        }
    }

    if (mode == "any") {
        if (!has_any_) {
            std::cerr << "[ComponentDetector] Modello 'any' non caricato per la modalità 'any'." << std::endl;
            return result;
        }
        result.detections = runInference(net_any_, proc_img, config.conf_threshold, config.nms_threshold,
                                         config.imgsz, config.tile_size, config.overlap, config.max_det,
                                         1, {"component"}, true);
    } else if (mode == "fine") {
        if (!has_fine_) {
            std::cerr << "[ComponentDetector] Modello 'fine' non caricato per la modalità 'fine'." << std::endl;
            return result;
        }
        result.detections = runInference(net_fine_, proc_img, config.conf_threshold, config.nms_threshold,
                                         config.imgsz, config.tile_size, config.overlap, config.max_det,
                                         static_cast<int>(fine_classes_.size()), fine_classes_, false);
    } else { // "both"
        if (!has_any_ || !has_fine_) {
            std::cerr << "[ComponentDetector] Entrambi i modelli 'any' e 'fine' sono necessari per la modalità 'both'." << std::endl;
            return result;
        }

        // 1) any: trova tutte le componenti
        auto any_dets = runInference(net_any_, proc_img, config.conf_threshold, config.nms_threshold,
                                     config.imgsz, config.tile_size, config.overlap, config.max_det,
                                     1, {"component"}, true);

        // 2) fine: classifica le componenti
        auto fine_dets = runInference(net_fine_, proc_img, config.conf_threshold, config.nms_threshold,
                                      config.imgsz, config.tile_size, config.overlap, config.max_det,
                                      static_cast<int>(fine_classes_.size()), fine_classes_, false);

        // 3) match IoU: abbina ciascuna box 'any' con la migliore box 'fine'
        for (auto& ad : any_dets) {
            float max_iou = 0.0f;
            int best_fine_idx = -1;

            for (size_t j = 0; j < fine_dets.size(); ++j) {
                float iou = calculateIoU(ad.box, fine_dets[j].box);
                if (iou > max_iou) {
                    max_iou = iou;
                    best_fine_idx = static_cast<int>(j);
                }
            }

            if (max_iou >= config.match_iou && best_fine_idx >= 0) {
                ad.label = fine_dets[best_fine_idx].label;
                ad.label_conf = fine_dets[best_fine_idx].label_conf;
                ad.class_id = fine_dets[best_fine_idx].class_id;
            } else {
                ad.label = "?";
                ad.label_conf = 0.0f;
                ad.class_id = -1;
            }
        }
        result.detections = any_dets;
    }

    // Riscala le coordinate se l'immagine era stata scalata
    if (scale != 1.0f && scale > 0.0f) {
        float inv_scale = 1.0f / scale;
        for (auto& det : result.detections) {
            det.box.x *= inv_scale;
            det.box.y *= inv_scale;
            det.box.width *= inv_scale;
            det.box.height *= inv_scale;
        }
    }

    // Calcolo conteggi
    for (const auto& det : result.detections) {
        result.counts[det.label]++;
    }

    // Renderizza le immagini di visualizzazione
    result.elements_vis = drawElements(image, result.detections);
    if (mode != "any") {
        auto [vis, _] = drawTypes(image, result.detections, config.label_display_mode, config.min_label_px);
        result.types_vis = vis;
    }

    return result;
}

cv::Mat ComponentDetector::drawElements(const cv::Mat& img,
                                        const std::vector<ComponentDetection>& detections,
                                        int target,
                                        float fill) {
    auto [canvas, up] = makeCanvas(img, target);
    int h = canvas.rows;
    int w = canvas.cols;
    auto [lw, _] = getStyle(h, w);

    cv::Mat overlay = canvas.clone();
    for (const auto& det : detections) {
        cv::Rect box(static_cast<int>(std::round(det.box.x * up)),
                     static_cast<int>(std::round(det.box.y * up)),
                     static_cast<int>(std::round(det.box.width * up)),
                     static_cast<int>(std::round(det.box.height * up)));
        cv::rectangle(overlay, box, ELEMENT_COLOUR, -1);
    }

    cv::Mat out;
    cv::addWeighted(overlay, fill, canvas, 1.0f - fill, 0.0, out);

    for (const auto& det : detections) {
        cv::Rect box(static_cast<int>(std::round(det.box.x * up)),
                     static_cast<int>(std::round(det.box.y * up)),
                     static_cast<int>(std::round(det.box.width * up)),
                     static_cast<int>(std::round(det.box.height * up)));
        cv::rectangle(out, box, cv::Scalar(0, 0, 0), lw + 2, cv::LINE_AA);
        cv::rectangle(out, box, ELEMENT_COLOUR, lw, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> rows = {{"element", static_cast<int>(detections.size())}};
    std::map<std::string, cv::Scalar> colors = {{"element", ELEMENT_COLOUR}};
    cv::Mat legend = createLegend(rows, static_cast<int>(detections.size()), colors, h, "Elements");

    return joinWithLegend(out, legend);
}

std::pair<cv::Mat, std::map<std::string, int>> ComponentDetector::drawTypes(
    const cv::Mat& img,
    const std::vector<ComponentDetection>& detections,
    const std::string& label_mode,
    int min_label_px,
    int target) {

    auto [canvas, up] = makeCanvas(img, target);
    int h = canvas.rows;
    int w = canvas.cols;
    auto [lw, fs] = getStyle(h, w);

    std::vector<std::string> labels;
    labels.reserve(detections.size());
    for (const auto& det : detections) {
        labels.push_back(det.label);
    }
    auto palette = getPalette(labels);

    cv::Mat out = canvas.clone();

    // Disegna tutti i rettangoli
    for (const auto& det : detections) {
        cv::Rect box(static_cast<int>(std::round(det.box.x * up)),
                     static_cast<int>(std::round(det.box.y * up)),
                     static_cast<int>(std::round(det.box.width * up)),
                     static_cast<int>(std::round(det.box.height * up)));

        cv::Scalar col = palette[det.label];
        cv::rectangle(out, box, cv::Scalar(0, 0, 0), lw + 2, cv::LINE_AA);
        cv::rectangle(out, box, col, lw, cv::LINE_AA);
    }

    // Disegna le etichette di testo senza sovrapposizioni
    if (label_mode != "none") {
        int th_txt = std::max(1, lw - 1);
        std::vector<cv::Rect> taken_rects;

        // Ordina le componenti per area decrescente (le più grandi prima)
        std::vector<size_t> order(detections.size());
        for (size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return (detections[a].box.width * detections[a].box.height) >
                   (detections[b].box.width * detections[b].box.height);
        });

        for (size_t i : order) {
            const auto& det = detections[i];
            int x0 = static_cast<int>(std::round(det.box.x * up));
            int y0 = static_cast<int>(std::round(det.box.y * up));
            int bw = static_cast<int>(std::round(det.box.width * up));
            int bh = static_cast<int>(std::round(det.box.height * up));
            int x1 = x0 + bw;
            int y1 = y0 + bh;

            if (label_mode == "auto" && std::min(bw, bh) < min_label_px * up) {
                continue;
            }

            std::string text;
            if (label_mode == "ids") {
                text = std::to_string(det.id);
            } else if (label_mode == "names") {
                std::ostringstream ss;
                ss << det.label << " " << std::fixed << std::setprecision(2) << det.box_conf;
                text = ss.str();
            } else { // "auto"
                text = getShortLabel(det.label);
            }

            if (text.empty()) continue;

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fs, th_txt, &baseline);
            int tw = text_size.width;
            int th = text_size.height;

            // 4 posizioni possibili per l'etichetta
            std::vector<cv::Point> candidates = {
                {x0, y0 - 4},
                {x0, y1 + th + 4},
                {x1 - tw, y0 - 4},
                {x1 - tw, y1 + th + 4}
            };

            for (const auto& pt : candidates) {
                cv::Rect r(pt.x - 2, pt.y - th - 2, tw + 4, th + 6);
                if (r.x < 0 || r.y < 0 || r.x + r.width > w || r.y + r.height > h) {
                    continue;
                }

                bool overlaps = false;
                for (const auto& taken : taken_rects) {
                    if ((r & taken).area() > 0) {
                        overlaps = true;
                        break;
                    }
                }

                if (!overlaps) {
                    taken_rects.push_back(r);
                    cv::putText(out, text, pt, cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(0, 0, 0), th_txt + 3, cv::LINE_AA);
                    cv::putText(out, text, pt, cv::FONT_HERSHEY_SIMPLEX, fs, palette[det.label], th_txt, cv::LINE_AA);
                    break;
                }
            }
        }
    }

    // Conteggi ordinati
    std::map<std::string, int> counts;
    for (const auto& det : detections) {
        counts[det.label]++;
    }

    std::vector<std::pair<std::string, int>> rows(counts.begin(), counts.end());
    std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    cv::Mat legend = createLegend(rows, static_cast<int>(detections.size()), palette, h, "Types");
    return {joinWithLegend(out, legend), counts};
}

bool ComponentDetector::saveResults(const std::string& output_dir,
                                    const std::string& file_stem,
                                    const cv::Mat& orig_img,
                                    const DetectionResult& result,
                                    const std::string& mode) {
    (void)orig_img;
    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[ComponentDetector] Errore creazione cartella " << output_dir << ": " << ec.message() << std::endl;
        return false;
    }

    fs::path dir(output_dir);

    // 1) Step 1: WHERE (_elements.jpg)
    if (!result.elements_vis.empty()) {
        std::string elem_path = (dir / (file_stem + "_elements.jpg")).string();
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 92};
        cv::imwrite(elem_path, result.elements_vis, params);
        std::cout << "[ComponentDetector] Salvata immagine elementi: " << elem_path << std::endl;
    }

    // 2) Step 2: WHAT (_types.jpg)
    if (mode != "any" && !result.types_vis.empty()) {
        std::string types_path = (dir / (file_stem + "_types.jpg")).string();
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 92};
        cv::imwrite(types_path, result.types_vis, params);
        std::cout << "[ComponentDetector] Salvata immagine tipi: " << types_path << std::endl;
    }

    // 3) Salvataggio CSV (_detections.csv)
    std::string csv_path = (dir / (file_stem + "_detections.csv")).string();
    std::ofstream csv_file(csv_path);
    if (csv_file.is_open()) {
        csv_file << "id,label,box_conf,label_conf,x0,y0,x1,y1\n";
        for (const auto& d : result.detections) {
            int x0 = static_cast<int>(std::round(d.box.x));
            int y0 = static_cast<int>(std::round(d.box.y));
            int x1 = static_cast<int>(std::round(d.box.x + d.box.width));
            int y1 = static_cast<int>(std::round(d.box.y + d.box.height));
            csv_file << d.id << ","
                     << d.label << ","
                     << std::fixed << std::setprecision(3) << d.box_conf << ","
                     << std::fixed << std::setprecision(3) << d.label_conf << ","
                     << x0 << "," << y0 << "," << x1 << "," << y1 << "\n";
        }
        csv_file.close();
        std::cout << "[ComponentDetector] Salvato report CSV: " << csv_path << std::endl;
    }

    // 4) Salvataggio JSON (_summary.json)
    std::string json_path = (dir / (file_stem + "_summary.json")).string();
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
        json_file << "{\n";
        json_file << "  \"image\": \"" << file_stem << "\",\n";
        json_file << "  \"total\": " << result.detections.size() << ",\n";
        json_file << "  \"counts\": {\n";
        size_t count_idx = 0;
        for (auto it = result.counts.begin(); it != result.counts.end(); ++it, ++count_idx) {
            json_file << "    \"" << it->first << "\": " << it->second;
            if (count_idx + 1 < result.counts.size()) {
                json_file << ",";
            }
            json_file << "\n";
        }
        json_file << "  }\n";
        json_file << "}\n";
        json_file.close();
        std::cout << "[ComponentDetector] Salvato riepilogo JSON: " << json_path << std::endl;
    }

    return true;
}

} // namespace pcb
