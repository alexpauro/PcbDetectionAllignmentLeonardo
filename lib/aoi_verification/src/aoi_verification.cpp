/**
 * @file aoi_verification.cpp
 * @brief Implementazione principale della classe AoiVerificationProcessor e pipeline di esecuzione.
 */

#include "aoi_verification.hpp"
#include "aoi_internal.hpp"
#include "common_utils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace pcb {

AoiVerificationProcessor::AoiVerificationProcessor() {}

std::vector<std::string> AoiVerificationProcessor::getSupportedClasses() {
    return pcb::common::getDefaultClasses();
}

bool AoiVerificationProcessor::saveResults(
    const std::string& outputDir,
    const std::string& paramsDir,
    const std::string& baseStem,
    const AoiVerificationResult& result) const {

    std::error_code ec;
    fs::create_directories(outputDir, ec);
    fs::create_directories(paramsDir, ec);

    // 1. Salva CSV dettagliato (escludendo extra non-AOI per coerenza)
    std::string csvPath = (fs::path(outputDir) / (baseStem + "_aoi_verified_components.csv")).string();
    std::ofstream csv(csvPath);
    if (csv.is_open()) {
        csv << "id,ref_des,status,final_label,original_label,confidence,source,center_x,center_y,width,height,"
            << "angle_deg,radius,area,nominal_x_mm,nominal_y_mm,nominal_width_mm,nominal_height_mm,"
            << "nominal_x_px,nominal_y_px,offset_px,offset_mm,aoi_part_number,aoi_outline,notes,"
            << "datasheet_width_mm,datasheet_height_mm,datasheet_thickness_mm,datasheet_source,nominal_rot_deg\n";

        for (const auto& c : result.verified_components) {
            if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) continue;

            std::string stStr = "MATCH_EXACT";
            if (c.status == AoiMatchStatus::CLASS_CORRECTED) stStr = "CLASS_CORRECTED";
            else if (c.status == AoiMatchStatus::RECOVERED_BY_AOI) stStr = "RECOVERED_BY_AOI";

            double nomX_mm = c.nominal_x_mm, nomY_mm = c.nominal_y_mm, nomRot_deg = c.nominal_rot_deg;
            if (nomX_mm == 0.0 && nomY_mm == 0.0) {
                auto itAoi = std::find_if(result.aoi_components.begin(), result.aoi_components.end(), [&](const AoiComponent& a) {
                    return a.ref_des == c.ref_des;
                });
                if (itAoi != result.aoi_components.end()) {
                    nomX_mm = itAoi->x_mm;
                    nomY_mm = itAoi->y_mm;
                    nomRot_deg = itAoi->rot_deg;
                }
            }

            csv << c.id << ",\""
                << c.ref_des << "\",\""
                << stStr << "\",\""
                << c.label << "\",\""
                << c.original_label << "\","
                << std::fixed << std::setprecision(3) << c.confidence << ",\""
                << c.source_name << "\","
                << std::fixed << std::setprecision(2)
                << c.center.x << "," << c.center.y << ","
                << c.box.width << "," << c.box.height << ","
                << c.rotated_box.angle << "," << c.radius << "," << c.area << ","
                << std::setprecision(3) << nomX_mm << "," << nomY_mm << ","
                << c.nominal_width_mm << "," << c.nominal_height_mm << ","
                << std::setprecision(2) << c.nominal_center_px.x << "," << c.nominal_center_px.y << ","
                << c.offset_error_px << "," << std::setprecision(3) << c.offset_error_mm << ",\""
                << c.aoi_part_number << "\",\""
                << c.aoi_outline << "\",\""
                << c.notes << "\","
                << c.datasheet_width_mm << ","
                << c.datasheet_height_mm << ","
                << c.datasheet_thickness_mm << ",\""
                << c.datasheet_source << "\","
                << std::setprecision(2) << nomRot_deg << "\n";
        }
        std::cout << "[AOI] Salvato CSV componenti verificati in: " << csvPath << std::endl;
    }

    // 2. Salva JSON di riepilogo
    std::string jsonPath = (fs::path(outputDir) / (baseStem + "_aoi_verified_summary.json")).string();
    std::ofstream js(jsonPath);
    if (js.is_open()) {
        js << "{\n";
        js << "  \"board\": \"" << result.board_name << "\",\n";
        js << "  \"side\": \"" << result.side << "\",\n";
        js << "  \"total_aoi_nominal\": " << result.summary.total_aoi_nominal << ",\n";
        js << "  \"total_detections_input\": " << result.summary.total_detections_input << ",\n";
        js << "  \"exact_matches\": " << result.summary.exact_matches << ",\n";
        js << "  \"class_corrected\": " << result.summary.class_corrected << ",\n";
        js << "  \"recovered_by_aoi\": " << result.summary.recovered_by_aoi << ",\n";
        js << "  \"mean_offset_error_px\": " << result.summary.mean_offset_error_px << ",\n";
        js << "  \"mean_offset_error_mm\": " << result.summary.mean_offset_error_mm << ",\n";
        js << "  \"transform\": {\n";
        js << "    \"orientation\": \"" << result.transform.orientation_name << "\",\n";
        js << "    \"distortion_model\": \"" << (result.transform.has_distortion_model ? "Homography 3x3" : "Linear") << "\",\n";
        js << "    \"tx\": " << result.transform.tx << ",\n";
        js << "    \"ty\": " << result.transform.ty << ",\n";
        js << "    \"scale_px_per_mm\": " << result.transform.scale_px_per_mm << ",\n";
        js << "    \"inlier_count\": " << result.transform.inlier_count << ",\n";
        js << "    \"inlier_ratio\": " << result.transform.inlier_ratio << "\n";
        js << "  },\n";
        js << "  \"class_counts\": {\n";
        bool first = true;
        for (const auto& [label, cnt] : result.summary.class_distribution) {
            if (!first) js << ",\n";
            first = false;
            js << "    \"" << label << "\": " << cnt;
        }
        js << "\n  }\n";
        js << "}\n";
        std::cout << "[AOI] Salvato JSON riepilogo in: " << jsonPath << std::endl;
    }

    // 3. Salva immagini annotate (senza extra)
    if (!result.best_alignment_vis.empty()) {
        std::string imgAlignPath = (fs::path(outputDir) / (baseStem + "_best_alignment.jpg")).string();
        cv::imwrite(imgAlignPath, result.best_alignment_vis);
        std::cout << "[AOI] Salvata immagine allineamento in: " << imgAlignPath << std::endl;
    }

    if (!result.verified_types_vis.empty()) {
        std::string imgTypesPath = (fs::path(outputDir) / (baseStem + "_verified_types.jpg")).string();
        cv::imwrite(imgTypesPath, result.verified_types_vis);
        std::cout << "[AOI] Salvata immagine tipi verificati in: " << imgTypesPath << std::endl;
    }

    if (!result.verified_elements_vis.empty()) {
        std::string imgElemsPath = (fs::path(outputDir) / (baseStem + "_verified_elements.jpg")).string();
        cv::imwrite(imgElemsPath, result.verified_elements_vis);
        std::cout << "[AOI] Salvata immagine elementi verificati in: " << imgElemsPath << std::endl;
    }

    // 4. Salva parametri YAML
    std::string yamlPath = (fs::path(paramsDir) / ("aoi_alignment_" + result.board_name + "_" + result.side + ".yml")).string();
    cv::FileStorage fsYaml(yamlPath, cv::FileStorage::WRITE);
    if (fsYaml.isOpened()) {
        fsYaml << "board_name" << result.board_name;
        fsYaml << "side" << result.side;
        fsYaml << "orientation_name" << result.transform.orientation_name;
        fsYaml << "has_distortion_model" << result.transform.has_distortion_model;
        if (result.transform.has_distortion_model) {
            fsYaml << "homography_3x3" << result.transform.homography_3x3;
        }
        fsYaml << "tx" << result.transform.tx;
        fsYaml << "ty" << result.transform.ty;
        fsYaml << "scale_px_per_mm" << result.transform.scale_px_per_mm;
        fsYaml << "inlier_count" << result.transform.inlier_count;
        fsYaml << "inlier_ratio" << result.transform.inlier_ratio;
        fsYaml << "avg_error_px" << result.transform.avg_error_px;
        fsYaml << "avg_error_mm" << result.transform.avg_error_mm;
        fsYaml.release();
        std::cout << "[AOI] Salvati parametri YAML in: " << yamlPath << std::endl;
    }

    return true;
}

bool AoiVerificationProcessor::processBoardSide(
    const std::string& boardName,
    const std::string& side,
    const std::string& undistortedDir,
    const std::string& processedOutputDir,
    const std::string& aoiBaseDir,
    const std::string& gerberBaseDir,
    const std::string& paramsDir,
    const std::string& verificationOutputDir,
    bool showWindows,
    const std::string& preferredBaseImage,
    const std::string& cvDetectionOutputDir) {

    std::cout << "\n========================================================" << std::endl;
    std::cout << " AVVIO VERIFICA AOI: " << boardName << " [" << side << "]" << std::endl;
    std::cout << "========================================================" << std::endl;

    // 1. Cerca il file AOI (.asc)
    fs::path aoiFilePath;
    for (const auto& entry : fs::recursive_directory_iterator(aoiBaseDir)) {
        if (entry.is_regular_file()) {
            std::string pStr = entry.path().string();
            std::string ext = entry.path().extension().string();
            if ((ext == ".asc" || ext == ".txt" || ext == ".csv") &&
                pStr.find(boardName) != std::string::npos &&
                pStr.find(side) != std::string::npos) {
                aoiFilePath = entry.path();
                break;
            }
        }
    }

    if (aoiFilePath.empty() || !fs::exists(aoiFilePath)) {
        std::cerr << "[AOI] ERRORE: Nessun file AOI (.asc) trovato per "
                  << boardName << " [" << side << "] in: " << aoiBaseDir << std::endl;
        return false;
    }

    std::string mappingFilePath = (fs::path(paramsDir) / "component_cv_mapping.txt").string();

    // 2. Parsing file AOI e associazione classi
    std::vector<AoiComponent> aoiComps = parseAoiFile(aoiFilePath.string(), mappingFilePath, true);
    if (aoiComps.empty()) {
        std::cerr << "[AOI] ERRORE: Nessun componente estratto dal file AOI." << std::endl;
        return false;
    }

    // 3. Notifica sui requisiti utente:
    // "l'ingombro riconosciuto tramite il file gerber è molto sbagliato, non usare più questo metodo"
    // "utilizza i riquadri trovati in dl per i componenti riconosciuti"
    std::cout << "[AOI] Ingombro da piazzole Gerber disabilitato come da specifica utente." << std::endl;
    std::cout << "[AOI] Riquadri determinati direttamente da DL e da sovrapposizione multi-foto CV." << std::endl;

    // 4. Leggi scala da board_alignment_<BOARD>_<SIDE>.yml
    std::string boardAlignYaml = (fs::path(paramsDir) / ("board_alignment_" + boardName + "_" + side + ".yml")).string();
    double pixels_per_mm = 0.0;
    cv::FileStorage fsAlign(boardAlignYaml, cv::FileStorage::READ);
    if (fsAlign.isOpened()) {
        fsAlign["pixels_per_mm"] >> pixels_per_mm;
        fsAlign.release();
    }

    if (pixels_per_mm <= 1e-4) {
        pixels_per_mm = 11.39326;
    }

    // 5. Carica immagine PCB di riferimento scelta dall'utente (per tutte le fasi successive)
    // REQUISITO UTENTE:
    // "usa l'immagine scelta dall'utente per l'output di riconoscimento componenti in tutte le fasi successive (è solo una visualizzazione grafica)"
    fs::path sideUndistDir = fs::path(undistortedDir) / boardName / side;
    cv::Mat pcbImg;
    std::string chosenStem = preferredBaseImage;

    // Se non passata esplicitamente, leggi l'immagine scelta dall'utente dal riepilogo della fase precedente
    if (chosenStem.empty()) {
        fs::path summaryPath = fs::path(processedOutputDir) / boardName / side / "final_fused_summary.json";
        if (!fs::exists(summaryPath)) {
            summaryPath = fs::path(processedOutputDir) / boardName / side / "step1_merged_dl_summary.json";
        }
        if (fs::exists(summaryPath)) {
            std::ifstream sj(summaryPath);
            std::string sline;
            while (std::getline(sj, sline)) {
                size_t pos = sline.find("\"best_reference_image\":");
                if (pos != std::string::npos) {
                    size_t q1 = sline.find('\"', pos + 23);
                    if (q1 != std::string::npos) {
                        size_t q2 = sline.find('\"', q1 + 1);
                        if (q2 != std::string::npos) {
                            chosenStem = sline.substr(q1 + 1, q2 - q1 - 1);
                        }
                    }
                    break;
                }
            }
        }
    }

    std::string baseStem = boardName + "_" + side;
    if (!chosenStem.empty() && fs::exists(sideUndistDir)) {
        for (const auto& ext : {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"}) {
            fs::path p = sideUndistDir / (chosenStem + ext);
            if (fs::exists(p)) {
                pcbImg = cv::imread(p.string());
                if (!pcbImg.empty()) {
                    baseStem = chosenStem;
                    std::cout << "[AOI] Utilizzo dell'immagine base scelta dall'utente per il rendering grafico: "
                              << p.filename().string() << std::endl;
                    break;
                }
            }
        }
    }

    // Fallback se l'immagine specifica non è stata trovata
    if (pcbImg.empty() && fs::exists(sideUndistDir)) {
        for (const auto& entry : fs::directory_iterator(sideUndistDir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                if (ext == ".bmp" || ext == ".png" || ext == ".jpg") {
                    pcbImg = cv::imread(entry.path().string());
                    if (!pcbImg.empty()) {
                        baseStem = entry.path().stem().string();
                        break;
                    }
                }
            }
        }
    }

    // 6. Carica la sovrapposizione di tutte le foto e di tutti i componenti in CV
    // REQUISITO UTENTE:
    // "utilizza la sovrapposizione di tutte le foto e di tutti i componenti fatta in cv per recuperare i componenti non riconosciuti"
    std::string cvDir = cvDetectionOutputDir;
    if (cvDir.empty()) {
        fs::path p = fs::path(processedOutputDir).parent_path() / "pcb_cv_component_detection";
        if (fs::exists(p)) cvDir = p.string();
        else if (fs::exists("data/outputs/pcb_cv_component_detection")) cvDir = "data/outputs/pcb_cv_component_detection";
        else cvDir = "../data/outputs/pcb_cv_component_detection";
    }
    std::vector<CvComponent> allCvComps = loadAllCvDetectionsForBoardSide(cvDir, boardName, side);

    // 7. Carica i componenti rilevati in ingresso da final_fused_components.csv
    fs::path fusedCsvPath = fs::path(processedOutputDir) / boardName / side / "final_fused_components.csv";
    std::vector<FinalComponent> detections;

    if (fs::exists(fusedCsvPath)) {
        std::ifstream cfile(fusedCsvPath);
        std::string line;
        std::getline(cfile, line);
        while (std::getline(cfile, line)) {
            std::string s = trim(line);
            if (s.empty()) continue;

            std::vector<std::string> fields;
            bool insideQuotes = false;
            std::string curToken;
            for (char ch : s) {
                if (ch == '\"') insideQuotes = !insideQuotes;
                else if (ch == ',' && !insideQuotes) {
                    fields.push_back(trim(curToken));
                    curToken.clear();
                } else curToken += ch;
            }
            fields.push_back(trim(curToken));

            if (fields.size() >= 12) {
                FinalComponent fc;
                fc.id = std::stoi(fields[0]);
                fc.label = fields[2];
                fc.confidence = std::stof(fields[3]);
                fc.center.x = std::stof(fields[5]);
                fc.center.y = std::stof(fields[6]);
                float w = std::stof(fields[7]);
                float h = std::stof(fields[8]);
                fc.box = cv::Rect2f(fc.center.x - w / 2, fc.center.y - h / 2, w, h);
                float ang = std::stof(fields[9]);
                fc.rotated_box = cv::RotatedRect(fc.center, cv::Size2f(w, h), ang);
                fc.shape = (fields[4] == "circle") ? ShapeType::CIRCLE : ShapeType::RECTANGLE;
                fc.radius = std::stof(fields[10]);
                fc.area = std::stof(fields[11]);
                detections.push_back(fc);
            }
        }
    }

    double imgW = pcbImg.empty() ? 1227.0 : static_cast<double>(pcbImg.cols);
    double imgH = pcbImg.empty() ? 889.0 : static_cast<double>(pcbImg.rows);

    // 8. Trova l'origine ed esegui la modellazione della distorsione ottica
    AoiAlignmentTransform transform = findBestOriginAndDistortionAlignment(aoiComps, detections, pixels_per_mm, imgW, imgH, 3.5);
    if (!transform.valid) {
        std::cerr << "[AOI] Impossibile determinare l'allineamento geometrico con AOI." << std::endl;
        return false;
    }

    // 9. Esegui verifica, correzione classi, recupero tramite sovrapposizione CV e risoluzione sovrapposizioni
    AoiVerificationResult result = verifyAndClassify(pcbImg, aoiComps, detections, allCvComps, transform, 3.2);
    result.board_name = boardName;
    result.side = side;

    // 10. Genera immagini di visualizzazione sull'immagine scelta dall'utente (I componenti extra NON vengono rappresentati)
    result.best_alignment_vis = renderBestAlignmentOverlay(pcbImg, aoiComps, detections, transform);
    result.verified_types_vis = renderVerifiedTypesOverlay(pcbImg, result.verified_components);
    result.verified_elements_vis = renderVerifiedElementsOverlay(pcbImg, result.verified_components);

    // 11. Salva tutti i risultati
    std::string outSideDir = (fs::path(verificationOutputDir) / boardName / side).string();
    saveResults(outSideDir, paramsDir, baseStem, result);

    // 12. Mostra finestre se richiesto
    if (showWindows && !result.best_alignment_vis.empty()) {
        std::string win1 = "Verifica AOI: Allineamento Coordinate - " + boardName + " [" + side + "]";
        cv::namedWindow(win1, cv::WINDOW_NORMAL);
        cv::resizeWindow(win1, 1300, 850);
        cv::imshow(win1, result.best_alignment_vis);

        std::string win2 = "Verifica AOI: Componenti Verificati - " + boardName + " [" + side + "]";
        cv::namedWindow(win2, cv::WINDOW_NORMAL);
        cv::resizeWindow(win2, 1400, 900);
        cv::imshow(win2, result.verified_types_vis);

        std::cout << "\nPremi un tasto su una finestra OpenCV per continuare..." << std::endl;
        cv::waitKey(0);
        cv::destroyWindow(win1);
        cv::destroyWindow(win2);
    }

    return true;
}

} // namespace pcb
