/**
 * @file aoi_engine.cpp
 * @brief Implementazione del motore di matching, fusione e verifica AOI vs Detections.
 */

#include "aoi_verification.hpp"
#include "aoi_internal.hpp"
#include "output_processor.hpp"
#include "component_detector.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <filesystem>

namespace fs = std::filesystem;

namespace pcb {

std::vector<CvComponent> AoiVerificationProcessor::loadAllCvDetectionsForBoardSide(
    const std::string& cvOutputDir,
    const std::string& boardName,
    const std::string& side) {

    std::vector<CvComponent> allComps;
    fs::path sideDir = fs::path(cvOutputDir) / boardName / side;
    if (!fs::exists(sideDir)) {
        if (fs::exists(cvOutputDir)) sideDir = cvOutputDir;
        else {
            std::cerr << "[AOI] Cartella rilevamenti CV non trovata: " << sideDir << std::endl;
            return allComps;
        }
    }

    std::vector<fs::path> csvFiles;
    for (const auto& entry : fs::directory_iterator(sideDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string name = entry.path().filename().string();
            if (name.find("_components_cv.csv") != std::string::npos) {
                csvFiles.push_back(entry.path());
            }
        }
    }
    std::sort(csvFiles.begin(), csvFiles.end());

    std::cout << "[AOI] Caricamento sovrapposizione CV da " << csvFiles.size() << " foto..." << std::endl;
    for (const auto& csvPath : csvFiles) {
        auto comps = OutputProcessor::loadCvComponentsCsv(csvPath.string());
        allComps.insert(allComps.end(), comps.begin(), comps.end());
    }

    std::cout << "[AOI] Sovrapposizione multi-foto CV completata: caricati "
              << allComps.size() << " elementi geometrici da " << csvFiles.size() << " esposizioni." << std::endl;
    return allComps;
}

void AoiVerificationProcessor::resolveOverlapsWithDl(std::vector<VerifiedPcbComponent>& verifiedComps) {
    // 1. Raccogli tutti i componenti DL-confirmed (status esatto o corretto da DL)
    std::vector<size_t> dlIndices;
    std::vector<size_t> recIndices;

    for (size_t i = 0; i < verifiedComps.size(); ++i) {
        if (verifiedComps[i].status == AoiMatchStatus::UNMATCHED_DETECTION) continue;

        // I componenti riconosciuti da DL non possono mai essere sovrapposti né modificati ("DL non sbaglia mai")
        if (verifiedComps[i].status == AoiMatchStatus::MATCH_EXACT_CLASS ||
            verifiedComps[i].status == AoiMatchStatus::CLASS_CORRECTED) {
            dlIndices.push_back(i);
        } else if (verifiedComps[i].status == AoiMatchStatus::RECOVERED_BY_AOI) {
            recIndices.push_back(i);
        }
    }

    // 2. Controllo e ridimensionamento componenti CV / recuperati rispetto ai riquadri DL
    for (size_t rIdx : recIndices) {
        auto& rec = verifiedComps[rIdx];
        cv::Rect2f r_rec = rec.box;

        for (size_t dIdx : dlIndices) {
            const auto& dl = verifiedComps[dIdx];
            cv::Rect2f r_dl = dl.box;

            cv::Rect2f inter = r_rec & r_dl;
            if (inter.width > 0.5f && inter.height > 0.5f) {
                // Sovrapposizione con DL rilevata: ridimensiona il box recuperato/CV
                float dx = rec.center.x - dl.center.x;
                float dy = rec.center.y - dl.center.y;

                bool cutHorizontal = (std::abs(dx) / std::max(1.0f, r_dl.width) >= std::abs(dy) / std::max(1.0f, r_dl.height));

                if (cutHorizontal) {
                    if (dx > 0) {
                        float new_x = r_dl.x + r_dl.width + 1.0f;
                        float new_w = (r_rec.x + r_rec.width) - new_x;
                        if (new_w >= 4.0f) {
                            r_rec.x = new_x;
                            r_rec.width = new_w;
                        } else {
                            cutHorizontal = false;
                        }
                    } else {
                        float new_w = r_dl.x - 1.0f - r_rec.x;
                        if (new_w >= 4.0f) {
                            r_rec.width = new_w;
                        } else {
                            cutHorizontal = false;
                        }
                    }
                }

                if (!cutHorizontal) {
                    if (dy > 0) {
                        float new_y = r_dl.y + r_dl.height + 1.0f;
                        float new_h = (r_rec.y + r_rec.height) - new_y;
                        if (new_h >= 4.0f) {
                            r_rec.y = new_y;
                            r_rec.height = new_h;
                        }
                    } else {
                        float new_h = r_dl.y - 1.0f - r_rec.y;
                        if (new_h >= 4.0f) {
                            r_rec.height = new_h;
                        }
                    }
                }

                r_rec.width = std::max(2.0f, r_rec.width);
                r_rec.height = std::max(2.0f, r_rec.height);
                rec.box = r_rec;
                rec.center = cv::Point2f(r_rec.x + r_rec.width * 0.5f, r_rec.y + r_rec.height * 0.5f);
                rec.rotated_box = cv::RotatedRect(rec.center, r_rec.size(), rec.rotated_box.angle);
                rec.area = r_rec.width * r_rec.height;
                rec.notes += " [Riquadro ridimensionato per evitare sovrapposizione con DL (" + dl.ref_des + ")]";
            }
        }
    }

    // 3. Risoluzione sovrapposizioni mutue tra componenti recuperati
    for (size_t i = 0; i < recIndices.size(); ++i) {
        for (size_t j = i + 1; j < recIndices.size(); ++j) {
            auto& c1 = verifiedComps[recIndices[i]];
            auto& c2 = verifiedComps[recIndices[j]];
            cv::Rect2f inter = c1.box & c2.box;
            if (inter.width > 0.5f && inter.height > 0.5f) {
                auto& toTrim = (c1.offset_error_px > c2.offset_error_px) ? c1 : c2;
                const auto& toStay = (c1.offset_error_px > c2.offset_error_px) ? c2 : c1;

                cv::Rect2f r_trim = toTrim.box;
                cv::Rect2f r_stay = toStay.box;
                float dx = toTrim.center.x - toStay.center.x;
                float dy = toTrim.center.y - toStay.center.y;

                if (std::abs(dx) >= std::abs(dy)) {
                    if (dx > 0) {
                        float new_x = r_stay.x + r_stay.width + 1.0f;
                        float new_w = (r_trim.x + r_trim.width) - new_x;
                        if (new_w >= 4.0f) { r_trim.x = new_x; r_trim.width = new_w; }
                    } else {
                        float new_w = r_stay.x - 1.0f - r_trim.x;
                        if (new_w >= 4.0f) { r_trim.width = new_w; }
                    }
                } else {
                    if (dy > 0) {
                        float new_y = r_stay.y + r_stay.height + 1.0f;
                        float new_h = (r_trim.y + r_trim.height) - new_y;
                        if (new_h >= 4.0f) { r_trim.y = new_y; r_trim.height = new_h; }
                    } else {
                        float new_h = r_stay.y - 1.0f - r_trim.y;
                        if (new_h >= 4.0f) { r_trim.height = new_h; }
                    }
                }
                r_trim.width = std::max(2.0f, r_trim.width);
                r_trim.height = std::max(2.0f, r_trim.height);
                toTrim.box = r_trim;
                toTrim.center = cv::Point2f(r_trim.x + r_trim.width * 0.5f, r_trim.y + r_trim.height * 0.5f);
                toTrim.rotated_box = cv::RotatedRect(toTrim.center, r_trim.size(), toTrim.rotated_box.angle);
                toTrim.area = r_trim.width * r_trim.height;
            }
        }
    }
}

AoiVerificationResult AoiVerificationProcessor::verifyAndClassify(
    const cv::Mat& pcbImage,
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    const std::vector<CvComponent>& allCvSuperposition,
    const AoiAlignmentTransform& transform,
    double tolerance_mm) const {

    AoiVerificationResult res;
    res.aoi_components = aoiComps;
    res.transform = transform;
    res.summary.total_aoi_nominal = static_cast<int>(aoiComps.size());
    res.summary.total_detections_input = static_cast<int>(detections.size());

    if (!transform.valid) {
        std::cerr << "[AOI] Trasformazione non valida: impossibile procedere alla verifica." << std::endl;
        res.success = false;
        return res;
    }

    const double s = transform.scale_px_per_mm;
    const double tol_px = tolerance_mm * s;

    // Se la sovrapposizione CV non è fornita, eseguiamo CvComponentDetector al volo su pcbImage
    std::vector<CvComponent> cvPool = allCvSuperposition;
    if (cvPool.empty() && !pcbImage.empty()) {
        CvComponentDetector cvDetector;
        CvDetectionConfig cvCfg;
        cvCfg.show_windows = false;
        CvDetectionResult cvRes = cvDetector.detect(pcbImage, cvCfg);
        cvPool = cvRes.components;
        std::cout << "[AOI] Eseguito rilevamento CV al volo: trovati "
                  << cvPool.size() << " elementi geometrici su PCB." << std::endl;
    }

    // Pool dei componenti riconosciuti da Deep Learning e fusione (da final_fused_components)
    struct CandidateMatch {
        int id = 0;
        cv::Point2f center;
        cv::Rect2f box;
        cv::RotatedRect rbox;
        ShapeType shape = ShapeType::RECTANGLE;
        float radius = 0.0f;
        float area = 0.0f;
        std::string label = "?";
        float conf = 0.0f;
        std::string source;
        bool used = false;
    };

    std::vector<CandidateMatch> pool;
    for (const auto& d : detections) {
        CandidateMatch cm;
        cm.id = d.id;
        cm.center = d.center;
        cm.box = d.box;
        cm.rbox = d.rotated_box;
        cm.shape = d.shape;
        cm.radius = d.radius;
        cm.area = d.area;
        cm.label = d.label;
        cm.conf = d.confidence;
        cm.source = OutputProcessor::sourceToString(d.source);
        cm.used = false;
        pool.push_back(cm);
    }

    // Raccoglitore dimensioni canoniche da DL per part_number (stesso codice componente = stesse dimensioni)
    std::map<std::string, std::vector<cv::Size2f>> partNumberDlSizes;

    int nextId = 1;
    double totalOffsetErrPx = 0.0;
    int offsetCount = 0;

    // FASE 1: Associazione con componenti riconosciuti da DL
    // REQUISITI UTENTE:
    // - "utilizza i riquadri trovati in dl per i componenti riconosciuti"
    // - "dl non sbaglia mai"
    for (size_t i = 0; i < res.aoi_components.size(); ++i) {
        auto& a = res.aoi_components[i];
        cv::Point2f nomPx = transform.aoiToPixel(a.x_mm, a.y_mm);
        a.nominal_px = nomPx;

        int bestIdx = -1;
        double bestDist = 1e9;
        double searchTol = tol_px;

        for (size_t p = 0; p < pool.size(); ++p) {
            if (pool[p].used) continue;
            double dist = cv::norm(nomPx - pool[p].center);
            if (dist <= searchTol) {
                double penalty = (pool[p].label == a.assigned_class) ? 0.0 :
                                 (pool[p].label == "?") ? 2.0 : 5.0;
                if (dist + penalty < bestDist) {
                    bestDist = dist + penalty;
                    bestIdx = static_cast<int>(p);
                }
            }
        }

        VerifiedPcbComponent vc;
        vc.id = nextId++;
        vc.ref_des = a.ref_des;
        vc.label = a.assigned_class;
        vc.aoi_part_number = a.part_number;
        vc.aoi_outline = a.outline;
        vc.aoi_description = a.description;
        vc.nominal_center_px = nomPx;
        vc.nominal_x_mm = a.x_mm;
        vc.nominal_y_mm = a.y_mm;
        vc.nominal_rot_deg = a.rot_deg;

        if (bestIdx >= 0) {
            pool[bestIdx].used = true;
            const auto& matched = pool[bestIdx];

            a.matched = true;
            a.matched_detection_id = matched.id;
            double realDist = cv::norm(nomPx - matched.center);
            a.match_distance_px = realDist;
            a.match_distance_mm = realDist / s;

            // REQUISITO UTENTE: I box trovati in processed output sono SEMPRE corretti.
            // aoi_verification deve solo guardare la label dei componenti trovati precedentemente
            // e cambiarla solo in quella giusta se necessario (es. D306 e D406), SENZA modificare il box.
            vc.center = matched.center;
            vc.box = matched.box;
            if (matched.rbox.size.width > 0.0f && matched.rbox.size.height > 0.0f) {
                vc.rotated_box = matched.rbox;
            } else {
                vc.rotated_box = cv::RotatedRect(matched.center, matched.box.size(), 0.0f);
            }
            vc.shape = matched.shape;
            vc.radius = matched.radius;
            vc.area = matched.area;

            vc.datasheet_width_mm = a.datasheet_width_mm;
            vc.datasheet_height_mm = a.datasheet_height_mm;
            vc.datasheet_thickness_mm = a.datasheet_thickness_mm;
            vc.datasheet_source = a.datasheet_source;

            if (a.datasheet_width_mm > 0.0 && a.datasheet_height_mm > 0.0) {
                vc.nominal_width_mm = a.datasheet_width_mm;
                vc.nominal_height_mm = a.datasheet_height_mm;
            } else if (a.pad_box_w_mm > 0.0 && a.pad_box_h_mm > 0.0) {
                vc.nominal_width_mm = a.pad_box_w_mm;
                vc.nominal_height_mm = a.pad_box_h_mm;
            } else {
                vc.nominal_width_mm = matched.box.width / s;
                vc.nominal_height_mm = matched.box.height / s;
            }

            vc.original_label = matched.label;
            vc.confidence = std::max(matched.conf, 0.90f);
            vc.source_name = matched.source;
            vc.offset_error_px = realDist;
            vc.offset_error_mm = realDist / s;

            totalOffsetErrPx += realDist;
            offsetCount++;

            if (matched.label == a.assigned_class) {
                vc.status = AoiMatchStatus::MATCH_EXACT_CLASS;
                vc.notes = "Riconosciuto da DL (" + a.ref_des + "), classe confermata: " + a.assigned_class;
                res.summary.exact_matches++;
            } else {
                vc.status = AoiMatchStatus::CLASS_CORRECTED;
                vc.notes = "Classe corretta da '" + matched.label + "' a '" + a.assigned_class + "' tramite dati teorici AOI (" + a.ref_des + ")";
                res.summary.class_corrected++;
            }

            // Memorizza le dimensioni DL per questo codice componente univoco
            partNumberDlSizes[a.part_number].push_back(matched.box.size());

        } else {
            // Non riconosciuto da DL: verrà recuperato nella FASE 2
            a.matched = false;
            a.matched_detection_id = -1;
            a.match_distance_px = 0.0;
            a.match_distance_mm = 0.0;
            vc.status = AoiMatchStatus::RECOVERED_BY_AOI;
            vc.original_label = "NOT_DETECTED";
            vc.confidence = 0.85f;
            vc.center = nomPx;
            vc.offset_error_px = 0.0;
            vc.offset_error_mm = 0.0;
        }

        res.verified_components.push_back(vc);
    }

    // FASE 2: Recupero componenti non riconosciuti da DL tramite SOVRAPPOSIZIONE CV
    // REQUISITI UTENTE:
    // - "utilizza la sovrapposizione di tutte le foto e di tutti i componenti fatta in cv per recuperare i componenti non riconosciuti"
    // - "se un componente nel file AOI ha lo stesso codice componente significa che le dimensioni sono uguali"
    for (size_t i = 0; i < res.verified_components.size(); ++i) {
        auto& vc = res.verified_components[i];
        if (vc.status != AoiMatchStatus::RECOVERED_BY_AOI) continue;

        const auto& a = res.aoi_components[i];
        cv::Point2f nomPx = vc.nominal_center_px;
        float boardBaseRot = (transform.orientation_name == "rot180") ? 180.0f : 0.0f;
        double totalRot = a.rot_deg + boardBaseRot + transform.residual_angle_deg;
        double normImgAngle = std::fmod(totalRot, 180.0);
        if (normImgAngle < 0.0) normImgAngle += 180.0;

        // Stima delle dimensioni attese in pixel per il componente recuperato
        float expW_px = 18.0f, expH_px = 30.0f;
        if (a.datasheet_width_mm > 0.0 && a.datasheet_height_mm > 0.0) {
            expW_px = static_cast<float>(std::max(a.datasheet_width_mm, a.datasheet_height_mm) * s);
            expH_px = static_cast<float>(std::min(a.datasheet_width_mm, a.datasheet_height_mm) * s);
        } else {
            auto itDlSizes = partNumberDlSizes.find(a.part_number);
            if (itDlSizes != partNumberDlSizes.end() && !itDlSizes->second.empty()) {
                std::vector<float> dws, dhs;
                for (const auto& sz : itDlSizes->second) {
                    dws.push_back(std::max(sz.width, sz.height));
                    dhs.push_back(std::min(sz.width, sz.height));
                }
                std::sort(dws.begin(), dws.end());
                std::sort(dhs.begin(), dhs.end());
                expW_px = dws[dws.size() / 2];
                expH_px = dhs[dhs.size() / 2];
            } else {
                double w_mm = 1.6, h_mm = 0.8;
                getNominalOutlineDimensions(a.outline, w_mm, h_mm);
                expW_px = static_cast<float>(std::max(w_mm, h_mm) * s);
                expH_px = static_cast<float>(std::min(w_mm, h_mm) * s);
            }
        }

        // Cerca nella sovrapposizione CV il componente geometrico compatibile più vicino alla coordinata nominale
        int bestCvIdx = -1;
        double bestCvDist = 1e9;
        double cvSearchTol = tol_px * 1.1;

        for (size_t k = 0; k < cvPool.size(); ++k) {
            double dist = cv::norm(nomPx - cvPool[k].center);
            if (dist <= cvSearchTol) {
                float cMax = std::max(cvPool[k].rotated_box.size.width, cvPool[k].rotated_box.size.height);
                float cMin = std::min(cvPool[k].rotated_box.size.width, cvPool[k].rotated_box.size.height);
                // Filtra candidati palesemente incompatibili per dimensione
                if (cMax > expW_px * 2.2f || cMax < expW_px * 0.40f || cMin < expH_px * 0.35f) continue;
                if (dist < bestCvDist) {
                    bestCvDist = dist;
                    bestCvIdx = static_cast<int>(k);
                }
            }
        }

        if (bestCvIdx >= 0) {
            const auto& cvBest = cvPool[bestCvIdx];
            // Calcola la mediana dimensionale tra tutti i componenti CV sovrapposti nelle vicinanze
            std::vector<float> ws, hs;
            for (const auto& cvc : cvPool) {
                if (cv::norm(cvBest.center - cvc.center) < 8.0) {
                    ws.push_back(cvc.rotated_box.size.width);
                    hs.push_back(cvc.rotated_box.size.height);
                }
            }
            std::sort(ws.begin(), ws.end());
            std::sort(hs.begin(), hs.end());
            float medW = ws.empty() ? cvBest.rotated_box.size.width : ws[ws.size() / 2];
            float medH = hs.empty() ? cvBest.rotated_box.size.height : hs[hs.size() / 2];

            vc.center = cvBest.center;
            vc.shape = cvBest.shape;
            vc.radius = cvBest.radius;
            vc.rotated_box = cv::RotatedRect(cvBest.center, cv::Size2f(medW, medH), cvBest.rotated_box.angle);
            vc.box = vc.rotated_box.boundingRect2f();
            vc.area = vc.box.width * vc.box.height;

            vc.nominal_x_mm = a.x_mm;
            vc.nominal_y_mm = a.y_mm;
            vc.nominal_rot_deg = a.rot_deg;

            vc.datasheet_width_mm = a.datasheet_width_mm;
            vc.datasheet_height_mm = a.datasheet_height_mm;
            vc.datasheet_thickness_mm = a.datasheet_thickness_mm;
            vc.datasheet_source = a.datasheet_source;

            if (a.datasheet_width_mm > 0.0 && a.datasheet_height_mm > 0.0) {
                vc.nominal_width_mm = a.datasheet_width_mm;
                vc.nominal_height_mm = a.datasheet_height_mm;
            } else {
                vc.nominal_width_mm = vc.box.width / s;
                vc.nominal_height_mm = vc.box.height / s;
            }

            vc.source_name = "RECOVERED_CV_SUPERPOSITION";
            vc.notes = "Componente recuperato tramite sovrapposizione multi-foto CV (" + a.ref_des + ")";
            vc.offset_error_px = bestCvDist;
            vc.offset_error_mm = bestCvDist / s;

        } else {
            // Nessun candidato CV nelle vicinanze: recupero con dimensioni canoniche DL o datasheet
            float w_px = 18.0f, h_px = 30.0f;
            if (a.datasheet_width_mm > 0.0 && a.datasheet_height_mm > 0.0) {
                w_px = static_cast<float>(a.datasheet_width_mm * s);
                h_px = static_cast<float>(a.datasheet_height_mm * s);
            } else {
                auto itDlSizes = partNumberDlSizes.find(a.part_number);
                if (itDlSizes != partNumberDlSizes.end() && !itDlSizes->second.empty()) {
                    std::vector<float> dws, dhs;
                    for (const auto& sz : itDlSizes->second) {
                        dws.push_back(sz.width);
                        dhs.push_back(sz.height);
                    }
                    std::sort(dws.begin(), dws.end());
                    std::sort(dhs.begin(), dhs.end());
                    w_px = dws[dws.size() / 2];
                    h_px = dhs[dhs.size() / 2];
                } else {
                    double w_mm = 1.6, h_mm = 0.8;
                    getNominalOutlineDimensions(a.outline, w_mm, h_mm);
                    w_px = static_cast<float>(w_mm * s);
                    h_px = static_cast<float>(h_mm * s);
                }
            }

            vc.nominal_x_mm = a.x_mm;
            vc.nominal_y_mm = a.y_mm;
            vc.nominal_rot_deg = a.rot_deg;

            vc.datasheet_width_mm = a.datasheet_width_mm;
            vc.datasheet_height_mm = a.datasheet_height_mm;
            vc.datasheet_thickness_mm = a.datasheet_thickness_mm;
            vc.datasheet_source = a.datasheet_source;

            vc.center = nomPx;
            vc.shape = (a.outline.find("CHUUT") != std::string::npos) ? ShapeType::CIRCLE : ShapeType::RECTANGLE;
            vc.radius = (vc.shape == ShapeType::CIRCLE) ? std::max(w_px, h_px) * 0.5f : 0.0f;
            vc.rotated_box = cv::RotatedRect(nomPx, cv::Size2f(w_px, h_px), static_cast<float>(normImgAngle));
            vc.box = vc.rotated_box.boundingRect2f();
            vc.area = vc.box.width * vc.box.height;
            vc.nominal_width_mm = a.datasheet_width_mm > 0.0 ? a.datasheet_width_mm : (w_px / s);
            vc.nominal_height_mm = a.datasheet_height_mm > 0.0 ? a.datasheet_height_mm : (h_px / s);
            vc.source_name = "RECOVERED_AOI_NOMINAL";
            vc.notes = "Componente recuperato tramite coordinate teoriche AOI (" + a.ref_des + ")";
            vc.offset_error_px = 0.0;
            vc.offset_error_mm = 0.0;
        }

        res.summary.recovered_by_aoi++;
    }

    // FASE 3: Risoluzione sovrapposizioni ("non possono esserci componenti sovrapposti: dl non sbaglia mai, cv potrebbe sbagliare")
    resolveOverlapsWithDl(res.verified_components);

    // Aggiorna distribuzione classi
    for (const auto& vc : res.verified_components) {
        if (vc.status != AoiMatchStatus::UNMATCHED_DETECTION) {
            res.summary.class_distribution[vc.label]++;
        }
    }

    // FASE 4: Componenti extra senza corrispondenza AOI (NON verranno rappresentati nelle immagini)
    for (const auto& cand : pool) {
        if (!cand.used) {
            VerifiedPcbComponent vc;
            vc.id = nextId++;
            vc.ref_des = "N/A";
            vc.label = cand.label;
            vc.original_label = cand.label;
            vc.confidence = cand.conf;
            vc.status = AoiMatchStatus::UNMATCHED_DETECTION;
            vc.source_name = cand.source;
            vc.center = cand.center;
            vc.box = cand.box;
            vc.rotated_box = cand.rbox;
            vc.shape = cand.shape;
            vc.radius = cand.radius;
            vc.area = cand.area;
            vc.nominal_center_px = cand.center;
            vc.notes = "Rilevamento reale senza corrispondenza AOI (componente extra, non rappresentato nelle immagini)";

            res.summary.unmatched_extra_detections++;
            res.verified_components.push_back(vc);
        }
    }

    res.summary.total_verified_output = static_cast<int>(res.verified_components.size());
    res.summary.mean_offset_error_px = (offsetCount > 0) ? (totalOffsetErrPx / offsetCount) : 0.0;
    res.summary.mean_offset_error_mm = res.summary.mean_offset_error_px / s;
    res.success = true;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "[AOI] RIEPILOGO VERIFICA ARMONIZZATA (DL + SOVRAPPOSIZIONE CV):" << std::endl;
    std::cout << "  - Componenti nominali AOI:         " << res.summary.total_aoi_nominal << std::endl;
    std::cout << "  - Match esatti confermati (DL):    " << res.summary.exact_matches << std::endl;
    std::cout << "  - Classi errate corrette con AOI:  " << res.summary.class_corrected << std::endl;
    std::cout << "  - Totale associati su scheda:      " << (res.summary.exact_matches + res.summary.class_corrected)
              << " / " << res.summary.total_aoi_nominal << " ("
              << std::fixed << std::setprecision(1) << ((res.summary.exact_matches + res.summary.class_corrected) * 100.0 / res.summary.total_aoi_nominal) << "%)" << std::endl;
    std::cout << "  - Componenti mancanti recuperati:  " << res.summary.recovered_by_aoi << std::endl;
    std::cout << "  - Rilevamenti extra (non disegnati):" << res.summary.unmatched_extra_detections << std::endl;
    std::cout << "  - Scostamento medio reale/nominale:" << std::fixed << std::setprecision(2)
              << res.summary.mean_offset_error_px << " px (" << std::setprecision(3)
              << res.summary.mean_offset_error_mm << " mm)" << std::endl;
    std::cout << "========================================================" << std::endl;

    return res;
}

} // namespace pcb

