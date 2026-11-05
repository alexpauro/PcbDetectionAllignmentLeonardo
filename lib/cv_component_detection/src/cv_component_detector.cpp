#include "cv_component_detector.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;

namespace pcb {

void CvComponentDetector::computeAutoCannyThresholds(const cv::Mat& gray, double& lower, double& upper) const {
    // Calcola l'istogramma per trovare la mediana esatta dei pixel in scala di grigi
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange[] = { range };
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, histRange, true, false);

    int totalPixels = gray.rows * gray.cols;
    int halfPixels = totalPixels / 2;
    int cumulative = 0;
    double median = 128.0;

    for (int i = 0; i < 256; ++i) {
        cumulative += static_cast<int>(hist.at<float>(i));
        if (cumulative >= halfPixels) {
            median = static_cast<double>(i);
            break;
        }
    }

    lower = std::max(20.0, 0.66 * median);
    upper = std::min(240.0, 1.33 * median);
}

CvDetectionResult CvComponentDetector::detect(const cv::Mat& input_image,
                                             const CvDetectionConfig& config) {
    CvDetectionResult result;
    if (input_image.empty()) {
        std::cerr << "[CvComponentDetector] ERRORE: Immagine di input vuota!" << std::endl;
        return result;
    }

    int img_w = input_image.cols;
    int img_h = input_image.rows;
    float total_img_area = static_cast<float>(img_w * img_h);

    // =========================================================================
    // STEP 1: Pre-processing & Riduzione Rumore (Bilateral + CLAHE)
    // =========================================================================
    cv::Mat denoised;
    if (config.use_bilateral) {
        cv::bilateralFilter(input_image, denoised, config.bilateral_d,
                            config.bilateral_sigma_color, config.bilateral_sigma_space);
    } else {
        denoised = input_image.clone();
    }

    cv::Mat gray, hsv;
    cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(denoised, hsv, cv::COLOR_BGR2HSV);

    cv::Mat gray_clahe;
    if (config.use_clahe) {
        auto clahe = cv::createCLAHE(config.clahe_clip_limit,
                                     cv::Size(config.clahe_grid_size, config.clahe_grid_size));
        clahe->apply(gray, gray_clahe);
    } else {
        gray_clahe = gray.clone();
    }

    // =========================================================================
    // STEP 2: Filtro Colore Substrato PCB Verdognolo (Solder Mask)
    // =========================================================================
    cv::Mat green_substrate_mask = cv::Mat::zeros(input_image.size(), CV_8UC1);

    if (config.use_green_filter) {
        // Maschera HSV per il verde della maschera di saldatura
        cv::Mat hsv_green;
        cv::inRange(hsv,
                    cv::Scalar(config.h_min, config.s_min, config.v_min),
                    cv::Scalar(config.h_max, config.s_max, config.v_max),
                    hsv_green);

        // Calcolo dell'indice di verdura nello spazio BGR: (2*G - R - B) / (R + G + B + eps)
        std::vector<cv::Mat> bgr_channels;
        cv::split(denoised, bgr_channels);
        cv::Mat B_f, G_f, R_f;
        bgr_channels[0].convertTo(B_f, CV_32F);
        bgr_channels[1].convertTo(G_f, CV_32F);
        bgr_channels[2].convertTo(R_f, CV_32F);

        cv::Mat numerator = 2.0f * G_f - R_f - B_f;
        cv::Mat denominator = R_f + G_f + B_f + 1e-4f;
        cv::Mat green_ratio = numerator / denominator;

        cv::Mat bgr_green = cv::Mat::zeros(input_image.size(), CV_8UC1);
        bgr_green.setTo(255, green_ratio > config.green_ratio_thresh);

        // Il substrato verde è la combinazione di coerenza HSV e indice BGR
        cv::bitwise_and(hsv_green, bgr_green, green_substrate_mask);
    }

    result.green_substrate_mask = green_substrate_mask.clone();

    // Ciò che si discosta dal colore verdognolo del PCB è potenziale componente/pad
    cv::Mat non_substrate;
    cv::bitwise_not(green_substrate_mask, non_substrate);

    // =========================================================================
    // STEP 2B: Banco Filtri Colore Differenziati per Componenti (Multi-Color)
    // =========================================================================
    cv::Mat multi_color_mask = cv::Mat::zeros(input_image.size(), CV_8UC1);

    if (config.use_multi_color_filters) {
        // a) Chip neri IC / SMD resistor scuri (bassa luminosità o Lab L molto basso)
        if (config.detect_dark_chips) {
            cv::Mat dark_mask;
            cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 60), dark_mask);
            cv::bitwise_or(multi_color_mask, dark_mask, multi_color_mask);
        }
        // b) Parti metalliche lucide, stagno, pin argentati e quarzi metallici
        if (config.detect_silver_metal) {
            cv::Mat silver_mask;
            cv::inRange(hsv, cv::Scalar(0, 0, 160), cv::Scalar(180, 50, 255), silver_mask);
            cv::bitwise_or(multi_color_mask, silver_mask, multi_color_mask);
        }
        // c) Condensatori ceramici (tan, marroncino, ocra, giallo)
        if (config.detect_ceramic_capacitors) {
            cv::Mat ceramic_mask;
            cv::inRange(hsv, cv::Scalar(8, 55, 60), cv::Scalar(35, 255, 220), ceramic_mask);
            cv::bitwise_or(multi_color_mask, ceramic_mask, multi_color_mask);
        }
        // d) Piazzole dorate e rame (testpoint, ENIG, connettori)
        if (config.detect_gold_pads) {
            cv::Mat gold_mask;
            cv::inRange(hsv, cv::Scalar(15, 60, 75), cv::Scalar(45, 255, 255), gold_mask);
            cv::bitwise_or(multi_color_mask, gold_mask, multi_color_mask);
        }
        // e) Componenti blu / azzurri (condensatori a film, resistori blu di precisione)
        if (config.detect_blue_components) {
            cv::Mat blue_mask;
            cv::inRange(hsv, cv::Scalar(90, 45, 35), cv::Scalar(135, 255, 255), blue_mask);
            cv::bitwise_or(multi_color_mask, blue_mask, multi_color_mask);
        }
        // f) Componenti rossi / viola (LED, tantalio, diodi)
        if (config.detect_red_components) {
            cv::Mat red1, red2, red_mask;
            cv::inRange(hsv, cv::Scalar(0, 50, 40), cv::Scalar(12, 255, 255), red1);
            cv::inRange(hsv, cv::Scalar(165, 50, 40), cv::Scalar(180, 255, 255), red2);
            cv::bitwise_or(red1, red2, red_mask);
            cv::bitwise_or(multi_color_mask, red_mask, multi_color_mask);
        }
        // g) Componenti ceramici bianchi e connettori chiari
        if (config.detect_white_components) {
            cv::Mat white_mask;
            cv::inRange(hsv, cv::Scalar(0, 0, 195), cv::Scalar(180, 35, 255), white_mask);
            cv::bitwise_or(multi_color_mask, white_mask, multi_color_mask);
        }

        // I componenti colorati non devono appartenere al substrato verde
        cv::bitwise_and(multi_color_mask, non_substrate, multi_color_mask);
    }
    result.multi_color_mask = multi_color_mask.clone();

    // =========================================================================
    // STEP 3: Filtro Bordi e Gradienti Avanzati (Canny, Scharr, Morph HATS)
    // =========================================================================
    cv::Mat edges = cv::Mat::zeros(input_image.size(), CV_8UC1);
    if (config.use_canny) {
        double lower_th = config.canny_thresh1;
        double upper_th = config.canny_thresh2;
        if (config.auto_canny) {
            computeAutoCannyThresholds(gray_clahe, lower_th, upper_th);
        }
        cv::Canny(gray_clahe, edges, lower_th, upper_th);
    }

    // Gradiente cromatico multi-canale (rileva transizioni di colore nette anche a parità di luminanza)
    cv::Mat color_gradient = cv::Mat::zeros(input_image.size(), CV_8UC1);
    if (config.use_color_gradient) {
        cv::Mat grad_max = cv::Mat::zeros(input_image.size(), CV_32F);
        std::vector<cv::Mat> channels;
        cv::split(denoised, channels);

        for (int ch = 0; ch < 3; ++ch) {
            cv::Mat gx, gy, mag;
            if (config.use_scharr_gradient) {
                cv::Scharr(channels[ch], gx, CV_32F, 1, 0);
                cv::Scharr(channels[ch], gy, CV_32F, 0, 1);
            } else {
                cv::Sobel(channels[ch], gx, CV_32F, 1, 0, 3);
                cv::Sobel(channels[ch], gy, CV_32F, 0, 1, 3);
            }
            cv::magnitude(gx, gy, mag);
            cv::max(grad_max, mag, grad_max);
        }

        double minVal, maxVal;
        cv::minMaxLoc(grad_max, &minVal, &maxVal);
        if (maxVal > 0) {
            grad_max.convertTo(color_gradient, CV_8UC1, 255.0 / maxVal);
        }
        cv::threshold(color_gradient, color_gradient, 30, 255, cv::THRESH_BINARY);
    }
    result.gradient_mask = color_gradient.clone();

    // Top-Hat (evidenzia saldature a stagno e pin argentati/metallici lucidi)
    // Black-Hat (evidenzia chip IC neri e componenti ceramici scuri)
    cv::Mat tophat_bin = cv::Mat::zeros(input_image.size(), CV_8UC1);
    cv::Mat blackhat_bin = cv::Mat::zeros(input_image.size(), CV_8UC1);
    if (config.use_morph_hats) {
        int ksize = std::max(3, config.hat_kernel_size);
        cv::Mat hat_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));

        cv::Mat tophat, blackhat;
        cv::morphologyEx(gray_clahe, tophat, cv::MORPH_TOPHAT, hat_kernel);
        cv::morphologyEx(gray_clahe, blackhat, cv::MORPH_BLACKHAT, hat_kernel);

        cv::threshold(tophat, tophat_bin, config.hat_threshold, 255, cv::THRESH_BINARY);
        cv::threshold(blackhat, blackhat_bin, config.hat_threshold, 255, cv::THRESH_BINARY);
    }

    // Maschera combinata di tutti i bordi e gradienti
    cv::Mat combined_edges;
    cv::bitwise_or(edges, tophat_bin, combined_edges);
    cv::bitwise_or(combined_edges, blackhat_bin, combined_edges);
    if (config.use_color_gradient) {
        cv::bitwise_or(combined_edges, color_gradient, combined_edges);
    }
    result.edges_mask = combined_edges.clone();

    // =========================================================================
    // STEP 4: Fusione Prove (Multi-Colore + Bordi) & Pulizia Piste
    // =========================================================================
    // Combiniamo le evidenze fisiche dei componenti (colore differenziato e bordi/gradienti).
    // Invece di fare l'OR con non_substrate (che rendeva quasi tutta l'immagine bianca
    // e fondeva tutti i componenti in un unico mega-blocco), utilizziamo il substrato verde
    // come filtro di esclusione (AND NOT) per preservare la netta separazione tra componenti.
    cv::Mat candidate_mask;
    if (config.use_multi_color_filters) {
        candidate_mask = multi_color_mask.clone();
        cv::bitwise_or(candidate_mask, combined_edges, candidate_mask);
    } else {
        candidate_mask = combined_edges.clone();
    }

    if (config.use_green_filter) {
        cv::bitwise_and(candidate_mask, non_substrate, candidate_mask);
    }

    // Azzeriamo i pixel lungo il bordo perimetrale estremo dell'immagine (3 px)
    // per impedire che il contorno dell'inquadratura racchiuda tutti i componenti
    const int border_margin = 3;
    candidate_mask.rowRange(0, border_margin).setTo(0);
    candidate_mask.rowRange(img_h - border_margin, img_h).setTo(0);
    candidate_mask.colRange(0, border_margin).setTo(0);
    candidate_mask.colRange(img_w - border_margin, img_w).setTo(0);

    // Operazione di Apertura (MORPH_OPEN):
    // Rimuove piste di rame sottili (1-2 px) e rumore isolato mantenendo intatti i componenti solidi
    cv::Mat open_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(config.open_kernel_size, config.open_kernel_size));
    cv::Mat opened_mask;
    cv::morphologyEx(candidate_mask, opened_mask, cv::MORPH_OPEN, open_kernel);

    // Operazione di Chiusura (MORPH_CLOSE):
    // Raggruppa i pin e le saldature al corpo principale del componente senza fondere componenti vicini
    cv::Mat close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(config.close_kernel_size, config.close_kernel_size));
    cv::Mat closed_mask;
    cv::morphologyEx(opened_mask, closed_mask, cv::MORPH_CLOSE, close_kernel);

    result.components_mask = closed_mask.clone();

    // =========================================================================
    // STEP 5: Estrazione Contorni e Fit Geometrico (Rettangolo Ruotato vs Cerchio)
    // =========================================================================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(closed_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int comp_id_counter = 1;
    for (const auto& contour : contours) {
        float area = static_cast<float>(cv::contourArea(contour));
        if (area < config.min_area || area > (config.max_area_ratio * total_img_area)) {
            continue; // Ignora rumore microscopico o l'intero telaio della scheda
        }

        float perimeter = static_cast<float>(cv::arcLength(contour, true));
        if (perimeter < 5.0f) {
            continue;
        }

        // Calcolo del rettangolo minimo orientato circoscritto (RotatedRect)
        cv::RotatedRect rrect = cv::minAreaRect(contour);
        float rw = rrect.size.width;
        float rh = rrect.size.height;
        float min_side = std::min(rw, rh);
        float max_side = std::max(rw, rh);

        if (min_side < config.min_dimension) {
            continue; // Ignora rumore lineare 1D
        }

        // Filtra piste sottili allungate di rame che non sono componenti
        if (config.filter_thin_traces) {
            float trace_ar = max_side / (min_side + 1e-4f);
            if (trace_ar > config.max_trace_aspect_ratio && min_side < config.max_trace_thickness) {
                continue;
            }
        }

        // Calcolo del cerchio minimo circoscritto
        cv::Point2f circle_center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contour, circle_center, radius);

        // Metriche geometriche per distinguere cerchio vs rettangolo
        float circularity = (4.0f * static_cast<float>(CV_PI) * area) / (perimeter * perimeter + 1e-5f);
        float aspect_ratio = std::min(rw, rh) / (std::max(rw, rh) + 1e-5f);
        float circle_area = static_cast<float>(CV_PI) * (radius * radius);
        float fill_circle = area / (circle_area + 1e-5f);
        float rect_area = rw * rh;
        float fill_rect = area / (rect_area + 1e-5f);

        CvComponent comp;
        comp.id = comp_id_counter++;
        comp.area = area;
        comp.perimeter = perimeter;
        comp.circularity = circularity;
        comp.aspect_ratio = aspect_ratio;
        comp.rotated_box = rrect;

        // Criterio di decisione: la figura è più vicina a un CERCHIO o a un RETTANGOLO?
        bool is_circle = (circularity >= config.circularity_thresh &&
                          aspect_ratio >= config.circle_aspect_ratio_thresh &&
                          fill_circle >= config.circle_fill_thresh &&
                          fill_circle >= fill_rect * 0.70f);

        if (is_circle) {
            comp.shape = ShapeType::CIRCLE;
            comp.center = circle_center;
            // Raggio aderente: media tra raggio circoscritto e raggio equivalente dall'area
            float r_equiv = std::sqrt(area / static_cast<float>(CV_PI));
            comp.radius = 0.5f * (radius + r_equiv);
            result.num_circles++;
        } else {
            comp.shape = ShapeType::RECTANGLE;
            comp.center = rrect.center;
            comp.radius = 0.0f;
            result.num_rectangles++;
        }

        result.components.push_back(comp);
    }

    // =========================================================================
    // STEP 6: Generazione Immagine Annotata
    // =========================================================================
    result.annotated_image = input_image.clone();
    drawComponents(result.annotated_image, result.components);

    return result;
}

void CvComponentDetector::drawComponents(cv::Mat& output,
                                        const std::vector<CvComponent>& components) const {
    const cv::Scalar color_rect(0, 255, 0);     // Verde brillante per rettangoli
    const cv::Scalar color_circ(0, 165, 255);   // Arancione/Oro per cerchi
    const cv::Scalar color_center(0, 0, 255);   // Rosso per punto centrale
    const cv::Scalar color_text(255, 255, 255); // Bianco per etichette
    const cv::Scalar color_box_bg(20, 20, 20);  // Sfondo scuro per leggibilità testo

    for (const auto& comp : components) {
        if (comp.shape == ShapeType::CIRCLE) {
            // Disegna il cerchio aderente alle dimensioni reali
            int rad = std::max(1, static_cast<int>(std::round(comp.radius)));
            cv::circle(output, comp.center, rad, color_circ, 2, cv::LINE_AA);
            cv::circle(output, comp.center, 2, color_center, -1, cv::LINE_AA);

            // Etichetta informativa: ID e Raggio
            std::string label = "#" + std::to_string(comp.id) + " (R:" +
                                std::to_string(static_cast<int>(std::round(comp.radius))) + ")";
            int baseLine = 0;
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
            cv::Point textOrigin(static_cast<int>(comp.center.x - textSize.width / 2),
                                 static_cast<int>(comp.center.y - rad - 4));
            if (textOrigin.y < 12) textOrigin.y = static_cast<int>(comp.center.y + rad + 14);

            cv::Rect bgRect(textOrigin.x - 2, textOrigin.y - textSize.height - 2,
                            textSize.width + 4, textSize.height + 4);
            bgRect &= cv::Rect(0, 0, output.cols, output.rows);
            if (bgRect.width > 0 && bgRect.height > 0) {
                cv::rectangle(output, bgRect, color_box_bg, cv::FILLED);
                cv::putText(output, label, textOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            color_circ, 1, cv::LINE_AA);
            }
        } else {
            // Disegna il rettangolo ruotato aderente
            cv::Point2f vertices[4];
            comp.rotated_box.points(vertices);

            for (int j = 0; j < 4; ++j) {
                cv::line(output, vertices[j], vertices[(j + 1) % 4], color_rect, 2, cv::LINE_AA);
            }
            cv::circle(output, comp.center, 2, color_center, -1, cv::LINE_AA);

            // Calcolo orientamento e asse principale per visualizzare la rotazione precisa
            float angle_deg = comp.rotated_box.angle;
            int rw = static_cast<int>(std::round(comp.rotated_box.size.width));
            int rh = static_cast<int>(std::round(comp.rotated_box.size.height));

            // Etichetta informativa: ID, dimensioni e rotazione
            std::string label = "#" + std::to_string(comp.id) + " [" +
                                std::to_string(rw) + "x" + std::to_string(rh) + "] " +
                                std::to_string(static_cast<int>(std::round(angle_deg))) + "deg";

            int baseLine = 0;
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);
            cv::Point textOrigin(static_cast<int>(comp.center.x - textSize.width / 2),
                                 static_cast<int>(comp.center.y - std::max(rw, rh) / 2 - 4));
            if (textOrigin.y < 12) textOrigin.y = static_cast<int>(comp.center.y + std::max(rw, rh) / 2 + 14);

            cv::Rect bgRect(textOrigin.x - 2, textOrigin.y - textSize.height - 2,
                            textSize.width + 4, textSize.height + 4);
            bgRect &= cv::Rect(0, 0, output.cols, output.rows);
            if (bgRect.width > 0 && bgRect.height > 0) {
                cv::rectangle(output, bgRect, color_box_bg, cv::FILLED);
                cv::putText(output, label, textOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            color_rect, 1, cv::LINE_AA);
            }
        }
    }

    // Banner superiore con statistiche riassuntive
    cv::Rect banner_rect(0, 0, output.cols, 32);
    cv::Mat banner_overlay = output(banner_rect);
    cv::Mat color_bar(banner_rect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(color_bar, 0.85, banner_overlay, 0.15, 0.0, banner_overlay);

    std::string summary_text = "Rilevamento CV Componenti: Totale=" + std::to_string(components.size()) +
                               " | Rettangoli Ruotati=" + std::to_string(std::count_if(components.begin(), components.end(),
                                    [](const CvComponent& c){ return c.shape == ShapeType::RECTANGLE; })) +
                               " | Cerchi=" + std::to_string(std::count_if(components.begin(), components.end(),
                                    [](const CvComponent& c){ return c.shape == ShapeType::CIRCLE; }));

    cv::putText(output, summary_text, cv::Point(12, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, color_text, 1, cv::LINE_AA);
}

void CvComponentDetector::displayResult(const std::string& window_title,
                                       const CvDetectionResult& result,
                                       int wait_ms) {
    if (result.annotated_image.empty()) {
        std::cerr << "[CvComponentDetector] ERRORE: Immagine annotata vuota per visualizzazione." << std::endl;
        return;
    }

    try {
        cv::namedWindow(window_title, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_title, 1280, 960);
        cv::imshow(window_title, result.annotated_image);

        std::cout << "\n========================================================" << std::endl;
        std::cout << "  FINESTRA ATTIVA: " << window_title << std::endl;
        std::cout << "  Totale componenti individuati: " << result.components.size() << std::endl;
        std::cout << "    - Rettangoli orientati: " << result.num_rectangles << std::endl;
        std::cout << "    - Cerchi: " << result.num_circles << std::endl;
        std::cout << "  Premi un tasto qualsiasi sulla finestra per continuare..." << std::endl;
        std::cout << "========================================================" << std::endl;

        cv::waitKey(wait_ms);
        cv::destroyWindow(window_title);
    } catch (const cv::Exception& e) {
        std::cout << "[CvComponentDetector] Visualizzazione grafica saltata (ambiente senza display grafico): "
                  << e.what() << std::endl;
    }
}

void CvComponentDetector::saveResults(const std::string& output_dir,
                                     const std::string& image_stem,
                                     const cv::Mat& original_image,
                                     const CvDetectionResult& result,
                                     bool save_debug_masks) {
    (void)original_image;
    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[CvComponentDetector] ERRORE creazione cartella: " << output_dir
                  << " (" << ec.message() << ")" << std::endl;
        return;
    }

    // 1. Salvataggio immagine principale annotata
    std::string annotated_path = output_dir + "/" + image_stem + "_components_cv.jpg";
    cv::imwrite(annotated_path, result.annotated_image);
    std::cout << "  [OK] Immagine annotata salvata in: " << annotated_path << std::endl;

    // 2. Salvataggio opzionale maschere intermedie per ispezione filtri
    if (save_debug_masks) {
        if (!result.green_substrate_mask.empty()) {
            cv::imwrite(output_dir + "/" + image_stem + "_mask_green_substrate.png", result.green_substrate_mask);
        }
        if (!result.multi_color_mask.empty()) {
            cv::imwrite(output_dir + "/" + image_stem + "_mask_multi_color.png", result.multi_color_mask);
        }
        if (!result.gradient_mask.empty()) {
            cv::imwrite(output_dir + "/" + image_stem + "_mask_gradient.png", result.gradient_mask);
        }
        if (!result.edges_mask.empty()) {
            cv::imwrite(output_dir + "/" + image_stem + "_mask_edges.png", result.edges_mask);
        }
        if (!result.components_mask.empty()) {
            cv::imwrite(output_dir + "/" + image_stem + "_mask_components.png", result.components_mask);
        }
    }

    // 3. Salvataggio dettagli componenti in formato CSV
    std::string csv_path = output_dir + "/" + image_stem + "_components_cv.csv";
    std::ofstream csv(csv_path);
    if (csv.is_open()) {
        csv << "id,shape,center_x,center_y,width,height,angle_deg,radius,area,circularity\n";
        for (const auto& comp : result.components) {
            std::string shape_str = (comp.shape == ShapeType::CIRCLE) ? "circle" : "rectangle";
            float w = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.size.width : comp.radius * 2.0f;
            float h = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.size.height : comp.radius * 2.0f;
            float angle = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.angle : 0.0f;

            csv << comp.id << ","
                << shape_str << ","
                << std::fixed << std::setprecision(2) << comp.center.x << ","
                << comp.center.y << ","
                << w << ","
                << h << ","
                << angle << ","
                << comp.radius << ","
                << comp.area << ","
                << comp.circularity << "\n";
        }
        csv.close();
        std::cout << "  [OK] Coordinate componenti CSV salvate in: " << csv_path << std::endl;
    }

    // 4. Salvataggio riepilogo in formato JSON
    std::string json_path = output_dir + "/" + image_stem + "_components_cv.json";
    std::ofstream json(json_path);
    if (json.is_open()) {
        json << "{\n";
        json << "  \"image\": \"" << image_stem << "\",\n";
        json << "  \"total_components\": " << result.components.size() << ",\n";
        json << "  \"rectangles\": " << result.num_rectangles << ",\n";
        json << "  \"circles\": " << result.num_circles << ",\n";
        json << "  \"components\": [\n";

        for (size_t i = 0; i < result.components.size(); ++i) {
            const auto& comp = result.components[i];
            std::string shape_str = (comp.shape == ShapeType::CIRCLE) ? "circle" : "rectangle";
            float w = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.size.width : comp.radius * 2.0f;
            float h = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.size.height : comp.radius * 2.0f;
            float angle = (comp.shape == ShapeType::RECTANGLE) ? comp.rotated_box.angle : 0.0f;

            json << "    {\n";
            json << "      \"id\": " << comp.id << ",\n";
            json << "      \"shape\": \"" << shape_str << "\",\n";
            json << "      \"center\": [" << std::fixed << std::setprecision(2) << comp.center.x << ", " << comp.center.y << "],\n";
            json << "      \"dimensions\": [" << w << ", " << h << "],\n";
            json << "      \"angle_deg\": " << angle << ",\n";
            json << "      \"radius\": " << comp.radius << ",\n";
            json << "      \"area\": " << comp.area << ",\n";
            json << "      \"circularity\": " << comp.circularity << "\n";
            json << "    }" << (i + 1 < result.components.size() ? "," : "") << "\n";
        }

        json << "  ]\n";
        json << "}\n";
        json.close();
        std::cout << "  [OK] Riepilogo JSON salvato in: " << json_path << std::endl;
    }
}

} // namespace pcb
