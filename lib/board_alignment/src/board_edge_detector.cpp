#include "board_edge_detector.hpp"
#include <opencv2/photo.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace pcb {

BoardEdgeDetector::BoardEdgeDetector(const BoardEdgeConfig& config)
    : config_(config) {
}

std::vector<cv::Point2f> BoardEdgeDetector::orderCorners(const std::vector<cv::Point2f>& pts) {
    if (pts.size() != 4) {
        return pts;
    }

    std::vector<cv::Point2f> ordered(4);
    std::vector<float> sum(4), diff(4);
    for (size_t i = 0; i < 4; ++i) {
        sum[i] = pts[i].x + pts[i].y;
        diff[i] = pts[i].y - pts[i].x;
    }

    size_t tlIdx = std::distance(sum.begin(), std::min_element(sum.begin(), sum.end()));
    size_t brIdx = std::distance(sum.begin(), std::max_element(sum.begin(), sum.end()));
    size_t trIdx = std::distance(diff.begin(), std::min_element(diff.begin(), diff.end()));
    size_t blIdx = std::distance(diff.begin(), std::max_element(diff.begin(), diff.end()));

    ordered[0] = pts[tlIdx]; // Top-Left
    ordered[1] = pts[trIdx]; // Top-Right
    ordered[2] = pts[brIdx]; // Bottom-Right
    ordered[3] = pts[blIdx]; // Bottom-Left

    return ordered;
}

cv::Mat BoardEdgeDetector::combineMultiExposure(const std::vector<cv::Mat>& images) const {
    if (images.empty()) return cv::Mat();
    if (images.size() == 1) return images[0].clone();

    if (config_.use_mertens_fusion) {
        try {
            cv::Ptr<cv::MergeMertens> merger = cv::createMergeMertens(1.0f, 1.0f, 1.0f);
            cv::Mat hdrFusionF;
            merger->process(images, hdrFusionF);
            cv::Mat fusedImg;
            hdrFusionF.convertTo(fusedImg, CV_8UC3, 255.0);
            return fusedImg;
        } catch (const cv::Exception& e) {
            std::cerr << "Attenzione: MergeMertens fallito (" << e.what() 
                      << "), utilizzo combinazione media pesata." << std::endl;
        }
    }

    cv::Mat accum = cv::Mat::zeros(images[0].size(), CV_32FC3);
    for (const auto& img : images) {
        cv::Mat fimg;
        img.convertTo(fimg, CV_32FC3);
        accum += fimg;
    }
    accum /= static_cast<float>(images.size());
    cv::Mat result;
    accum.convertTo(result, CV_8UC3);
    return result;
}

cv::Mat BoardEdgeDetector::computeSubstrateMask(const cv::Mat& fusedImg) const {
    cv::Mat hsv;
    cv::cvtColor(fusedImg, hsv, cv::COLOR_BGR2HSV);

    // Segmentazione mirata del substrato verdognolo:
    // Esclude lo sfondo metallico/bianco del supporto di test (dove R, G, B > 170)
    cv::Mat greenMask = cv::Mat::zeros(fusedImg.size(), CV_8UC1);
    for (int y = 0; y < fusedImg.rows; ++y) {
        const cv::Vec3b* bgr = fusedImg.ptr<cv::Vec3b>(y);
        const cv::Vec3b* h = hsv.ptr<cv::Vec3b>(y);
        uchar* out = greenMask.ptr<uchar>(y);
        for (int x = 0; x < fusedImg.cols; ++x) {
            int b = bgr[x][0], g = bgr[x][1], r = bgr[x][2];
            bool isGreenBGR = (g > r + 8 && g > b + 8 && r < 165 && b < 165);
            bool isGreenHSV = (h[x][0] >= 32 && h[x][0] <= 85 && h[x][1] >= 35 && h[x][2] >= 25);
            if (isGreenBGR || isGreenHSV) {
                out[x] = 255;
            }
        }
    }

    return greenMask;
}

cv::Mat BoardEdgeDetector::unifyDisconnectedEdges(const cv::Mat& greenMask) const {
    // Unione dei bordi non collegati e interrotti da componenti:
    // 1. Chiusura orizzontale ampia (collega bordi orizzontali interrotti da connettori o fori)
    // 2. Chiusura verticale ampia (collega bordi verticali interrotti)
    cv::Mat closedH, closedV, closed;
    cv::Mat kH = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(75, 5));
    cv::Mat kV = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 75));
    cv::morphologyEx(greenMask, closedH, cv::MORPH_CLOSE, kH);
    cv::morphologyEx(greenMask, closedV, cv::MORPH_CLOSE, kV);
    cv::bitwise_or(closedH, closedV, closed);

    // 3. Chiusura quadrata per saldare il corpo della scheda
    cv::Mat kSquare = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(31, 31));
    cv::morphologyEx(closed, closed, cv::MORPH_CLOSE, kSquare);

    // Azzeriamo cornice perimetrale minima di 3 pixel per evitare contatto con i bordi del sensore
    const int pad = 4;
    closed.rowRange(0, pad).setTo(0);
    closed.rowRange(closed.rows - pad, closed.rows).setTo(0);
    closed.colRange(0, pad).setTo(0);
    closed.colRange(closed.cols - pad, closed.cols).setTo(0);

    return closed;
}

std::vector<DetectedBoardEdge> BoardEdgeDetector::detectCandidates(const std::vector<cv::Mat>& images) const {
    std::vector<DetectedBoardEdge> candidates;
    if (images.empty()) return candidates;

    cv::Mat fused = combineMultiExposure(images);
    cv::Mat greenMask = computeSubstrateMask(fused);
    cv::Mat unifiedMask = unifyDisconnectedEdges(greenMask);

    double totalArea = static_cast<double>(fused.cols * fused.rows);

    auto addCandidate = [&](const std::vector<cv::Point2f>& pts, const std::string& method, double solidity) {
        if (pts.size() < 3) return;

        cv::Rect bb = cv::boundingRect(pts);
        cv::RotatedRect rr = cv::minAreaRect(pts);

        double rw = rr.size.width;
        double rh = rr.size.height;
        if (rw <= 20.0 || rh <= 20.0) return;

        double area = cv::contourArea(pts);
        double rectArea = rw * rh;
        if (rectArea > (config_.max_board_area_ratio * totalArea) || rectArea < (config_.min_board_area_ratio * totalArea)) {
            return;
        }

        DetectedBoardEdge edge;
        edge.valid = true;
        edge.method_tag = method;
        edge.area_px = (area > 1.0) ? area : rectArea * solidity;
        edge.solidity = solidity;
        edge.rotated_rect_px = rr;
        edge.bounding_box_px = bb;
        edge.width_px = std::max(rw, rh);
        edge.height_px = std::min(rw, rh);
        edge.aspect_ratio = (edge.height_px > 1e-4) ? (edge.width_px / edge.height_px) : 1.0;

        // Margini di vuoto/sfondo rispetto ai 4 lati dell'immagine
        edge.margin_top = bb.y;
        edge.margin_bottom = fused.rows - (bb.y + bb.height);
        edge.margin_left = bb.x;
        edge.margin_right = fused.cols - (bb.x + bb.width);

        cv::Point2f cpts[4];
        rr.points(cpts);
        std::vector<cv::Point2f> cornerVec(cpts, cpts + 4);
        edge.corners_px = orderCorners(cornerVec);
        edge.contour_px = pts;

        candidates.push_back(edge);
    };

    // -------------------------------------------------------------------------
    // Metodo 1: Profilo di densità del substrato verdognolo (Riconoscimento margini vuoti)
    // -------------------------------------------------------------------------
    {
        std::vector<int> rowCount(fused.rows, 0);
        std::vector<int> colCount(fused.cols, 0);
        for (int y = 0; y < fused.rows; ++y) {
            for (int x = 0; x < fused.cols; ++x) {
                if (greenMask.at<uchar>(y, x) > 0) {
                    rowCount[y]++;
                    colCount[x]++;
                }
            }
        }

        int threshRow = static_cast<int>(fused.cols * 0.05);
        int threshCol = static_cast<int>(fused.rows * 0.05);

        int topY = 0;
        while (topY < fused.rows && rowCount[topY] < threshRow) topY++;
        int bottomY = fused.rows - 1;
        while (bottomY > 0 && rowCount[bottomY] < threshRow) bottomY--;

        int leftX = 0;
        while (leftX < fused.cols && colCount[leftX] < threshCol) leftX++;
        int rightX = fused.cols - 1;
        while (rightX > 0 && colCount[rightX] < threshCol) rightX--;

        if (rightX > leftX + 100 && bottomY > topY + 100) {
            std::vector<cv::Point2f> rectPts = {
                cv::Point2f(static_cast<float>(leftX), static_cast<float>(topY)),
                cv::Point2f(static_cast<float>(rightX), static_cast<float>(topY)),
                cv::Point2f(static_cast<float>(rightX), static_cast<float>(bottomY)),
                cv::Point2f(static_cast<float>(leftX), static_cast<float>(bottomY))
            };
            addCandidate(rectPts, "density_profile_green", 0.98);
        }
    }

    // -------------------------------------------------------------------------
    // Metodo 2: Contorni e Convex Hull della maschera unificata (Unione bordi spezzati)
    // -------------------------------------------------------------------------
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(unifiedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& c : contours) {
            if (cv::contourArea(c) < (config_.min_board_area_ratio * totalArea)) continue;

            // Convex Hull per colmare qualsiasi rientranza o componente coprente
            std::vector<cv::Point> hull;
            cv::convexHull(c, hull);

            std::vector<cv::Point2f> hullF;
            for (const auto& pt : hull) hullF.push_back(cv::Point2f(pt.x, pt.y));

            double area = cv::contourArea(hullF);
            cv::RotatedRect rr = cv::minAreaRect(hullF);
            double rectArea = rr.size.width * rr.size.height;
            double solidity = (rectArea > 1.0) ? (area / rectArea) : 1.0;

            addCandidate(hullF, "unified_mask_convex_hull", solidity);
        }
    }

    // -------------------------------------------------------------------------
    // Metodo 3: Binarizzazione Otsu con Chiusura Morfologica Direzionale
    // -------------------------------------------------------------------------
    {
        cv::Mat gray, binOtsu;
        cv::cvtColor(fused, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
        cv::threshold(gray, binOtsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        cv::Mat kH = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(51, 5));
        cv::Mat kV = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 51));
        cv::morphologyEx(binOtsu, binOtsu, cv::MORPH_CLOSE, kH);
        cv::morphologyEx(binOtsu, binOtsu, cv::MORPH_CLOSE, kV);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binOtsu, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& c : contours) {
            if (cv::contourArea(c) < (config_.min_board_area_ratio * totalArea)) continue;
            std::vector<cv::Point> hull;
            cv::convexHull(c, hull);
            std::vector<cv::Point2f> hullF;
            for (const auto& pt : hull) hullF.push_back(cv::Point2f(pt.x, pt.y));
            addCandidate(hullF, "otsu_directional_closing", 0.92);
        }
    }

    return candidates;
}

DetectedBoardEdge BoardEdgeDetector::selectBestCandidate(
    const std::vector<DetectedBoardEdge>& candidates,
    const GerberBoardOutline& gerberOutline) const {

    if (candidates.empty()) {
        return DetectedBoardEdge();
    }

    double targetAR = gerberOutline.valid ? gerberOutline.aspect_ratio : 1.366;
    DetectedBoardEdge bestCandidate;
    double minPenalty = 1e9;

    std::cout << "\nValutazione candidati bordo scheda nelle foto (Target AR Gerber = "
              << std::fixed << std::setprecision(3) << targetAR << "):" << std::endl;

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        double diffAR = std::abs(c.aspect_ratio - targetAR) / targetAR;

        // Penalità basata su:
        // 1. Scostamento dall'aspect ratio del disegno Gerber (peso forte)
        // 2. Solidità geometrica
        // 3. Presenza di margini vuoti attorno (penalizza se tocca i bordi estremi del frame)
        double emptyMarginBonus = 0.0;
        if (c.margin_top > 5 && c.margin_bottom > 5 && (c.margin_left > 5 || c.margin_right > 5)) {
            emptyMarginBonus = -15.0; // Bonus per candidato con vuoto rilevato attorno alla scheda
        }

        double penalty = diffAR * 100.0 + (1.0 - c.solidity) * 20.0 + emptyMarginBonus;

        std::cout << "  - Candidato #" << (i + 1) << " [" << c.method_tag << "]: "
                  << c.width_px << "x" << c.height_px << " px, AR=" << std::setprecision(3) << c.aspect_ratio
                  << " (diff=" << std::setprecision(1) << (diffAR * 100.0) << "%), "
                  << "Vuoto: [T:" << c.margin_top << ", B:" << c.margin_bottom 
                  << ", L:" << c.margin_left << ", R:" << c.margin_right << " px] "
                  << "-> Penalità=" << penalty << std::endl;

        if (penalty < minPenalty) {
            minPenalty = penalty;
            bestCandidate = c;
            bestCandidate.score = 1.0 / (1.0 + diffAR);
        }
    }

    std::cout << "\n>>> BORDO SCHEDA ASSOCIATO (MIGLIORE CORRISPONDENZA CON MARGINI VUOTI):" << std::endl;
    std::cout << "    Metodo selezionato: " << bestCandidate.method_tag << std::endl;
    std::cout << "    Dimensioni bordo foto: " << bestCandidate.width_px << " x " << bestCandidate.height_px << " px" << std::endl;
    std::cout << "    Margini di vuoto rilevati attorno alla scheda:" << std::endl;
    std::cout << "      - In alto (Top)   : " << bestCandidate.margin_top << " px" << std::endl;
    std::cout << "      - In basso (Bottom): " << bestCandidate.margin_bottom << " px" << std::endl;
    std::cout << "      - A sinistra (Left): " << bestCandidate.margin_left << " px" << std::endl;
    std::cout << "      - A destra (Right) : " << bestCandidate.margin_right << " px" << std::endl;
    std::cout << "    Aspect ratio foto: " << bestCandidate.aspect_ratio
              << " vs Gerber: " << targetAR 
              << " (Scostamento: " << std::fixed << std::setprecision(2)
              << (std::abs(bestCandidate.aspect_ratio - targetAR) / targetAR * 100.0) << "%)" << std::endl;

    return bestCandidate;
}

cv::Mat BoardEdgeDetector::renderCandidatesVisualization(
    const cv::Mat& fusedImg,
    const std::vector<DetectedBoardEdge>& candidates,
    const DetectedBoardEdge& selected) const {

    cv::Mat vis = fusedImg.clone();

    // 1. Disegna tutti i candidati scartati in giallo/arancione sottile
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        if (c.corners_px.size() == 4 && c.method_tag != selected.method_tag) {
            for (int j = 0; j < 4; ++j) {
                cv::line(vis, c.corners_px[j], c.corners_px[(j + 1) % 4], cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
            }
        }
    }

    // 2. Disegna il candidato selezionato in verde brillante spesso (spessore 3)
    if (selected.corners_px.size() == 4) {
        for (int j = 0; j < 4; ++j) {
            cv::line(vis, selected.corners_px[j], selected.corners_px[(j + 1) % 4], cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            cv::circle(vis, selected.corners_px[j], 6, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            cv::circle(vis, selected.corners_px[j], 8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        }

        // Evidenzia i margini vuoti
        cv::Rect bb = selected.bounding_box_px;
        cv::rectangle(vis, cv::Rect(0, 0, vis.cols, bb.y), cv::Scalar(50, 50, 50), -1); // Vuoto Top
        cv::rectangle(vis, cv::Rect(0, bb.y + bb.height, vis.cols, vis.rows - (bb.y + bb.height)), cv::Scalar(50, 50, 50), -1); // Vuoto Bottom
        cv::rectangle(vis, cv::Rect(0, bb.y, bb.x, bb.height), cv::Scalar(50, 50, 50), -1); // Vuoto Left
        cv::rectangle(vis, cv::Rect(bb.x + bb.width, bb.y, vis.cols - (bb.x + bb.width), bb.height), cv::Scalar(50, 50, 50), -1); // Vuoto Right

        // Ripristina trasparenza sul vuoto
        cv::addWeighted(vis, 0.7, fusedImg, 0.3, 0.0, vis);
    }

    // Banner diagnostico
    int bannerH = 150;
    int bannerW = 500;
    cv::Rect bRect(15, 15, bannerW, bannerH);
    bRect &= cv::Rect(0, 0, vis.cols, vis.rows);
    cv::Mat roi = vis(bRect);
    cv::Mat dark = cv::Mat::zeros(roi.size(), vis.type());
    cv::addWeighted(roi, 0.2, dark, 0.8, 0.0, roi);
    cv::rectangle(vis, bRect, cv::Scalar(0, 255, 0), 2);

    cv::putText(vis, "DIAGNOSTICA BORDI SCHEDA RICONOSCIUTI", cv::Point(25, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    std::stringstream ss1, ss2, ss3, ss4;
    ss1 << "Candidati trovati: " << candidates.size() << " | Metodo: " << selected.method_tag;
    cv::putText(vis, ss1.str(), cv::Point(25, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    ss2 << "Bordo scheda: " << static_cast<int>(selected.width_px) << "x" << static_cast<int>(selected.height_px)
        << " px (AR: " << std::fixed << std::setprecision(2) << selected.aspect_ratio << ")";
    cv::putText(vis, ss2.str(), cv::Point(25, 94), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(100, 255, 100), 1, cv::LINE_AA);

    ss3 << "Vuoto rilevato: Top=" << selected.margin_top << "px, Bottom=" << selected.margin_bottom
        << "px, Left=" << selected.margin_left << "px, Right=" << selected.margin_right << "px";
    cv::putText(vis, ss3.str(), cv::Point(25, 120), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    ss4 << "Verde PCB: Riconosciuto ed unito (componenti coprenti sanati)";
    cv::putText(vis, ss4.str(), cv::Point(25, 144), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(200, 200, 255), 1, cv::LINE_AA);

    return vis;
}

} // namespace pcb
