#include "board_alignment.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <algorithm>
#include <cmath>

namespace fs = std::filesystem;

namespace pcb {

BoardAlignmentProcessor::BoardAlignmentProcessor() {
}

cv::Point2f AlignmentResult::pixelToMillimeter(const cv::Point2f& ptPx) const {
    if (!homography_px_to_mm.empty()) {
        std::vector<cv::Point2f> src = {ptPx};
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homography_px_to_mm);
        return dst[0];
    } else if (!affine_px_to_mm.empty()) {
        cv::Mat ptMat = (cv::Mat_<double>(3, 1) << ptPx.x, ptPx.y, 1.0);
        cv::Mat resMat = affine_px_to_mm * ptMat;
        return cv::Point2f(static_cast<float>(resMat.at<double>(0, 0)),
                           static_cast<float>(resMat.at<double>(1, 0)));
    }
    return cv::Point2f(ptPx.x * static_cast<float>(mm_per_pixel),
                       ptPx.y * static_cast<float>(mm_per_pixel));
}

AlignmentResult BoardAlignmentProcessor::computeAlignment(
    const std::vector<cv::Mat>& images,
    const GerberBoardOutline& gerberOutline,
    const std::string& boardName,
    const std::string& side) const {

    AlignmentResult res;
    res.board_name = boardName;
    res.side = side;
    res.gerber_outline = gerberOutline;

    if (images.empty()) {
        std::cerr << "ERRORE: Nessuna immagine fornita per l'allineamento." << std::endl;
        return res;
    }

    // 1. Riconoscimento candidati bordo scheda dalle foto (unendo le esposizioni)
    std::vector<DetectedBoardEdge> candidates = edgeDetector_.detectCandidates(images);
    if (candidates.empty()) {
        std::cerr << "ERRORE: Nessun candidato bordo PCB rilevato nelle immagini." << std::endl;
        return res;
    }

    // 2. Selezione del candidato migliore associato al disegno Gerber
    DetectedBoardEdge bestEdge = edgeDetector_.selectBestCandidate(candidates, gerberOutline);
    if (!bestEdge.valid) {
        std::cerr << "ERRORE: Bordo PCB selezionato non valido." << std::endl;
        return res;
    }

    res.detected_edge = bestEdge;
    res.crop_rect_px = bestEdge.bounding_box_px;

    // Se il Gerber è valido, calcoliamo la relazione geometrica mm <-> pixel
    if (gerberOutline.valid) {
        double gW = gerberOutline.width_mm;
        double gH = gerberOutline.height_mm;
        double imgW = bestEdge.width_px;
        double imgH = bestEdge.height_px;

        // Calcolo della scala (pixel per mm)
        double scaleX = imgW / (gW + 1e-5);
        double scaleY = imgH / (gH + 1e-5);
        res.pixels_per_mm = (scaleX + scaleY) / 2.0;
        res.mm_per_pixel = (res.pixels_per_mm > 1e-5) ? (1.0 / res.pixels_per_mm) : 0.0;
        res.aspect_ratio_error = std::abs(bestEdge.aspect_ratio - gerberOutline.aspect_ratio) / gerberOutline.aspect_ratio;
        res.rotation_angle_deg = bestEdge.rotated_rect_px.angle;

        // Punti 2D per il calcolo di omografia e affine
        // Coordinate Gerber ideali del rettangolo del bordo (in mm)
        std::vector<cv::Point2f> ptsGerber(4);
        ptsGerber[0] = cv::Point2f(0.0f, 0.0f);
        ptsGerber[1] = cv::Point2f(static_cast<float>(gW), 0.0f);
        ptsGerber[2] = cv::Point2f(static_cast<float>(gW), static_cast<float>(gH));
        ptsGerber[3] = cv::Point2f(0.0f, static_cast<float>(gH));

        // Punti dell'immagine (in pixel)
        std::vector<cv::Point2f> ptsImg = bestEdge.corners_px;

        // Verifica congruenza orientamento: se il rettangolo foto è ruotato a 90° rispetto a Gerber,
        // adattiamo i punti Gerber per un perfetto allineamento orientato
        bool imgIsWider = (bestEdge.corners_px[1].x - bestEdge.corners_px[0].x) > 
                          (bestEdge.corners_px[3].y - bestEdge.corners_px[0].y);
        bool gbrIsWider = (gW >= gH);

        if (imgIsWider != gbrIsWider) {
            ptsGerber[0] = cv::Point2f(0.0f, 0.0f);
            ptsGerber[1] = cv::Point2f(0.0f, static_cast<float>(gH));
            ptsGerber[2] = cv::Point2f(static_cast<float>(gW), static_cast<float>(gH));
            ptsGerber[3] = cv::Point2f(static_cast<float>(gW), 0.0f);
        }

        // Calcolo omografia 3x3
        res.homography_px_to_mm = cv::findHomography(ptsImg, ptsGerber);
        if (!res.homography_px_to_mm.empty()) {
            res.homography_mm_to_px = res.homography_px_to_mm.inv();
        }

        // Calcolo trasformazione affine 2x3
        res.affine_px_to_mm = cv::estimateAffine2D(ptsImg, ptsGerber);
        if (!res.affine_px_to_mm.empty()) {
            cv::invertAffineTransform(res.affine_px_to_mm, res.affine_mm_to_px);
        }

        res.success = true;
    } else {
        // Modalità standalone (senza gerber): stima scala fittizia / conservativa
        res.pixels_per_mm = 10.0; // Valore di default indicativo
        res.mm_per_pixel = 0.1;
        res.success = true;
    }

    return res;
}

cv::Mat BoardAlignmentProcessor::renderAlignmentOverlay(
    const cv::Mat& srcImage,
    const AlignmentResult& alignment,
    const std::string& imageName) const {

    cv::Mat vis = srcImage.clone();
    if (vis.empty() || !alignment.success) {
        return vis;
    }

    const auto& edge = alignment.detected_edge;

    // 1. Disegno del poligono / perimetro del bordo scheda in verde brillante
    if (edge.corners_px.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            cv::line(vis, edge.corners_px[i], edge.corners_px[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }

        // Disegno dei 4 angoli con cerchietti e label
        const char* cornerLabels[4] = {"TL", "TR", "BR", "BL"};
        for (int i = 0; i < 4; ++i) {
            cv::circle(vis, edge.corners_px[i], 6, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            cv::circle(vis, edge.corners_px[i], 8, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

            cv::Point2f ptMm = alignment.pixelToMillimeter(edge.corners_px[i]);
            std::stringstream ss;
            ss << cornerLabels[i] << " (" << std::fixed << std::setprecision(1) << ptMm.x << "," << ptMm.y << "mm)";
            cv::putText(vis, ss.str(), edge.corners_px[i] + cv::Point2f(8.0f, -6.0f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            cv::putText(vis, ss.str(), edge.corners_px[i] + cv::Point2f(8.0f, -6.0f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // 2. Disegno della terna di assi cartesiani allineati al PCB
        cv::Point2f origin = edge.corners_px[0];
        cv::Point2f axisXDir = edge.corners_px[1] - edge.corners_px[0];
        cv::Point2f axisYDir = edge.corners_px[3] - edge.corners_px[0];
        float lenX = static_cast<float>(cv::norm(axisXDir));
        float lenY = static_cast<float>(cv::norm(axisYDir));
        float arrowLen = std::min(lenX, lenY) * 0.25f;

        if (lenX > 1e-3f && lenY > 1e-3f) {
            cv::Point2f ptAxisX = origin + (axisXDir / lenX) * arrowLen;
            cv::Point2f ptAxisY = origin + (axisYDir / lenY) * arrowLen;

            cv::arrowedLine(vis, origin, ptAxisX, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, 0.2);
            cv::putText(vis, "X (mm)", ptAxisX + cv::Point2f(5, 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            cv::arrowedLine(vis, origin, ptAxisY, cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0, 0.2);
            cv::putText(vis, "Y (mm)", ptAxisY + cv::Point2f(5, 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        }
    }

    // 3. Banner informativo semitrasparente in alto a sinistra
    int bannerW = 480;
    int bannerH = 175;
    cv::Rect bannerRect(15, 15, bannerW, bannerH);
    bannerRect &= cv::Rect(0, 0, vis.cols, vis.rows);

    cv::Mat bannerRoi = vis(bannerRect);
    cv::Mat darkOverlay = cv::Mat::zeros(bannerRoi.size(), vis.type());
    cv::addWeighted(bannerRoi, 0.25, darkOverlay, 0.75, 0.0, bannerRoi);
    cv::rectangle(vis, bannerRect, cv::Scalar(0, 255, 0), 2);

    auto drawText = [&](const std::string& txt, int x, int y, const cv::Scalar& color, double fsize = 0.48, int thick = 1) {
        cv::putText(vis, txt, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fsize, cv::Scalar(0, 0, 0), thick + 2, cv::LINE_AA);
        cv::putText(vis, txt, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fsize, color, thick, cv::LINE_AA);
    };

    int startY = 38;
    int stepY = 24;

    std::string title = "ALLINEAMENTO SCHEDA: " + alignment.board_name + " [" + alignment.side + "]";
    drawText(title, 25, startY, cv::Scalar(0, 255, 255), 0.55, 2);

    std::stringstream s1, s2, s3, s4, s5;
    s1 << "Foto: " << static_cast<int>(edge.width_px) << "x" << static_cast<int>(edge.height_px)
       << " px (AR: " << std::fixed << std::setprecision(2) << edge.aspect_ratio << ")";
    drawText(s1.str(), 25, startY + stepY, cv::Scalar(255, 255, 255));

    if (alignment.gerber_outline.valid) {
        s2 << "Gerber: " << std::fixed << std::setprecision(2) << alignment.gerber_outline.width_mm
           << "x" << alignment.gerber_outline.height_mm << " mm (AR: "
           << alignment.gerber_outline.aspect_ratio << ")";
        drawText(s2.str(), 25, startY + stepY * 2, cv::Scalar(100, 255, 100));

        s3 << "Layer: " << alignment.gerber_outline.best_contour.layer_name
           << " (Area: " << std::setprecision(1) << alignment.gerber_outline.area_mm2 << " mm^2)";
        drawText(s3.str(), 25, startY + stepY * 3, cv::Scalar(220, 220, 220));

        s4 << "Scala: " << std::setprecision(2) << alignment.pixels_per_mm
           << " px/mm  (" << std::setprecision(4) << alignment.mm_per_pixel << " mm/px)";
        drawText(s4.str(), 25, startY + stepY * 4, cv::Scalar(0, 255, 255));

        s5 << "Errore AR Bordo: " << std::setprecision(2) << (alignment.aspect_ratio_error * 100.0) << "%";
        drawText(s5.str(), 25, startY + stepY * 5, cv::Scalar(200, 200, 255));
    } else {
        drawText("Gerber non presente (Rilevamento bordo autonomo)", 25, startY + stepY * 2, cv::Scalar(0, 165, 255));
    }

    if (!imageName.empty()) {
        drawText("Img: " + imageName, vis.cols - 350, 30, cv::Scalar(255, 255, 255), 0.5, 1);
    }

    return vis;
}

cv::Mat BoardAlignmentProcessor::cropAlongBoardEdge(
    const cv::Mat& srcImage,
    const AlignmentResult& alignment,
    bool straighten) const {

    if (srcImage.empty() || !alignment.success) {
        return srcImage.clone();
    }

    const auto& edge = alignment.detected_edge;

    // Se il rettangolo è allineato (o quasi) agli assi e non serve raddrizzamento
    if (!straighten || std::abs(edge.rotated_rect_px.angle) < 0.8 || std::abs(std::abs(edge.rotated_rect_px.angle) - 90.0) < 0.8) {
        cv::Rect r = edge.bounding_box_px;
        r = r & cv::Rect(0, 0, srcImage.cols, srcImage.rows);
        if (r.width > 10 && r.height > 10) {
            return srcImage(r).clone();
        }
    }

    // Ritaglio orientato tramite warpPerspective (raddrizza perfettamente la scheda)
    if (edge.corners_px.size() == 4) {
        float outW = static_cast<float>(edge.width_px);
        float outH = static_cast<float>(edge.height_px);

        std::vector<cv::Point2f> dstCorners = {
            cv::Point2f(0.0f, 0.0f),
            cv::Point2f(outW, 0.0f),
            cv::Point2f(outW, outH),
            cv::Point2f(0.0f, outH)
        };

        cv::Mat warpMat = cv::getPerspectiveTransform(edge.corners_px, dstCorners);
        cv::Mat cropped;
        cv::warpPerspective(srcImage, cropped, warpMat, cv::Size(static_cast<int>(outW), static_cast<int>(outH)),
                            cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        return cropped;
    }

    // Fallback con bounding box
    cv::Rect r = edge.bounding_box_px & cv::Rect(0, 0, srcImage.cols, srcImage.rows);
    return srcImage(r).clone();
}

bool BoardAlignmentProcessor::saveAlignmentParams(
    const std::string& filePath,
    const AlignmentResult& alignment) const {

    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "ERRORE: Impossibile creare il file parametri: " << filePath << std::endl;
        return false;
    }

    fs << "board_name" << alignment.board_name;
    fs << "side" << alignment.side;
    fs << "pixels_per_mm" << alignment.pixels_per_mm;
    fs << "mm_per_pixel" << alignment.mm_per_pixel;
    fs << "rotation_angle_deg" << alignment.rotation_angle_deg;
    fs << "aspect_ratio_error" << alignment.aspect_ratio_error;

    if (alignment.gerber_outline.valid) {
        fs << "gerber_source_file" << alignment.gerber_outline.best_contour.source_file;
        fs << "gerber_layer_name" << alignment.gerber_outline.best_contour.layer_name;
        fs << "gerber_width_mm" << alignment.gerber_outline.width_mm;
        fs << "gerber_height_mm" << alignment.gerber_outline.height_mm;
        fs << "gerber_area_mm2" << alignment.gerber_outline.area_mm2;
        fs << "gerber_aspect_ratio" << alignment.gerber_outline.aspect_ratio;
    }

    fs << "image_width_px" << alignment.detected_edge.width_px;
    fs << "image_height_px" << alignment.detected_edge.height_px;
    fs << "image_area_px" << alignment.detected_edge.area_px;
    fs << "image_solidity" << alignment.detected_edge.solidity;

    if (!alignment.homography_px_to_mm.empty()) {
        fs << "homography_px_to_mm" << alignment.homography_px_to_mm;
        fs << "homography_mm_to_px" << alignment.homography_mm_to_px;
    }
    if (!alignment.affine_px_to_mm.empty()) {
        fs << "affine_px_to_mm" << alignment.affine_px_to_mm;
        fs << "affine_mm_to_px" << alignment.affine_mm_to_px;
    }

    fs.release();
    return true;
}

bool BoardAlignmentProcessor::processBoardSide(
    const std::string& boardName,
    const std::string& side,
    const std::string& undistortedDir,
    const std::string& gerberDir,
    const std::string& alignmentOutputDir,
    const std::string& paramsOutputDir,
    bool showWindows) const {

    fs::path sidePath = fs::path(undistortedDir) / boardName / side;
    if (!fs::exists(sidePath)) {
        std::cerr << "Cartella non trovata: " << sidePath << std::endl;
        return false;
    }

    // 1. Raccogliamo i file immagine validi nella cartella (escludendo eventuali sottocartelle come ok/)
    std::vector<std::string> validExts = {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"};
    std::vector<fs::path> imagePaths;
    fs::path okSideDir = sidePath / "ok";

    for (const auto& entry : fs::directory_iterator(sidePath)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
            if (std::find(validExts.begin(), validExts.end(), ext) != validExts.end()) {
                imagePaths.push_back(entry.path());
            }
        }
    }
    std::sort(imagePaths.begin(), imagePaths.end());

    // Se sidePath è vuoto o contiene file già ritagliati e ok/ contiene le originali non ritagliate, usiamo ok/
    if (fs::exists(okSideDir) && !fs::is_empty(okSideDir)) {
        bool useOkDir = imagePaths.empty();
        if (!useOkDir && !imagePaths.empty()) {
            fs::path sampleOk = okSideDir / imagePaths[0].filename();
            if (fs::exists(sampleOk)) {
                cv::Mat sImg = cv::imread(imagePaths[0].string());
                cv::Mat okImg = cv::imread(sampleOk.string());
                if (!sImg.empty() && !okImg.empty() && (sImg.cols < okImg.cols || sImg.rows < okImg.rows)) {
                    useOkDir = true;
                    std::cout << "Rilevate immagini già ritagliate in " << sidePath 
                              << ". Ricarico le originali non ritagliate da " << okSideDir << "." << std::endl;
                }
            }
        }

        if (useOkDir) {
            imagePaths.clear();
            for (const auto& entry : fs::directory_iterator(okSideDir)) {
                if (entry.is_regular_file()) {
                    std::string ext = entry.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                    if (std::find(validExts.begin(), validExts.end(), ext) != validExts.end()) {
                        imagePaths.push_back(entry.path());
                    }
                }
            }
            std::sort(imagePaths.begin(), imagePaths.end());
        }
    }

    if (imagePaths.empty()) {
        std::cout << "Nessuna immagine trovata in " << sidePath << ". Salto." << std::endl;
        return true;
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << " AVVIO RICONOSCIMENTO BORDI ED ALLINEAMENTO GERBER " << std::endl;
    std::cout << " Scheda: " << boardName << " | Lato: " << side << std::endl;
    std::cout << " Numero foto disponibili (esposizioni diverse): " << imagePaths.size() << std::endl;
    std::cout << "========================================================" << std::endl;

    // 2. Parsing Gerber per la scheda: cerca lo strato con area massima
    GerberBoardOutline gerberOutline = GerberParser::findAndParseForBoard(gerberDir, boardName);
    if (!gerberOutline.valid) {
        std::cout << "ATTENZIONE: Nessun disegno Gerber valido trovato in " << gerberDir 
                  << " per la scheda " << boardName << "." << std::endl;
        std::cout << "Procedo con riconoscimento bordo e ritaglio basato sulle immagini fotografiche." << std::endl;
    }

    // 3. Caricamento di tutte le immagini della serie
    std::vector<cv::Mat> images;
    images.reserve(imagePaths.size());
    for (const auto& p : imagePaths) {
        cv::Mat m = cv::imread(p.string());
        if (!m.empty()) {
            images.push_back(m);
        } else {
            std::cerr << "Attenzione: Impossibile leggere immagine " << p << std::endl;
        }
    }

    if (images.empty()) {
        std::cerr << "ERRORE: Nessuna immagine caricata correttamente per " << boardName << " [" << side << "]" << std::endl;
        return false;
    }

    // 4. Calcolo allineamento e associazione Pixel <-> Gerber
    AlignmentResult alignment = computeAlignment(images, gerberOutline, boardName, side);
    if (!alignment.success) {
        std::cerr << "ERRORE: Allineamento fallito per " << boardName << " [" << side << "]" << std::endl;
        return false;
    }

    // 5. Creazione cartelle di output
    fs::path outAlignSideDir = fs::path(alignmentOutputDir) / boardName / side;
    fs::create_directories(outAlignSideDir);
    fs::create_directories(okSideDir);
    fs::create_directories(paramsOutputDir);

    // Salvataggio dell'immagine diagnostica con tutti i bordi riconosciuti dal software
    cv::Mat fusedImg = edgeDetector_.combineMultiExposure(images);
    std::vector<DetectedBoardEdge> allCandidates = edgeDetector_.detectCandidates(images);
    cv::Mat debugImg = edgeDetector_.renderCandidatesVisualization(fusedImg, allCandidates, alignment.detected_edge);
    std::string debugOutPath = (outAlignSideDir / "board_edge_debug.png").string();
    cv::imwrite(debugOutPath, debugImg);
    std::cout << "Diagnostica bordi riconosciuti salvata in: " << debugOutPath << std::endl;

    if (showWindows && !debugImg.empty()) {
        std::string dbgWinName = "Diagnostica Bordi Riconosciuti: " + boardName + " [" + side + "]";
        cv::namedWindow(dbgWinName, cv::WINDOW_NORMAL);
        cv::resizeWindow(dbgWinName, 1350, 950);
        cv::imshow(dbgWinName, debugImg);
        std::cout << "\n>>> Visualizzazione a schermo bordi scheda. Premi un tasto sulla finestra OpenCV per continuare..." << std::endl;
        cv::waitKey(0);
        cv::destroyWindow(dbgWinName);
    }

    // 6. Salvataggio parametri calibrazione / associazione
    std::string paramsFile = (fs::path(paramsOutputDir) / ("board_alignment_" + boardName + "_" + side + ".yml")).string();
    if (saveAlignmentParams(paramsFile, alignment)) {
        std::cout << "Parametri allineamento salvati in: " << paramsFile << std::endl;
    }

    // 7. Elaborazione di ciascuna foto:
    //    a) Salvataggio visualizzazione con riquadro bordo e allineamento
    //    b) Ritaglio immagine lungo il bordo
    //    c) Spostamento dell'originale nella cartella ok/
    //    d) Salvataggio dell'immagine ritagliata nella cartella undistorted
    std::cout << "\nGenerazione visualizzazioni e ritaglio immagini lungo il bordo..." << std::endl;
    cv::Mat lastVis;

    for (size_t i = 0; i < imagePaths.size(); ++i) {
        const auto& imgPath = imagePaths[i];
        const auto& rawImg = images[i];
        std::string filename = imgPath.filename().string();
        std::string stem = imgPath.stem().string();

        // a) Immagine con riquadro del bordo e allineamento trovato
        cv::Mat visImg = renderAlignmentOverlay(rawImg, alignment, filename);
        lastVis = visImg;
        std::string alignOutPath = (outAlignSideDir / (stem + "_aligned.png")).string();
        cv::imwrite(alignOutPath, visImg);

        // b) Ritaglio lungo il bordo
        cv::Mat croppedImg = cropAlongBoardEdge(rawImg, alignment, true);

        // c) Spostamento / conservazione originale non ritagliata in ok/
        fs::path destOk = okSideDir / filename;
        if (imgPath != destOk) {
            std::error_code ec;
            fs::copy_file(imgPath, destOk, fs::copy_options::overwrite_existing, ec);
            if (!ec) {
                fs::remove(imgPath, ec);
            }
        }

        // d) Salvataggio immagine ritagliata nella cartella undistorted (sidePath / filename)
        fs::path destCropped = sidePath / filename;
        cv::imwrite(destCropped.string(), croppedImg);

        std::cout << "  - [" << (i + 1) << "/" << imagePaths.size() << "] "
                  << filename << ": ritaglio (" << croppedImg.cols << "x" << croppedImg.rows 
                  << " px) salvato. Originale salvata in ok/. Overlay: " << alignOutPath << std::endl;
    }

    if (showWindows && !lastVis.empty()) {
        std::string winName = "Bordo e Allineamento Gerber: " + boardName + " [" + side + "]";
        cv::namedWindow(winName, cv::WINDOW_NORMAL);
        cv::resizeWindow(winName, 1300, 900);
        cv::imshow(winName, lastVis);
        std::cout << "\nPremi un tasto sulla finestra OpenCV per continuare..." << std::endl;
        cv::waitKey(0);
        cv::destroyWindow(winName);
    }

    std::cout << "Allineamento e ritaglio per " << boardName << " [" << side << "] completato con successo!" << std::endl;
    return true;
}

} // namespace pcb

