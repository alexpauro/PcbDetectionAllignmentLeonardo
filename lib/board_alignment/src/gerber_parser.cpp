#include "gerber_parser.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace fs = std::filesystem;

namespace pcb {

namespace {

// Calcola le proprietà geometriche di un contorno poligonale
void finalizeContour(GerberContour& c) {
    if (c.polygon_mm.size() < 3) {
        c.area_mm2 = 0.0;
        return;
    }

    c.area_mm2 = std::abs(cv::contourArea(c.polygon_mm));

    std::vector<cv::Point2f> pts = c.polygon_mm;
    c.min_area_rect_mm = cv::minAreaRect(pts);
    c.bounding_box_mm = cv::boundingRect(pts);

    float rw = c.min_area_rect_mm.size.width;
    float rh = c.min_area_rect_mm.size.height;
    c.width_mm = std::max(rw, rh);
    c.height_mm = std::min(rw, rh);
    c.aspect_ratio = (c.height_mm > 1e-4) ? (c.width_mm / c.height_mm) : 1.0;
}

// Discretizza un arco circolare G02 (orario) o G03 (antiorario) in segmenti poligonali
void discretizeArc(
    double startX, double startY,
    double endX, double endY,
    double offsetI, double offsetJ,
    int gCode,
    std::vector<cv::Point2f>& outPoints) {

    double centerX = startX + offsetI;
    double centerY = startY + offsetJ;
    double radius = std::hypot(offsetI, offsetJ);

    if (radius < 1e-5) {
        outPoints.push_back(cv::Point2f(static_cast<float>(endX), static_cast<float>(endY)));
        return;
    }

    double startAngle = std::atan2(startY - centerY, startX - centerX);
    double endAngle = std::atan2(endY - centerY, endX - centerX);

    if (gCode == 2) { // CW
        if (endAngle >= startAngle) endAngle -= 2.0 * CV_PI;
    } else { // CCW
        if (endAngle <= startAngle) endAngle += 2.0 * CV_PI;
    }

    int steps = 8;
    for (int s = 1; s <= steps; ++s) {
        double t = static_cast<double>(s) / steps;
        double ang = startAngle + t * (endAngle - startAngle);
        outPoints.push_back(cv::Point2f(
            static_cast<float>(centerX + radius * std::cos(ang)),
            static_cast<float>(centerY + radius * std::sin(ang))
        ));
    }
}

} // namespace

bool GerberParser::parseSingleFile(const std::string& filePath, std::vector<GerberContour>& outContours) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "ERRORE: Impossibile aprire il file Gerber: " << filePath << std::endl;
        return false;
    }

    std::string sourceFilename = fs::path(filePath).filename().string();
    std::string currentLayerName = sourceFilename;

    // Default RS-274X: pollici (Inches -> mm) e divisore 10^6
    double unitScale = 25.4;
    double xDivisor = 1000000.0;
    double yDivisor = 1000000.0;

    // Stato modale del parser RS-274X
    double curX = 0.0;
    double curY = 0.0;
    int currentDCode = 0;   // 1 = Draw, 2 = Move, 3 = Flash
    int currentGCode = 1;   // 1 = Linear, 2 = CW Arc, 3 = CCW Arc
    bool inContourMode = false; // G36/G37
    bool inMacro = false;       // %AM...%

    std::vector<cv::Point2f> currentLoop;
    std::vector<cv::Point2f> currentPolygon;

    auto pushContour = [&](const std::vector<cv::Point2f>& poly, const std::string& subLayer = "") {
        if (poly.size() >= 3) {
            GerberContour gc;
            gc.layer_name = subLayer.empty() ? currentLayerName : subLayer;
            gc.source_file = sourceFilename;
            gc.polygon_mm = poly;
            finalizeContour(gc);
            if (gc.area_mm2 > 0.05) {
                outContours.push_back(gc);
            }
        }
    };

    std::string line;

    while (std::getline(file, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
            line.pop_back();
        }
        if (line.empty()) continue;

        // Gestione macro aperture multilinea (%AM...%)
        if (inMacro) {
            if (line.find('%') != std::string::npos) {
                inMacro = false;
            }
            continue;
        }
        if (line.find("%AM") != std::string::npos) {
            if (line.find('%', line.find("%AM") + 3) == std::string::npos) {
                inMacro = true;
            }
            continue;
        }

        // Ignora commenti G04
        if (line[0] == 'G' && line.size() >= 3 && line[1] == '0' && line[2] == '4') {
            continue;
        }

        // 1. Comandi estesi (%...%)
        if (line[0] == '%') {
            if (line.find("MOIN") != std::string::npos) {
                unitScale = 25.4; // Pollici -> mm
            } else if (line.find("MOMM") != std::string::npos) {
                unitScale = 1.0;  // Millimetri
            } else if (line.find("FS") != std::string::npos) {
                size_t posX = line.find('X');
                if (posX != std::string::npos && posX + 2 < line.size()) {
                    int dec = line[posX + 2] - '0';
                    if (dec >= 1 && dec <= 8) xDivisor = std::pow(10.0, dec);
                }
                size_t posY = line.find('Y');
                if (posY != std::string::npos && posY + 2 < line.size()) {
                    int dec = line[posY + 2] - '0';
                    if (dec >= 1 && dec <= 8) yDivisor = std::pow(10.0, dec);
                }
            } else if (line.find("LN") != std::string::npos) {
                size_t lnPos = line.find("LN");
                size_t star = line.find('*', lnPos);
                if (star != std::string::npos) {
                    if (currentLoop.size() >= 3) {
                        pushContour(currentLoop);
                        currentLoop.clear();
                    }
                    currentLayerName = line.substr(lnPos + 2, star - (lnPos + 2));
                }
            }
            continue;
        }

        if (line.find('*') == std::string::npos) continue;

        // Modalità contorno poligonale G36 / G37
        if (line.find("G36") != std::string::npos) {
            inContourMode = true;
            currentPolygon.clear();
            continue;
        }
        if (line.find("G37") != std::string::npos) {
            inContourMode = false;
            if (!currentPolygon.empty()) {
                pushContour(currentPolygon);
                currentPolygon.clear();
            }
            continue;
        }

        // Tokenizer sequenziale veloce (no regex per massime prestazioni)
        bool hasX = false, hasY = false, hasD = false, hasI = false, hasJ = false;
        double newX = curX, newY = curY;
        double offsetI = 0.0, offsetJ = 0.0;

        char* endptr = nullptr;
        for (size_t k = 0; k < line.size(); ++k) {
            char c = line[k];
            if (c == 'X') {
                newX = (std::strtod(&line[k + 1], &endptr) / xDivisor) * unitScale;
                hasX = true;
                k = endptr - &line[0] - 1;
            } else if (c == 'Y') {
                newY = (std::strtod(&line[k + 1], &endptr) / yDivisor) * unitScale;
                hasY = true;
                k = endptr - &line[0] - 1;
            } else if (c == 'I') {
                offsetI = (std::strtod(&line[k + 1], &endptr) / xDivisor) * unitScale;
                hasI = true;
                k = endptr - &line[0] - 1;
            } else if (c == 'J') {
                offsetJ = (std::strtod(&line[k + 1], &endptr) / yDivisor) * unitScale;
                hasJ = true;
                k = endptr - &line[0] - 1;
            } else if (c == 'D') {
                currentDCode = std::strtol(&line[k + 1], &endptr, 10);
                hasD = true;
                k = endptr - &line[0] - 1;
            } else if (c == 'G') {
                int g = std::strtol(&line[k + 1], &endptr, 10);
                if (g == 1 || g == 2 || g == 3) currentGCode = g;
                k = endptr - &line[0] - 1;
            }
        }

        cv::Point2f nextPt(static_cast<float>(newX), static_cast<float>(newY));

        if (inContourMode) {
            currentPolygon.push_back(nextPt);
            curX = newX;
            curY = newY;
            continue;
        }

        // D02: Penna SU / Spostamento a nuova origine
        if (currentDCode == 2 && (hasX || hasY || (hasD && currentDCode == 2))) {
            if (currentLoop.size() >= 3) {
                pushContour(currentLoop);
            }
            currentLoop.clear();
            curX = newX;
            curY = newY;
            currentLoop.push_back(cv::Point2f(static_cast<float>(curX), static_cast<float>(curY)));
        }
        // D01: Penna GIÙ / Tracciamento linea o arco
        else if (currentDCode == 1 && (hasX || hasY)) {
            if ((currentGCode == 2 || currentGCode == 3) && (hasI || hasJ)) {
                discretizeArc(curX, curY, newX, newY, offsetI, offsetJ, currentGCode, currentLoop);
            } else {
                currentLoop.push_back(nextPt);
            }
            curX = newX;
            curY = newY;

            // Chiusura automatica del perimetro al ritorno sul punto di partenza
            if (currentLoop.size() >= 4) {
                cv::Point2f startPt = currentLoop.front();
                float dist = cv::norm(currentLoop.back() - startPt);
                if (dist < 0.2f) {
                    pushContour(currentLoop);
                    currentLoop.clear();
                    currentLoop.push_back(startPt);
                }
            }
        }

        // Cambio tool/apertura (es. G54D11* o D10+): chiude eventuale loop
        if (line.find("G54") != std::string::npos || (hasD && currentDCode > 3)) {
            if (currentLoop.size() >= 3) {
                pushContour(currentLoop);
            }
            currentLoop.clear();
        }
    }

    if (!currentPolygon.empty()) {
        pushContour(currentPolygon);
    }
    if (currentLoop.size() >= 3) {
        pushContour(currentLoop);
    }

    return !outContours.empty();
}

GerberBoardOutline GerberParser::parse(const std::string& path) {
    GerberBoardOutline result;
    result.file_path = path;

    if (!fs::exists(path)) {
        std::cerr << "ERRORE: Percorso Gerber non esistente: " << path << std::endl;
        return result;
    }

    std::vector<fs::path> targetFiles;
    if (fs::is_directory(path)) {
        // Estensioni file CAD/CAM di produzione (Mentor Graphics Xpedition, Altium, Cadence, ODB)
        std::vector<std::string> validExts = {
            ".gdo", ".gbr", ".gerber", ".gko", ".gm1", ".gm2", ".gml",
            ".out", ".outline", ".gtl", ".gbl", ".gto", ".gbo", ".gts", ".gbs",
            ".art", ".pho", ".drl"
        };
        for (const auto& entry : fs::directory_iterator(path)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                if (std::find(validExts.begin(), validExts.end(), ext) != validExts.end()) {
                    targetFiles.push_back(entry.path());
                }
            }
        }
        std::sort(targetFiles.begin(), targetFiles.end());
    } else {
        targetFiles.push_back(fs::path(path));
    }

    if (targetFiles.empty()) {
        std::cerr << "ERRORE: Nessun file Gerber valido trovato in: " << path << std::endl;
        return result;
    }

    std::cout << "\nAnalisi file Gerber di produzione per identificazione bordo scheda (area massima):" << std::endl;
    for (const auto& f : targetFiles) {
        std::vector<GerberContour> fileContours;
        if (parseSingleFile(f.string(), fileContours)) {
            result.all_contours.insert(result.all_contours.end(), fileContours.begin(), fileContours.end());
        }
    }

    if (result.all_contours.empty()) {
        std::cerr << "ATTENZIONE: Nessun contorno/strato poligonale rilevato nei file Gerber in " << path << std::endl;
        return result;
    }

    // =========================================================================
    // "il file gerber contiene diversi strati: cerca quello con area massima che rappresenta il bordo della scheda"
    // =========================================================================
    double maxArea = -1.0;
    size_t bestIdx = 0;

    for (size_t idx = 0; idx < result.all_contours.size(); ++idx) {
        const auto& c = result.all_contours[idx];
        if (c.area_mm2 > maxArea) {
            maxArea = c.area_mm2;
            bestIdx = idx;
        }
    }

    result.best_contour = result.all_contours[bestIdx];
    result.width_mm = result.best_contour.width_mm;
    result.height_mm = result.best_contour.height_mm;
    result.area_mm2 = result.best_contour.area_mm2;
    result.aspect_ratio = result.best_contour.aspect_ratio;
    result.valid = (result.area_mm2 > 0.0);

    std::cout << "\n>>> STRATO BORDO SCHEDA SELEZIONATO (AREA MASSIMA):" << std::endl;
    std::cout << "    File sorgente: " << result.best_contour.source_file << std::endl;
    std::cout << "    Layer: " << result.best_contour.layer_name << std::endl;
    std::cout << "    Dimensioni disegno Gerber: " << std::fixed << std::setprecision(2)
              << result.width_mm << " x " << result.height_mm << " mm" << std::endl;
    std::cout << "    Area totale bordo: " << result.area_mm2 << " mm^2" << std::endl;
    std::cout << "    Aspect Ratio: " << result.aspect_ratio << std::endl;

    return result;
}

GerberBoardOutline GerberParser::findAndParseForBoard(const std::string& gerberBaseDir, const std::string& boardName) {
    if (!fs::exists(gerberBaseDir)) {
        std::cerr << "Cartella base Gerber non trovata: " << gerberBaseDir << std::endl;
        return GerberBoardOutline();
    }

    // 1. Sottocartella specifica per la scheda: <gerberBaseDir>/<boardName>/
    fs::path boardDir = fs::path(gerberBaseDir) / boardName;
    if (fs::exists(boardDir) && fs::is_directory(boardDir)) {
        return parse(boardDir.string());
    }

    // 2. Cerca file con nome scheda
    for (const auto& entry : fs::directory_iterator(gerberBaseDir)) {
        if (entry.is_regular_file()) {
            std::string stem = entry.path().stem().string();
            std::string stemUpper = stem;
            std::string bnameUpper = boardName;
            std::transform(stemUpper.begin(), stemUpper.end(), stemUpper.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
            std::transform(bnameUpper.begin(), bnameUpper.end(), bnameUpper.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });

            if (stemUpper == bnameUpper || stemUpper.find(bnameUpper) != std::string::npos) {
                return parse(entry.path().string());
            }
        }
    }

    // 3. Se la cartella base contiene direttamente i file
    return parse(gerberBaseDir);
}

} // namespace pcb
