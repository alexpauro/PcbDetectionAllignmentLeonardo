#include "aoi_verification.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cctype>

namespace pcb {

// Palette colori ufficiale per le classi di componenti (BGR)
static const std::map<std::string, cv::Scalar> CLASS_COLORS = {
    {"Resistor",               cv::Scalar(0, 165, 255)},    // Arancione
    {"Capacitor",              cv::Scalar(255, 144, 30)},   // Blu chiaro
    {"Electrolytic Capacitor", cv::Scalar(180, 105, 255)},  // Rosa vivo
    {"IC",                     cv::Scalar(0, 215, 255)},    // Oro / Giallo caldo
    {"Inductor",               cv::Scalar(205, 90, 106)},   // Viola cadetto
    {"Diode",                  cv::Scalar(50, 205, 50)},    // Verde lime
    {"LED",                    cv::Scalar(0, 255, 127)},    // Verde primavera
    {"Transistor",             cv::Scalar(238, 130, 238)},  // Violetto
    {"Oscillator",             cv::Scalar(147, 20, 255)},   // Rosa porpora
    {"Connector",              cv::Scalar(255, 191, 0)},    // Deep sky blue
    {"Switch/Button",          cv::Scalar(0, 69, 255)},     // Rosso arancio
    {"Other",                  cv::Scalar(128, 128, 128)},  // Grigio
    {"Marker",                 cv::Scalar(255, 255, 0)},    // Ciano
    {"?",                      cv::Scalar(80, 80, 80)}      // Grigio scuro
};

// Colori di stato per la verifica AOI (BGR)
static const cv::Scalar COLOR_EXACT_MATCH     = cv::Scalar(40, 200, 40);   // Verde brillante
static const cv::Scalar COLOR_CLASS_CORRECTED = cv::Scalar(0, 160, 255);   // Ambra / Arancio
static const cv::Scalar COLOR_RECOVERED       = cv::Scalar(240, 220, 20);  // Ciano dorato

static std::string trim(const std::string& s) {
    auto start = s.begin();
    while (start != s.end() && std::isspace(static_cast<unsigned char>(*start))) {
        start++;
    }
    auto end = s.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(static_cast<unsigned char>(*end)));
    return (start < end + 1) ? std::string(start, end + 1) : "";
}

static std::string toUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

cv::Point2f AoiAlignmentTransform::aoiToPixel(double x_mm, double y_mm) const {
    if (!valid) return cv::Point2f(0.0f, 0.0f);

    // 1. Se è disponibile il modello di correzione distorsione ottica e prospettiva (Omografia 3x3)
    if (has_distortion_model && !homography_3x3.empty()) {
        std::vector<cv::Point2f> src = { cv::Point2f(static_cast<float>(x_mm), static_cast<float>(y_mm)) };
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homography_3x3);
        if (!dst.empty()) {
            return dst[0];
        }
    }

    // 2. Modello lineare rigido base (fallback iniziale)
    double xr = mxx * x_mm + mxy * y_mm;
    double yr = myx * x_mm + myy * y_mm;

    if (std::abs(residual_angle_deg) > 1e-4) {
        double rad = residual_angle_deg * CV_PI / 180.0;
        double cos_r = std::cos(rad);
        double sin_r = std::sin(rad);
        double x_fine = cos_r * xr - sin_r * yr;
        double y_fine = sin_r * xr + cos_r * yr;
        xr = x_fine;
        yr = y_fine;
    }

    double px = scale_px_per_mm * xr + tx;
    double py = scale_px_per_mm * yr + ty;

    return cv::Point2f(static_cast<float>(px), static_cast<float>(py));
}

cv::Point2f AoiAlignmentTransform::pixelToAoi(const cv::Point2f& ptPx) const {
    if (!valid) return cv::Point2f(0.0f, 0.0f);

    if (has_distortion_model && !homography_inv_3x3.empty()) {
        std::vector<cv::Point2f> src = { ptPx };
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homography_inv_3x3);
        if (!dst.empty()) {
            return dst[0];
        }
    }

    if (scale_px_per_mm <= 1e-6) return cv::Point2f(0.0f, 0.0f);

    double x_unscaled = (ptPx.x - tx) / scale_px_per_mm;
    double y_unscaled = (ptPx.y - ty) / scale_px_per_mm;

    double xr = x_unscaled;
    double yr = y_unscaled;
    if (std::abs(residual_angle_deg) > 1e-4) {
        double rad = -residual_angle_deg * CV_PI / 180.0;
        double cos_r = std::cos(rad);
        double sin_r = std::sin(rad);
        xr = cos_r * x_unscaled - sin_r * y_unscaled;
        yr = sin_r * x_unscaled + cos_r * y_unscaled;
    }

    double det = mxx * myy - mxy * myx;
    if (std::abs(det) < 1e-6) return cv::Point2f(0.0f, 0.0f);

    double x_mm = (myy * xr - mxy * yr) / det;
    double y_mm = (-myx * xr + mxx * yr) / det;

    return cv::Point2f(static_cast<float>(x_mm), static_cast<float>(y_mm));
}

AoiVerificationProcessor::AoiVerificationProcessor() {}

std::vector<std::string> AoiVerificationProcessor::getSupportedClasses() {
    return {
        "Resistor",
        "Capacitor",
        "Electrolytic Capacitor",
        "IC",
        "Inductor",
        "Diode",
        "LED",
        "Transistor",
        "Oscillator",
        "Connector",
        "Switch/Button",
        "Other",
        "Marker"
    };
}

bool AoiVerificationProcessor::loadMappingFile(const std::string& mappingFilePath) {
    mapping_table_.clear();
    mapping_lines_raw_.clear();

    std::ifstream infile(mappingFilePath);
    if (!infile.is_open()) {
        std::cerr << "[AOI] File di mapping non trovato: " << mappingFilePath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::string s = trim(line);
        if (s.empty() || s[0] == '#') continue;

        mapping_lines_raw_.push_back(s);

        size_t arrowPos = s.find("--> Class:");
        if (arrowPos != std::string::npos) {
            std::string leftPart = trim(s.substr(0, arrowPos));
            std::string className = trim(s.substr(arrowPos + 10));

            mapping_table_[leftPart] = className;

            std::istringstream iss(leftPart);
            std::string ref, x, y, q, side, code;
            if (iss >> ref >> x >> y >> q >> side >> code) {
                if (mapping_table_.find("REF:" + ref) == mapping_table_.end()) {
                    mapping_table_["REF:" + ref] = className;
                }
                if (mapping_table_.find("CODE:" + code) == mapping_table_.end()) {
                    mapping_table_["CODE:" + code] = className;
                }
            }
        }
    }

    std::cout << "[AOI] Caricate " << mapping_lines_raw_.size()
              << " associazioni dal file di mapping: " << mappingFilePath << std::endl;
    return true;
}

bool AoiVerificationProcessor::saveMappingEntry(const std::string& mappingFilePath,
                                                const std::string& rawLine,
                                                const std::string& assignedClass) {
    std::ofstream outfile(mappingFilePath, std::ios::app);
    if (!outfile.is_open()) {
        std::cerr << "[AOI] Impossibile aggiornare il file di mapping: " << mappingFilePath << std::endl;
        return false;
    }

    std::string entry = rawLine + " --> Class: " + assignedClass;
    outfile << entry << "\n";
    outfile.flush();

    mapping_lines_raw_.push_back(entry);
    mapping_table_[rawLine] = assignedClass;

    std::istringstream iss(rawLine);
    std::string ref, x, y, q, side, code;
    if (iss >> ref >> x >> y >> q >> side >> code) {
        mapping_table_["REF:" + ref] = assignedClass;
        mapping_table_["CODE:" + code] = assignedClass;
    }

    std::cout << "[AOI] Nuova associazione salvata nel file di mapping: "
              << ref << " -> " << assignedClass << std::endl;
    return true;
}

std::string AoiVerificationProcessor::inferClassFromKeywords(const std::string& ref,
                                                            const std::string& outline,
                                                            const std::string& description,
                                                            bool& isConfident) const {
    isConfident = false;
    std::string uRef = toUpper(ref);
    std::string uOutline = toUpper(outline);
    std::string uDesc = toUpper(description);

    // Resistor
    if (uRef.rfind("R", 0) == 0 && (uRef.size() > 1 && (std::isdigit(uRef[1]) || uRef[1] == 'N'))) {
        if (uDesc.find("RES") != std::string::npos ||
            uDesc.find("OHM") != std::string::npos ||
            uDesc.find("ZERO OHM") != std::string::npos ||
            uOutline.rfind("RH", 0) == 0) {
            isConfident = true;
            return "Resistor";
        }
    }

    // Capacitor (Electrolytic o Ceramico standard)
    if (uRef.rfind("C", 0) == 0 && (uRef.size() > 1 && std::isdigit(uRef[1]))) {
        bool isCapDesc = (uDesc.find("CAP") != std::string::npos ||
                          uDesc.find("COND") != std::string::npos ||
                          uDesc.find("FARAD") != std::string::npos ||
                          uOutline.rfind("CH", 0) == 0);

        if (isCapDesc) {
            if (uDesc.find("ELECTROLYTIC") != std::string::npos ||
                uDesc.find("ALUMINUM") != std::string::npos ||
                uDesc.find("POLARIZED") != std::string::npos ||
                uDesc.find("SOLID POLYMER") != std::string::npos ||
                uOutline.find("CHUUT") != std::string::npos) {
                isConfident = true;
                return "Electrolytic Capacitor";
            }
            isConfident = true;
            return "Capacitor";
        }
    }

    // Diode
    if (uRef.rfind("D", 0) == 0 && (uRef.size() > 1 && std::isdigit(uRef[1]))) {
        if (uDesc.find("DIODE") != std::string::npos ||
            uDesc.find("DIODO") != std::string::npos ||
            uDesc.find("ZENER") != std::string::npos ||
            uDesc.find("RECTIFIER") != std::string::npos ||
            uDesc.find("TVS") != std::string::npos ||
            uDesc.find("SCHOTTKY") != std::string::npos ||
            uDesc.find("BAS5") != std::string::npos ||
            uDesc.find("TRANSZORB") != std::string::npos) {
            isConfident = true;
            return "Diode";
        }
    }

    // Transistor
    if ((uRef.rfind("TR", 0) == 0 || uRef.rfind("Q", 0) == 0) ||
        (uRef.rfind("T", 0) == 0 && uRef.size() > 1 && std::isdigit(uRef[1]))) {
        if (uDesc.find("TRANS") != std::string::npos ||
            uDesc.find("MOSFET") != std::string::npos ||
            uDesc.find("FET") != std::string::npos ||
            uDesc.find("BJT") != std::string::npos ||
            uDesc.find("NPN") != std::string::npos ||
            uDesc.find("PNP") != std::string::npos ||
            uDesc.find("N CHANNEL") != std::string::npos ||
            uDesc.find("P-CHANNEL") != std::string::npos ||
            uOutline.find("SOT23") != std::string::npos ||
            uOutline.find("D2PAK") != std::string::npos ||
            uOutline.find("SOT89") != std::string::npos) {
            isConfident = true;
            return "Transistor";
        }
    }

    // IC
    if ((uRef.rfind("IC", 0) == 0 || uRef.rfind("U", 0) == 0)) {
        if (uDesc.find("IC") != std::string::npos ||
            uDesc.find("OP AMP") != std::string::npos ||
            uDesc.find("AMPLIFIER") != std::string::npos ||
            uDesc.find("REG") != std::string::npos ||
            uDesc.find("CTRL") != std::string::npos ||
            uDesc.find("SYNCH") != std::string::npos ||
            uDesc.find("OPTOCOUPLER") != std::string::npos ||
            uDesc.find("DRIVER") != std::string::npos ||
            uOutline.find("MSOP") != std::string::npos ||
            uOutline.find("SOT553") != std::string::npos ||
            uOutline.find("SON6") != std::string::npos) {
            isConfident = true;
            return "IC";
        }
    }

    // Inductor
    if (uRef.rfind("L", 0) == 0 && (uRef.size() > 1 && std::isdigit(uRef[1]))) {
        if (uDesc.find("IND") != std::string::npos ||
            uDesc.find("CHOKE") != std::string::npos ||
            uDesc.find("COIL") != std::string::npos ||
            uOutline.rfind("LH", 0) == 0) {
            isConfident = true;
            return "Inductor";
        }
    }

    // LED
    if (uRef.rfind("LED", 0) == 0 || uRef.rfind("DL", 0) == 0 || uDesc.find("LED") != std::string::npos) {
        isConfident = true;
        return "LED";
    }

    // Other (es. Fuse / Fusibile)
    if (uRef.rfind("F", 0) == 0 && (uDesc.find("FUSE") != std::string::npos || uDesc.find("FUSIBILE") != std::string::npos)) {
        isConfident = true;
        return "Other";
    }

    // Connector
    if (uRef.rfind("J", 0) == 0 || uRef.rfind("CON", 0) == 0 || uDesc.find("CONNECTOR") != std::string::npos) {
        isConfident = true;
        return "Connector";
    }

    // Switch/Button
    if (uRef.rfind("SW", 0) == 0 || uRef.rfind("BTN", 0) == 0 || uDesc.find("SWITCH") != std::string::npos) {
        isConfident = true;
        return "Switch/Button";
    }

    isConfident = false;
    if (uDesc.find("RES") != std::string::npos) return "Resistor";
    if (uDesc.find("CAP") != std::string::npos) return "Capacitor";
    if (uDesc.find("DIOD") != std::string::npos) return "Diode";
    if (uDesc.find("TRANS") != std::string::npos) return "Transistor";
    if (uDesc.find("IC") != std::string::npos) return "IC";
    return "Other";
}

std::string AoiVerificationProcessor::askUserForClass(const std::string& ref,
                                                      const std::string& partNumber,
                                                      const std::string& outline,
                                                      const std::string& description) const {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "NUOVO COMPONENTE AOI DA CATALOGARE" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "Ref Designator: " << ref << std::endl;
    std::cout << "Codice Parte:   " << partNumber << std::endl;
    std::cout << "Outline/Pkg:    " << outline << std::endl;
    std::cout << "Descrizione:    " << description << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;
    std::cout << "A quale classe di componenti appartiene?" << std::endl;

    auto classes = getSupportedClasses();
    for (size_t i = 0; i < classes.size(); ++i) {
        std::cout << "  " << std::setw(2) << (i + 1) << ") " << classes[i] << std::endl;
    }

    while (true) {
        std::cout << "\nInserisci il numero (1-" << classes.size() << ") oppure il nome esatto: ";
        std::string input;
        if (!(std::cin >> input)) {
            std::cin.clear();
            return "Other";
        }

        try {
            int num = std::stoi(input);
            if (num >= 1 && num <= static_cast<int>(classes.size())) {
                return classes[num - 1];
            }
        } catch (...) {}

        std::string inputUpper = toUpper(input);
        for (const auto& c : classes) {
            if (toUpper(c) == inputUpper) {
                return c;
            }
        }

        std::cout << "Selezione non valida. Riprova." << std::endl;
    }
}

std::vector<AoiComponent> AoiVerificationProcessor::parseAoiFile(const std::string& aoiFilePath,
                                                                const std::string& mappingFilePath,
                                                                bool interactive) {
    std::vector<AoiComponent> components;

    if (mapping_table_.empty()) {
        loadMappingFile(mappingFilePath);
    }

    std::ifstream file(aoiFilePath);
    if (!file.is_open()) {
        std::cerr << "[AOI] ERRORE: Impossibile aprire il file AOI: " << aoiFilePath << std::endl;
        return components;
    }

    std::cout << "[AOI] Analisi del file coordinate AOI: " << aoiFilePath << std::endl;

    std::string line;
    int autoAddedCount = 0;
    int userPromptCount = 0;

    while (std::getline(file, line)) {
        std::string s = trim(line);
        if (s.empty()) continue;

        if (s.rfind("Codice", 0) == 0 || s.rfind("Ref.", 0) == 0) {
            continue;
        }

        std::istringstream iss(s);
        std::string ref, xStr, yStr, qStr, side, code, outline;
        if (!(iss >> ref >> xStr >> yStr >> qStr >> side >> code)) {
            continue;
        }

        double x_mm = 0.0, y_mm = 0.0, q_deg = 0.0;
        try {
            x_mm = std::stod(xStr);
            y_mm = std::stod(yStr);
            q_deg = std::stod(qStr);
        } catch (...) {
            continue;
        }

        if (iss >> outline) {
        } else {
            outline = "-";
        }

        std::string desc;
        std::getline(iss, desc);
        desc = trim(desc);

        AoiComponent comp;
        comp.ref_des = ref;
        comp.x_mm = x_mm;
        comp.y_mm = y_mm;
        comp.rot_deg = q_deg;
        comp.side = side;
        comp.part_number = code;
        comp.outline = outline;
        comp.description = desc;
        comp.raw_line = s;

        std::string assignedClass = "";
        auto itRaw = mapping_table_.find(s);
        if (itRaw != mapping_table_.end()) {
            assignedClass = itRaw->second;
        } else {
            auto itRef = mapping_table_.find("REF:" + ref);
            if (itRef != mapping_table_.end()) {
                assignedClass = itRef->second;
            } else {
                auto itCode = mapping_table_.find("CODE:" + code);
                if (itCode != mapping_table_.end()) {
                    assignedClass = itCode->second;
                }
            }
        }

        if (assignedClass.empty()) {
            bool isConfident = false;
            std::string inferred = inferClassFromKeywords(ref, outline, desc, isConfident);

            if (isConfident) {
                assignedClass = inferred;
                saveMappingEntry(mappingFilePath, s, assignedClass);
                autoAddedCount++;
            } else if (interactive) {
                assignedClass = askUserForClass(ref, code, outline, desc);
                saveMappingEntry(mappingFilePath, s, assignedClass);
                userPromptCount++;
            } else {
                assignedClass = inferred;
            }
        }

        comp.assigned_class = assignedClass;
        components.push_back(comp);
    }

    std::cout << "[AOI] File analizzato con successo: " << components.size() << " componenti nominali." << std::endl;
    if (autoAddedCount > 0) {
        std::cout << "  - " << autoAddedCount << " nuovi riferimenti associati automaticamente con certezza al file mapping." << std::endl;
    }
    if (userPromptCount > 0) {
        std::cout << "  - " << userPromptCount << " riferimenti classificati con l'utente e salvati nel file mapping." << std::endl;
    }

    return components;
}

void AoiVerificationProcessor::getNominalOutlineDimensions(const std::string& outline, double& width_mm, double& height_mm) {
    std::string uOut = toUpper(outline);

    if (uOut.find("0603") != std::string::npos) {
        width_mm = 1.6; height_mm = 0.8;
    } else if (uOut.find("0805") != std::string::npos) {
        width_mm = 2.0; height_mm = 1.25;
    } else if (uOut.find("1206") != std::string::npos) {
        width_mm = 3.2; height_mm = 1.6;
    } else if (uOut.find("1210") != std::string::npos) {
        width_mm = 3.2; height_mm = 2.5;
    } else if (uOut.find("1812") != std::string::npos) {
        width_mm = 4.5; height_mm = 3.2;
    } else if (uOut.find("2512") != std::string::npos) {
        width_mm = 6.3; height_mm = 3.2;
    } else if (uOut.find("SOT23") != std::string::npos || uOut.find("TO236") != std::string::npos) {
        width_mm = 2.9; height_mm = 2.4;
    } else if (uOut.find("SOT553") != std::string::npos) {
        width_mm = 1.6; height_mm = 1.6;
    } else if (uOut.find("SOT89") != std::string::npos) {
        width_mm = 4.5; height_mm = 2.5;
    } else if (uOut.find("SOT685") != std::string::npos || uOut.find("SOT1212") != std::string::npos) {
        width_mm = 5.0; height_mm = 6.0;
    } else if (uOut.find("MSOP8") != std::string::npos || uOut.find("MSOP16") != std::string::npos) {
        width_mm = 3.0; height_mm = 4.9;
    } else if (uOut.find("D2PAK") != std::string::npos || uOut.find("TO-263") != std::string::npos) {
        width_mm = 10.0; height_mm = 15.0;
    } else if (uOut.find("SC79") != std::string::npos) {
        width_mm = 1.6; height_mm = 0.8;
    } else if (uOut.find("DO-214") != std::string::npos || uOut.find("DO214") != std::string::npos) {
        width_mm = 5.4; height_mm = 3.6;
    } else if (uOut.find("CHUUT") != std::string::npos) {
        width_mm = 10.0; height_mm = 10.0;
    } else if (uOut.find("RHMS1") != std::string::npos) {
        width_mm = 3.6; height_mm = 1.5;
    } else if (uOut.find("SON6") != std::string::npos) {
        width_mm = 2.0; height_mm = 2.0;
    } else if (uOut.find("SO4") != std::string::npos) {
        width_mm = 4.6; height_mm = 4.6;
    } else if (uOut.find("LH4") != std::string::npos || uOut.find("LH6") != std::string::npos) {
        width_mm = 7.0; height_mm = 7.0;
    } else {
        width_mm = 2.0; height_mm = 1.25;
    }
}

std::map<std::string, std::pair<double, double>> AoiVerificationProcessor::computePartNumberDimensionsFromGerber(
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<GerberPad>& pads) {

    std::map<std::string, std::vector<std::pair<double, double>>> codePadSamples;

    if (!pads.empty()) {
        for (const auto& a : aoiComps) {
            std::vector<GerberPad> nearby;
            for (const auto& p : pads) {
                double dist = std::hypot(p.center_mm.x - a.x_mm, p.center_mm.y - a.y_mm);
                if (dist <= 3.8) {
                    nearby.push_back(p);
                }
            }

            if (!nearby.empty()) {
                double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
                for (const auto& p : nearby) {
                    double pxMin = p.center_mm.x - p.width_mm * 0.5;
                    double pxMax = p.center_mm.x + p.width_mm * 0.5;
                    double pyMin = p.center_mm.y - p.height_mm * 0.5;
                    double pyMax = p.center_mm.y + p.height_mm * 0.5;
                    if (pxMin < minX) minX = pxMin;
                    if (pxMax > maxX) maxX = pxMax;
                    if (pyMin < minY) minY = pyMin;
                    if (pyMax > maxY) maxY = pyMax;
                }

                double w_mm = std::max(0.5, maxX - minX);
                double h_mm = std::max(0.5, maxY - minY);

                // Normalizza orientamento intrinseco in base all'angolo nominale Q
                double q = std::fmod(std::abs(a.rot_deg), 180.0);
                if (q > 45.0 && q < 135.0) {
                    std::swap(w_mm, h_mm);
                }

                // Sanity check: se l'ingombro supera 30 mm o è anomalo, usa outline
                if (w_mm > 30.0 || h_mm > 30.0 || w_mm < 0.4 || h_mm < 0.4) {
                    getNominalOutlineDimensions(a.outline, w_mm, h_mm);
                }

                codePadSamples[a.part_number].push_back({w_mm, h_mm});
            }
        }
    }

    std::map<std::string, std::pair<double, double>> canonicalDimensions;

    for (const auto& a : aoiComps) {
        if (canonicalDimensions.find(a.part_number) != canonicalDimensions.end()) {
            continue;
        }

        auto it = codePadSamples.find(a.part_number);
        if (it != codePadSamples.end() && !it->second.empty()) {
            std::vector<double> ws, hs;
            for (const auto& s : it->second) {
                ws.push_back(s.first);
                hs.push_back(s.second);
            }
            std::sort(ws.begin(), ws.end());
            std::sort(hs.begin(), hs.end());
            double medW = ws[ws.size() / 2];
            double medH = hs[hs.size() / 2];
            canonicalDimensions[a.part_number] = {medW, medH};
        } else {
            double defW = 2.0, defH = 1.25;
            getNominalOutlineDimensions(a.outline, defW, defH);
            canonicalDimensions[a.part_number] = {defW, defH};
        }
    }

    std::cout << "[AOI] Calcolate dimensioni canoniche per " << canonicalDimensions.size()
              << " codici componente unici dai layer Gerber." << std::endl;
    return canonicalDimensions;
}

AoiAlignmentTransform AoiVerificationProcessor::findBestOriginAndDistortionAlignment(
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    double pixels_per_mm,
    double imgWidth,
    double imgHeight,
    double initial_tolerance_mm) const {

    (void)imgWidth;
    (void)imgHeight;

    AoiAlignmentTransform bestTransform;
    bestTransform.valid = false;
    bestTransform.total_aoi_count = static_cast<int>(aoiComps.size());

    if (aoiComps.empty() || detections.empty() || pixels_per_mm <= 1e-4) {
        std::cerr << "[AOI] Dati insufficienti per l'allineamento origine." << std::endl;
        return bestTransform;
    }

    const double s = pixels_per_mm;
    const double tol_px = initial_tolerance_mm * s;

    struct SymmetryConfig {
        std::string name;
        double mxx, mxy, myx, myy;
    };

    const std::vector<SymmetryConfig> symConfigs = {
        {"rot0",    1.0,  0.0,  0.0,  1.0},
        {"rot90",   0.0, -1.0,  1.0,  0.0},
        {"rot180", -1.0,  0.0,  0.0, -1.0},
        {"rot270",  0.0,  1.0, -1.0,  0.0},
        {"flip_x", -1.0,  0.0,  0.0,  1.0},
        {"flip_y",  1.0,  0.0,  0.0, -1.0},
        {"diag1",   0.0,  1.0,  1.0,  0.0},
        {"diag2",   0.0, -1.0, -1.0,  0.0}
    };

    double bestScore = -1.0;

    // FASE 1: Trova l'orientamento ortogonale e l'origine iniziale
    for (const auto& sym : symConfigs) {
        const double binSize = 6.0;
        std::map<std::pair<int, int>, double> binVotes;

        for (const auto& a : aoiComps) {
            double ax_px = s * (sym.mxx * a.x_mm + sym.mxy * a.y_mm);
            double ay_px = s * (sym.myx * a.x_mm + sym.myy * a.y_mm);

            for (const auto& d : detections) {
                double weight = 0.5;
                if (d.label == a.assigned_class) weight = 3.0;
                else if (d.label == "?") weight = 1.2;

                double dtx = d.center.x - ax_px;
                double dty = d.center.y - ay_px;
                int bx = static_cast<int>(std::round(dtx / binSize));
                int by = static_cast<int>(std::round(dty / binSize));
                binVotes[{bx, by}] += weight;
            }
        }

        if (binVotes.empty()) continue;

        std::vector<std::pair<std::pair<int, int>, double>> topBins(binVotes.begin(), binVotes.end());
        std::sort(topBins.begin(), topBins.end(), [](const auto& a, const auto& b) {
            return a.second > b.second;
        });

        int checkCount = std::min(static_cast<int>(topBins.size()), 4);
        for (int k = 0; k < checkCount; ++k) {
            double cand_tx = topBins[k].first.first * binSize;
            double cand_ty = topBins[k].first.second * binSize;

            int inliers = 0;
            double score = 0.0;
            double errSum = 0.0;

            for (const auto& a : aoiComps) {
                double px = s * (sym.mxx * a.x_mm + sym.mxy * a.y_mm) + cand_tx;
                double py = s * (sym.myx * a.x_mm + sym.myy * a.y_mm) + cand_ty;

                double minD = 1e9;
                std::string bestDetLabel = "";
                for (const auto& d : detections) {
                    double dist = std::hypot(px - d.center.x, py - d.center.y);
                    if (dist < minD) {
                        minD = dist;
                        bestDetLabel = d.label;
                    }
                }

                if (minD <= tol_px) {
                    inliers++;
                    errSum += minD;
                    if (bestDetLabel == a.assigned_class) score += 3.0 - (minD / tol_px);
                    else if (bestDetLabel == "?") score += 1.5 - (minD / tol_px);
                    else score += 0.8 - (0.5 * minD / tol_px);
                }
            }

            if (score > bestScore) {
                bestScore = score;
                bestTransform.valid = true;
                bestTransform.orientation_name = sym.name;
                bestTransform.mxx = sym.mxx;
                bestTransform.mxy = sym.mxy;
                bestTransform.myx = sym.myx;
                bestTransform.myy = sym.myy;
                bestTransform.tx = cand_tx;
                bestTransform.ty = cand_ty;
                bestTransform.scale_px_per_mm = s;
                bestTransform.inlier_count = inliers;
                bestTransform.inlier_ratio = static_cast<double>(inliers) / aoiComps.size();
                bestTransform.avg_error_px = (inliers > 0) ? (errSum / inliers) : 0.0;
                bestTransform.avg_error_mm = bestTransform.avg_error_px / s;
            }
        }
    }

    // FASE 2: Modellazione e correzione della distorsione ottica e prospettica (Omografia 3x3)
    // Utilizza le corrispondenze certe distribuite sulla PCB per stimare il modello proiettivo esatto
    if (bestTransform.valid && bestTransform.inlier_count >= 8) {
        std::vector<cv::Point2f> srcPointsMm;
        std::vector<cv::Point2f> dstPointsPx;

        for (const auto& a : aoiComps) {
            double px_lin = s * (bestTransform.mxx * a.x_mm + bestTransform.mxy * a.y_mm) + bestTransform.tx;
            double py_lin = s * (bestTransform.myx * a.x_mm + bestTransform.myy * a.y_mm) + bestTransform.ty;

            double minD = 1e9;
            cv::Point2f bestDetPt;
            for (const auto& d : detections) {
                double dist = std::hypot(px_lin - d.center.x, py_lin - d.center.y);
                if (dist < minD) {
                    minD = dist;
                    bestDetPt = d.center;
                }
            }

            // Includi tutti i punti candidati entro 35 pixel per catturare la distorsione radiale ai bordi
            if (minD <= 35.0) {
                srcPointsMm.push_back(cv::Point2f(static_cast<float>(a.x_mm), static_cast<float>(a.y_mm)));
                dstPointsPx.push_back(bestDetPt);
            }
        }

        if (srcPointsMm.size() >= 12) {
            cv::Mat inlierMask;
            cv::Mat H = cv::findHomography(srcPointsMm, dstPointsPx, cv::RANSAC, 18.0, inlierMask);
            if (!H.empty()) {
                bestTransform.homography_3x3 = H;
                bestTransform.homography_inv_3x3 = H.inv();
                bestTransform.has_distortion_model = true;

                // Ricalcola corrispondenze ed errore medio tenendo conto del modello di distorsione
                int distInliers = 0;
                double distErrSum = 0.0;
                for (const auto& a : aoiComps) {
                    cv::Point2f pDist = bestTransform.aoiToPixel(a.x_mm, a.y_mm);
                    double minD = 1e9;
                    for (const auto& d : detections) {
                        double dist = cv::norm(pDist - d.center);
                        if (dist < minD) minD = dist;
                    }
                    if (minD <= tol_px) {
                        distInliers++;
                        distErrSum += minD;
                    }
                }

                bestTransform.inlier_count = distInliers;
                bestTransform.inlier_ratio = static_cast<double>(distInliers) / aoiComps.size();
                bestTransform.avg_error_px = (distInliers > 0) ? (distErrSum / distInliers) : 0.0;
                bestTransform.avg_error_mm = bestTransform.avg_error_px / s;
            }
        }
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << "[AOI] TRASFORMAZIONE ORIGINE E CORREZIONE DISTORSIONE:" << std::endl;
    std::cout << "  - Orientazione trovata:     " << bestTransform.orientation_name << std::endl;
    std::cout << "  - Modello distorsione:      " << (bestTransform.has_distortion_model ? "Attivo (Omografia 3x3)" : "Lineare") << std::endl;
    std::cout << "  - Corrispondenze agganciate:" << bestTransform.inlier_count << " / "
              << bestTransform.total_aoi_count << " componenti ("
              << std::fixed << std::setprecision(1) << (bestTransform.inlier_ratio * 100.0) << "%)" << std::endl;
    std::cout << "  - Errore medio residuo:     " << std::fixed << std::setprecision(2)
              << bestTransform.avg_error_px << " px (" << std::setprecision(3)
              << bestTransform.avg_error_mm << " mm)" << std::endl;
    std::cout << "========================================================" << std::endl;

    return bestTransform;
}

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

        double totalRot = a.rot_deg + transform.residual_angle_deg;

        if (bestIdx >= 0) {
            pool[bestIdx].used = true;
            const auto& matched = pool[bestIdx];

            a.matched = true;
            a.matched_detection_id = matched.id;
            double realDist = cv::norm(nomPx - matched.center);
            a.match_distance_px = realDist;
            a.match_distance_mm = realDist / s;

            // REQUISITO: Utilizza DIRETTAMENTE i riquadri trovati in DL per i componenti riconosciuti
            vc.center = matched.center;
            vc.box = matched.box;
            vc.rotated_box = matched.rbox;
            if (vc.rotated_box.size.width <= 0.0f || vc.rotated_box.size.height <= 0.0f) {
                vc.rotated_box = cv::RotatedRect(matched.center, matched.box.size(), static_cast<float>(totalRot));
            }
            vc.shape = matched.shape;
            vc.radius = matched.radius;
            vc.area = matched.area;
            vc.nominal_width_mm = matched.box.width / s;
            vc.nominal_height_mm = matched.box.height / s;

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
        double totalRot = a.rot_deg + transform.residual_angle_deg;

        // Cerca nella sovrapposizione CV il componente geometrico più vicino alla coordinata nominale
        int bestCvIdx = -1;
        double bestCvDist = 1e9;
        double cvSearchTol = tol_px * 1.1;

        for (size_t k = 0; k < cvPool.size(); ++k) {
            double dist = cv::norm(nomPx - cvPool[k].center);
            if (dist <= cvSearchTol && dist < bestCvDist) {
                bestCvDist = dist;
                bestCvIdx = static_cast<int>(k);
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
            vc.nominal_width_mm = vc.box.width / s;
            vc.nominal_height_mm = vc.box.height / s;
            vc.source_name = "RECOVERED_CV_SUPERPOSITION";
            vc.notes = "Componente recuperato tramite sovrapposizione multi-foto CV (" + a.ref_des + ")";
            vc.offset_error_px = bestCvDist;
            vc.offset_error_mm = bestCvDist / s;

        } else {
            // Nessun candidato CV nelle vicinanze: recupero con dimensioni canoniche DL per stesso part_number
            float w_px = 18.0f, h_px = 30.0f;
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

            vc.center = nomPx;
            vc.shape = (a.outline.find("CHUUT") != std::string::npos) ? ShapeType::CIRCLE : ShapeType::RECTANGLE;
            vc.radius = (vc.shape == ShapeType::CIRCLE) ? std::max(w_px, h_px) * 0.5f : 0.0f;
            vc.rotated_box = cv::RotatedRect(nomPx, cv::Size2f(w_px, h_px), static_cast<float>(totalRot));
            vc.box = vc.rotated_box.boundingRect2f();
            vc.area = vc.box.width * vc.box.height;
            vc.nominal_width_mm = w_px / s;
            vc.nominal_height_mm = h_px / s;
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

AoiVerificationResult AoiVerificationProcessor::verifyAndClassify(
    const cv::Mat& pcbImage,
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    const std::vector<GerberPad>& /*gerberPads*/,
    const AoiAlignmentTransform& transform,
    double tolerance_mm) const {

    std::vector<CvComponent> emptyCv;
    return verifyAndClassify(pcbImage, aoiComps, detections, emptyCv, transform, tolerance_mm);
}

cv::Mat AoiVerificationProcessor::createSideLegend(
    int targetHeight,
    const std::map<std::string, int>& counts,
    int totalComps,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extraStats) {

    int panelW = 340;
    cv::Mat panel(targetHeight, panelW, CV_8UC3, cv::Scalar(24, 24, 24));

    int y = 30;
    cv::putText(panel, title, cv::Point(14, y), cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    y += 18;
    cv::line(panel, cv::Point(14, y), cv::Point(panelW - 14, y), cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    y += 24;

    std::string totTxt = "Totale componenti AOI: " + std::to_string(totalComps);
    cv::putText(panel, totTxt, cv::Point(16, y), cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    y += 22;

    if (!extraStats.empty()) {
        for (const auto& [label, cnt] : extraStats) {
            std::string stTxt = label + ": " + std::to_string(cnt);
            cv::putText(panel, stTxt, cv::Point(16, y), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
            y += 18;
        }
        y += 8;
        cv::line(panel, cv::Point(14, y), cv::Point(panelW - 14, y), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
        y += 22;
    }

    if (!counts.empty()) {
        cv::putText(panel, "Distribuzione Classi:", cv::Point(16, y), cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        y += 22;

        for (const auto& [lbl, cnt] : counts) {
            if (cnt <= 0 || y > targetHeight - 25) continue;

            cv::Scalar col = cv::Scalar(180, 180, 180);
            auto it = palette.find(lbl);
            if (it != palette.end()) col = it->second;

            cv::rectangle(panel, cv::Rect(16, y - 11, 14, 14), col, cv::FILLED);
            cv::rectangle(panel, cv::Rect(16, y - 11, 14, 14), cv::Scalar(240, 240, 240), 1);

            std::string lineTxt = lbl + ": " + std::to_string(cnt);
            cv::putText(panel, lineTxt, cv::Point(38, y), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
            y += 20;
        }
    }

    return panel;
}

cv::Mat AoiVerificationProcessor::renderBestAlignmentOverlay(
    const cv::Mat& pcbImage,
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    const AoiAlignmentTransform& transform) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();

    // NOTA: I componenti extra NON vengono disegnati come richiesto!
    // Disegna solo i punti teorici AOI e i collegamenti ai componenti associati
    int matchedCount = 0;
    for (const auto& a : aoiComps) {
        cv::Point2f nom = transform.aoiToPixel(a.x_mm, a.y_mm);

        cv::Scalar nomCol = COLOR_EXACT_MATCH;
        if (!a.matched) {
            nomCol = cv::Scalar(0, 0, 255);
        }

        cv::drawMarker(canvas, nom, nomCol, cv::MARKER_CROSS, 8, 1, cv::LINE_AA);

        if (a.matched && a.matched_detection_id > 0) {
            matchedCount++;
            for (const auto& d : detections) {
                if (d.id == a.matched_detection_id) {
                    // Disegna solo il box del componente associato
                    cv::rectangle(canvas, d.box, cv::Scalar(80, 140, 255), 1, cv::LINE_AA);
                    cv::line(canvas, nom, d.center, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
                    break;
                }
            }
        }

        cv::putText(canvas, a.ref_des, cv::Point(static_cast<int>(nom.x + 3), static_cast<int>(nom.y - 3)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.28, nomCol, 1, cv::LINE_AA);
    }

    // Banner superiore scuro
    cv::Rect bannerRect(0, 0, canvas.cols, 42);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.85, banner, 0.15, 0.0, banner);

    std::string headerTxt = "Corrispondenza Coordinate AOI vs Immagine Reale (Correzione Distorsione Ottica): " +
        std::to_string(matchedCount) + "/" + std::to_string(aoiComps.size()) + " (" +
        transform.orientation_name + ", Offset medio: " +
        std::to_string(transform.avg_error_mm).substr(0, 5) + " mm)";
    cv::putText(canvas, headerTxt, cv::Point(14, 27), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    return canvas;
}

cv::Mat AoiVerificationProcessor::renderVerifiedTypesOverlay(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& verifiedComps) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();
    std::map<std::string, int> counts;
    int drawnCount = 0;

    for (const auto& c : verifiedComps) {
        // COMPONENTI EXTRA NON RAPPRESENTARLI!
        if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) {
            continue;
        }

        drawnCount++;
        counts[c.label]++;

        cv::Scalar col = cv::Scalar(180, 180, 180);
        auto it = CLASS_COLORS.find(c.label);
        if (it != CLASS_COLORS.end()) col = it->second;

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(2, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
        }
        cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        // REQUISITO UTENTE: Nelle immagini di output riporta solo il codice componente (es. R301 non R301-RESISTOR)
        std::string txt = (!c.ref_des.empty() && c.ref_des != "N/A") ? c.ref_des : c.label;
        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.35, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 6));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 14);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(18, 18, 18), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect bannerRect(0, 0, canvas.cols, 38);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, banner, 0.18, 0.0, banner);
    std::string btext = "Verifica AOI: Riconoscimento DL + Sovrapposizione CV (Extra esclusi)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // Leggenda laterale
    cv::Mat legend = createSideLegend(canvas.rows, counts, drawnCount,
                                      CLASS_COLORS, "Classi Verificate", {});

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
}

cv::Mat AoiVerificationProcessor::renderVerifiedElementsOverlay(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& verifiedComps) const {

    if (pcbImage.empty()) return {};

    cv::Mat canvas = pcbImage.clone();
    int nExact = 0, nCorrected = 0, nRecovered = 0;

    for (const auto& c : verifiedComps) {
        // COMPONENTI EXTRA NON RAPPRESENTARLI!
        if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) {
            continue;
        }

        cv::Scalar col;
        std::string stStr;

        if (c.status == AoiMatchStatus::MATCH_EXACT_CLASS) {
            col = COLOR_EXACT_MATCH;
            stStr = "OK";
            nExact++;
        } else if (c.status == AoiMatchStatus::CLASS_CORRECTED) {
            col = COLOR_CLASS_CORRECTED;
            stStr = "CORRETTO";
            nCorrected++;
        } else {
            col = COLOR_RECOVERED;
            stStr = "RECUPERATO";
            nRecovered++;
        }

        if (c.shape == ShapeType::CIRCLE) {
            int rad = std::max(2, static_cast<int>(std::round(c.radius)));
            cv::circle(canvas, c.center, rad, col, 2, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int j = 0; j < 4; ++j) {
                cv::line(canvas, pts[j], pts[(j + 1) % 4], col, 2, cv::LINE_AA);
            }
        }
        cv::circle(canvas, c.center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        // REQUISITO UTENTE: Nelle immagini di output riporta solo il codice componente (es. R301)
        std::string txt = (!c.ref_des.empty() && c.ref_des != "N/A") ? c.ref_des : ("#" + std::to_string(c.id));
        int baseLine = 0;
        cv::Size sz = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.34, 1, &baseLine);
        cv::Point pt(static_cast<int>(c.center.x - sz.width / 2),
                     static_cast<int>(c.center.y - 6));
        if (pt.y < 12) pt.y = static_cast<int>(c.center.y + 14);

        cv::Rect bg(pt.x - 2, pt.y - sz.height - 2, sz.width + 4, sz.height + 4);
        bg &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (bg.width > 0 && bg.height > 0) {
            cv::rectangle(canvas, bg, cv::Scalar(18, 18, 18), cv::FILLED);
            cv::putText(canvas, txt, pt, cv::FONT_HERSHEY_SIMPLEX, 0.34, col, 1, cv::LINE_AA);
        }
    }

    // Banner superiore
    cv::Rect bannerRect(0, 0, canvas.cols, 38);
    cv::Mat banner = canvas(bannerRect);
    cv::Mat dark(bannerRect.size(), CV_8UC3, cv::Scalar(15, 15, 15));
    cv::addWeighted(dark, 0.82, banner, 0.18, 0.0, banner);
    std::string btext = "Verifica Elementi AOI: Verde=OK, Arancione=Corretto, Ciano=Recuperato (Extra esclusi)";
    cv::putText(canvas, btext, cv::Point(14, 25), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::vector<std::pair<std::string, int>> extraStats = {
        {"Match Esatti (OK)", nExact},
        {"Classi Corrette",   nCorrected},
        {"Recuperati da AOI", nRecovered}
    };

    std::map<std::string, cv::Scalar> statusPalette = {
        {"Match Esatti (OK)", COLOR_EXACT_MATCH},
        {"Classi Corrette",   COLOR_CLASS_CORRECTED},
        {"Recuperati da AOI", COLOR_RECOVERED}
    };

    int totalNominalRepresented = nExact + nCorrected + nRecovered;
    cv::Mat legend = createSideLegend(canvas.rows, {}, totalNominalRepresented,
                                      statusPalette, "Stato Riconoscimento", extraStats);

    cv::Mat output(std::max(canvas.rows, legend.rows), canvas.cols + legend.cols, CV_8UC3);
    canvas.copyTo(output(cv::Rect(0, 0, canvas.cols, canvas.rows)));
    legend.copyTo(output(cv::Rect(canvas.cols, 0, legend.cols, legend.rows)));
    return output;
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
            << "nominal_x_px,nominal_y_px,offset_px,offset_mm,aoi_part_number,aoi_outline,notes\n";

        for (const auto& c : result.verified_components) {
            if (c.status == AoiMatchStatus::UNMATCHED_DETECTION) continue;

            std::string stStr = "MATCH_EXACT";
            if (c.status == AoiMatchStatus::CLASS_CORRECTED) stStr = "CLASS_CORRECTED";
            else if (c.status == AoiMatchStatus::RECOVERED_BY_AOI) stStr = "RECOVERED_BY_AOI";

            double nomX_mm = 0.0, nomY_mm = 0.0;
            auto itAoi = std::find_if(result.aoi_components.begin(), result.aoi_components.end(), [&](const AoiComponent& a) {
                return a.ref_des == c.ref_des;
            });
            if (itAoi != result.aoi_components.end()) {
                nomX_mm = itAoi->x_mm;
                nomY_mm = itAoi->y_mm;
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
                << c.notes << "\"\n";
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
        for (const std::string& ext : {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"}) {
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
