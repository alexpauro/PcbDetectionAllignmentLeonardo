#include "aoi_internal.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace pcb {

bool AoiVerificationProcessor::loadMappingFile(const std::string& mappingFilePath) {
    mapping_table_.clear();
    datasheet_table_.clear();
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
            std::string rightPart = trim(s.substr(arrowPos + 10));

            std::string className = rightPart;
            double width_mm = 0.0, height_mm = 0.0, thickness_mm = 0.0;
            std::string source = "";

            size_t pipe1 = rightPart.find('|');
            if (pipe1 != std::string::npos) {
                className = trim(rightPart.substr(0, pipe1));
                std::string remainder = rightPart.substr(pipe1 + 1);

                size_t dimPos = remainder.find("Dimensions (mm):");
                size_t pipe2 = remainder.find('|', dimPos != std::string::npos ? dimPos : 0);

                if (dimPos != std::string::npos) {
                    std::string dimStr;
                    if (pipe2 != std::string::npos) {
                        dimStr = trim(remainder.substr(dimPos + 16, pipe2 - (dimPos + 16)));
                        std::string srcStr = remainder.substr(pipe2 + 1);
                        size_t srcPos = srcStr.find("Source:");
                        if (srcPos != std::string::npos) {
                            source = trim(srcStr.substr(srcPos + 7));
                        } else {
                            source = trim(srcStr);
                        }
                    } else {
                        dimStr = trim(remainder.substr(dimPos + 16));
                    }

                    std::vector<std::string> dimParts;
                    std::stringstream ssDim(dimStr);
                    std::string segment;
                    while (std::getline(ssDim, segment, 'x')) {
                        std::string t = trim(segment);
                        if (!t.empty()) dimParts.push_back(t);
                    }
                    try {
                        if (dimParts.size() >= 1) width_mm = std::stod(dimParts[0]);
                        if (dimParts.size() >= 2) height_mm = std::stod(dimParts[1]);
                        if (dimParts.size() >= 3) thickness_mm = std::stod(dimParts[2]);
                    } catch (...) {}
                }
            }

            ComponentDatasheetInfo info;
            info.assigned_class = className;
            info.width_mm = width_mm;
            info.height_mm = height_mm;
            info.thickness_mm = thickness_mm;
            info.source = source;

            mapping_table_[leftPart] = className;
            datasheet_table_[leftPart] = info;

            std::istringstream iss(leftPart);
            std::string ref, x, y, q, side, code;
            if (iss >> ref >> x >> y >> q >> side >> code) {
                if (mapping_table_.find("REF:" + ref) == mapping_table_.end()) {
                    mapping_table_["REF:" + ref] = className;
                    datasheet_table_["REF:" + ref] = info;
                }
                if (mapping_table_.find("CODE:" + code) == mapping_table_.end()) {
                    mapping_table_["CODE:" + code] = className;
                    datasheet_table_["CODE:" + code] = info;
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
                                                const std::string& assignedClass,
                                                double width_mm,
                                                double height_mm,
                                                double thickness_mm,
                                                const std::string& source) {
    std::ofstream outfile(mappingFilePath, std::ios::app);
    if (!outfile.is_open()) {
        std::cerr << "[AOI] Impossibile aggiornare il file di mapping: " << mappingFilePath << std::endl;
        return false;
    }

    std::ostringstream oss;
    oss << rawLine << " --> Class: " << assignedClass;
    if (width_mm > 0.0 && height_mm > 0.0) {
        oss << " | Dimensions (mm): " << width_mm << " x " << height_mm;
        if (thickness_mm > 0.0) {
            oss << " x " << thickness_mm;
        }
    }
    if (!source.empty()) {
        oss << " | Source: " << source;
    }

    std::string entry = oss.str();
    outfile << entry << "\n";
    outfile.flush();

    mapping_lines_raw_.push_back(entry);
    mapping_table_[rawLine] = assignedClass;

    ComponentDatasheetInfo info;
    info.assigned_class = assignedClass;
    info.width_mm = width_mm;
    info.height_mm = height_mm;
    info.thickness_mm = thickness_mm;
    info.source = source;
    datasheet_table_[rawLine] = info;

    std::istringstream iss(rawLine);
    std::string ref, x, y, q, side, code;
    if (iss >> ref >> x >> y >> q >> side >> code) {
        mapping_table_["REF:" + ref] = assignedClass;
        mapping_table_["CODE:" + code] = assignedClass;
        datasheet_table_["REF:" + ref] = info;
        datasheet_table_["CODE:" + code] = info;
    }

    std::cout << "[AOI] Nuova associazione salvata nel file di mapping: "
              << ref << " -> " << assignedClass;
    if (width_mm > 0.0 && height_mm > 0.0) {
        std::cout << " [" << width_mm << " x " << height_mm << " mm]";
    }
    std::cout << std::endl;
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

    // 1. Prefisso del Reference Designator (standard IPC-7351)
    if (uRef.rfind("R", 0) == 0 && uRef.rfind("RN", 0) != 0) {
        if (uDesc.find("RES") != std::string::npos || uDesc.find("OHM") != std::string::npos || uOutline.find("0603") != std::string::npos || uOutline.find("0805") != std::string::npos) {
            isConfident = true;
            return "Resistor";
        }
        return "Resistor";
    }
    if (uRef.rfind("C", 0) == 0) {
        if (uDesc.find("ELECTROLYTIC") != std::string::npos || uDesc.find("EL.") != std::string::npos || uOutline.find("CHUUT") != std::string::npos) {
            isConfident = true;
            return "Electrolytic Capacitor";
        }
        if (uDesc.find("CAP") != std::string::npos || uDesc.find("FARAD") != std::string::npos || uDesc.find("PF") != std::string::npos || uDesc.find("UF") != std::string::npos || uDesc.find("NF") != std::string::npos) {
            isConfident = true;
            return "Capacitor";
        }
        return "Capacitor";
    }
    if (uRef.rfind("L", 0) == 0) {
        if (uDesc.find("IND") != std::string::npos || uDesc.find("COIL") != std::string::npos || uDesc.find("HENRY") != std::string::npos || uDesc.find("UH") != std::string::npos) {
            isConfident = true;
            return "Inductor";
        }
        return "Inductor";
    }
    if (uRef.rfind("D", 0) == 0) {
        if (uDesc.find("LED") != std::string::npos || uOutline.find("LED") != std::string::npos) {
            isConfident = true;
            return "LED";
        }
        if (uDesc.find("DIODE") != std::string::npos || uDesc.find("RECTIFIER") != std::string::npos || uDesc.find("SCHOTTKY") != std::string::npos || uDesc.find("ZENER") != std::string::npos) {
            isConfident = true;
            return "Diode";
        }
        return "Diode";
    }
    if (uRef.rfind("LED", 0) == 0 || uRef.rfind("DL", 0) == 0) {
        isConfident = true;
        return "LED";
    }
    if (uRef.rfind("Q", 0) == 0 || uRef.rfind("TR", 0) == 0) {
        if (uDesc.find("TRANSISTOR") != std::string::npos || uDesc.find("MOSFET") != std::string::npos || uDesc.find("NPN") != std::string::npos || uDesc.find("PNP") != std::string::npos) {
            isConfident = true;
            return "Transistor";
        }
        return "Transistor";
    }
    if (uRef.rfind("IC", 0) == 0 || uRef.rfind("U", 0) == 0) {
        isConfident = true;
        return "IC";
    }
    if (uRef.rfind("Y", 0) == 0 || uRef.rfind("X", 0) == 0 || uRef.rfind("OSC", 0) == 0) {
        isConfident = true;
        return "Oscillator";
    }
    if (uRef.rfind("J", 0) == 0 || uRef.rfind("P", 0) == 0 || uRef.rfind("CN", 0) == 0) {
        isConfident = true;
        return "Connector";
    }
    if (uRef.rfind("SW", 0) == 0 || uRef.rfind("PB", 0) == 0 || uRef.rfind("KEY", 0) == 0) {
        isConfident = true;
        return "Switch/Button";
    }

    // 2. Analisi della descrizione
    if (uDesc.find("ELECTROLYTIC") != std::string::npos) {
        isConfident = true;
        return "Electrolytic Capacitor";
    }
    if (uDesc.find("RESISTOR") != std::string::npos || uDesc.find("RES ") != std::string::npos) {
        isConfident = true;
        return "Resistor";
    }
    if (uDesc.find("CAPACITOR") != std::string::npos || uDesc.find("CAP ") != std::string::npos) {
        isConfident = true;
        return "Capacitor";
    }
    if (uDesc.find("DIODE") != std::string::npos) {
        isConfident = true;
        return "Diode";
    }
    if (uDesc.find("TRANSISTOR") != std::string::npos || uDesc.find("MOSFET") != std::string::npos) {
        isConfident = true;
        return "Transistor";
    }
    if (uDesc.find("INDUCTOR") != std::string::npos) {
        isConfident = true;
        return "Inductor";
    }
    if (uDesc.find("OSCILLATOR") != std::string::npos || uDesc.find("CRYSTAL") != std::string::npos) {
        isConfident = true;
        return "Oscillator";
    }
    if (uDesc.find("CONNECTOR") != std::string::npos || uDesc.find("HEADER") != std::string::npos) {
        isConfident = true;
        return "Connector";
    }
    if (uDesc.find("SWITCH") != std::string::npos || uDesc.find("BUTTON") != std::string::npos) {
        isConfident = true;
        return "Switch/Button";
    }
    if (uDesc.find(" IC ") != std::string::npos || uDesc.find("OP AMP") != std::string::npos || uDesc.find("REGULATOR") != std::string::npos) {
        isConfident = true;
        return "IC";
    }

    // 3. Analisi del package / outline
    if (uOutline.find("SOT") != std::string::npos || uOutline.find("SOIC") != std::string::npos ||
        uOutline.find("QFP") != std::string::npos || uOutline.find("QFN") != std::string::npos ||
        uOutline.find("MSOP") != std::string::npos || uOutline.find("BGA") != std::string::npos) {
        if (uRef.rfind("Q", 0) == 0) return "Transistor";
        if (uRef.rfind("D", 0) == 0) return "Diode";
        return "IC";
    }

    return "Other";
}

std::string AoiVerificationProcessor::askUserForClass(const std::string& ref,
                                                      const std::string& partNumber,
                                                      const std::string& outline,
                                                      const std::string& description,
                                                      double& out_w_mm,
                                                      double& out_h_mm,
                                                      double& out_thick_mm,
                                                      std::string& out_source) const {
    out_w_mm = 0.0;
    out_h_mm = 0.0;
    out_thick_mm = 0.0;
    out_source = "user_input";

    std::cout << "\n================================================================================" << std::endl;
    std::cout << "[AOI MAPPING] ATTENZIONE: COMPONENTE NON PRESENTE IN component_cv_mapping.txt" << std::endl;
    std::cout << "================================================================================" << std::endl;
    std::cout << "DATI IDENTIFICATIVI DEL COMPONENTE (dal file distinta/AOI):" << std::endl;
    std::cout << "  - Reference Designator (Ref): " << ref << std::endl;
    std::cout << "  - Codice Produttore / P/N:     " << partNumber << std::endl;
    std::cout << "  - Outline / Package CAD:       " << outline << std::endl;
    std::cout << "  - Descrizione:                 " << description << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "GUIDA ALLA RICERCA ONLINE DEI DATI MANCANTI:" << std::endl;
    std::cout << "  1. Cerca il codice produttore '" << partNumber << "' o il package '" << outline << "' su:" << std::endl;
    std::cout << "     * Octopart: https://octopart.com/search?q=" << partNumber << std::endl;
    std::cout << "     * DigiKey:  https://www.digikey.com/" << std::endl;
    std::cout << "     * Mouser:   https://www.mouser.com/" << std::endl;
    std::cout << "     * Motore di ricerca: Google/Bing (\"" << partNumber << " datasheet\")" << std::endl;
    std::cout << "  2. Apri il datasheet PDF ufficiale e consulta la sezione 'Package / Mechanical Data'." << std::endl;
    std::cout << "  3. Rileva le dimensioni esterne massime/nominali del corpo in millimetri (mm):" << std::endl;
    std::cout << "     * Larghezza (W in mm) ed Estensione (H/L in mm)" << std::endl;
    std::cout << "     * Spessore / Altezza profilo (Thickness in mm, facoltativo)" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "FORMATO DELLA STRINGA DI TESTO PER component_cv_mapping.txt:" << std::endl;
    std::cout << "  --> Class: <Classe> | Dimensions (mm): <W> x <H> [x <Spessore>] | Source: <fonte>" << std::endl;
    std::cout << "  Esempio: --> Class: IC | Dimensions (mm): 1.6 x 1.6 x 0.55 | Source: onsemi.com" << std::endl;
    std::cout << "  Esempio: --> Class: Resistor | Dimensions (mm): 1.6 x 0.8 x 0.45 | Source: vishay.com" << std::endl;
    std::cout << "================================================================================" << std::endl;

    auto classes = getSupportedClasses();

    std::cout << "Incolla la stringa di testo completa oppure premi INVIO per la procedura guidata: ";
    std::string fullLine;
    if (std::getline(std::cin, fullLine)) {
        std::string trimmedLine = trim(fullLine);
        if (!trimmedLine.empty()) {
            size_t cPos = trimmedLine.find("Class:");
            if (cPos != std::string::npos) {
                std::string sub = trimmedLine.substr(cPos + 6);
                size_t p1 = sub.find('|');
                std::string cls = trim(p1 != std::string::npos ? sub.substr(0, p1) : sub);

                size_t dPos = trimmedLine.find("Dimensions (mm):");
                if (dPos != std::string::npos) {
                    std::string dSub = trimmedLine.substr(dPos + 16);
                    size_t p2 = dSub.find('|');
                    std::string dStr = trim(p2 != std::string::npos ? dSub.substr(0, p2) : dSub);

                    std::vector<std::string> parts;
                    std::stringstream ss(dStr);
                    std::string item;
                    while (std::getline(ss, item, 'x')) {
                        std::string t = trim(item);
                        if (!t.empty()) parts.push_back(t);
                    }
                    try {
                        if (parts.size() >= 1) out_w_mm = std::stod(parts[0]);
                        if (parts.size() >= 2) out_h_mm = std::stod(parts[1]);
                        if (parts.size() >= 3) out_thick_mm = std::stod(parts[2]);
                    } catch (...) {}
                }

                size_t sPos = trimmedLine.find("Source:");
                if (sPos != std::string::npos) {
                    out_source = trim(trimmedLine.substr(sPos + 7));
                }

                if (!cls.empty()) {
                    return cls;
                }
            }
        }
    }

    // Modalità guidata passo-passo
    std::cout << "\nSeleziona la classe del componente:" << std::endl;
    for (size_t i = 0; i < classes.size(); ++i) {
        std::cout << "  " << std::setw(2) << (i + 1) << ") " << classes[i] << std::endl;
    }

    std::string chosenClass = "Other";
    while (true) {
        std::cout << "\nInserisci il numero (1-" << classes.size() << ") o il nome della classe: ";
        std::string input;
        if (!(std::cin >> input)) {
            std::cin.clear();
            bool conf = false;
            chosenClass = inferClassFromKeywords(ref, outline, description, conf);
            getNominalOutlineDimensions(outline, out_w_mm, out_h_mm);
            return chosenClass;
        }

        try {
            int num = std::stoi(input);
            if (num >= 1 && num <= static_cast<int>(classes.size())) {
                chosenClass = classes[num - 1];
                break;
            }
        } catch (...) {}

        std::string inputUpper = toUpper(input);
        for (const auto& c : classes) {
            if (toUpper(c) == inputUpper) {
                chosenClass = c;
                break;
            }
        }
        if (chosenClass != "Other" || inputUpper == "OTHER") break;

        std::cout << "Selezione non valida. Riprova." << std::endl;
    }

    // Richiesta dimensioni
    std::cout << "Inserisci dimensioni in mm (es: '1.6 0.8' o '1.6 0.8 0.45' o '1.6 x 0.8 x 0.45'): ";
    std::string dimLine;
    // Consuma eventuale newline residuo
    std::getline(std::cin, dimLine);
    if (dimLine.empty()) std::getline(std::cin, dimLine);
    dimLine = trim(dimLine);
    if (!dimLine.empty()) {
        for (char& ch : dimLine) {
            if (ch == 'x' || ch == 'X' || ch == ',') ch = ' ';
        }
        std::stringstream ss(dimLine);
        ss >> out_w_mm >> out_h_mm;
        double th = 0.0;
        if (ss >> th) out_thick_mm = th;
    }
    if (out_w_mm <= 0.0 || out_h_mm <= 0.0) {
        getNominalOutlineDimensions(outline, out_w_mm, out_h_mm);
    }

    // Richiesta fonte
    std::cout << "Inserisci fonte o sito del datasheet (es. 'ti.com', premi INVIO per default): ";
    std::string srcLine;
    std::getline(std::cin, srcLine);
    srcLine = trim(srcLine);
    if (!srcLine.empty()) {
        out_source = srcLine;
    } else {
        out_source = "user_datasheet";
    }

    return chosenClass;
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
        double ds_w = 0.0, ds_h = 0.0, ds_thick = 0.0;
        std::string ds_source = "";

        auto itRaw = datasheet_table_.find(s);
        if (itRaw != datasheet_table_.end()) {
            assignedClass = itRaw->second.assigned_class;
            ds_w = itRaw->second.width_mm;
            ds_h = itRaw->second.height_mm;
            ds_thick = itRaw->second.thickness_mm;
            ds_source = itRaw->second.source;
        } else {
            auto itRef = datasheet_table_.find("REF:" + ref);
            if (itRef != datasheet_table_.end()) {
                assignedClass = itRef->second.assigned_class;
                ds_w = itRef->second.width_mm;
                ds_h = itRef->second.height_mm;
                ds_thick = itRef->second.thickness_mm;
                ds_source = itRef->second.source;
            } else {
                auto itCode = datasheet_table_.find("CODE:" + code);
                if (itCode != datasheet_table_.end()) {
                    assignedClass = itCode->second.assigned_class;
                    ds_w = itCode->second.width_mm;
                    ds_h = itCode->second.height_mm;
                    ds_thick = itCode->second.thickness_mm;
                    ds_source = itCode->second.source;
                }
            }
        }

        if (assignedClass.empty()) {
            bool isConfident = false;
            std::string inferred = inferClassFromKeywords(ref, outline, desc, isConfident);

            if (isConfident) {
                assignedClass = inferred;
                getNominalOutlineDimensions(outline, ds_w, ds_h);
                ds_source = "IPC-7351 / Package standard";
                saveMappingEntry(mappingFilePath, s, assignedClass, ds_w, ds_h, ds_thick, ds_source);
                autoAddedCount++;
            } else if (interactive) {
                assignedClass = askUserForClass(ref, code, outline, desc, ds_w, ds_h, ds_thick, ds_source);
                saveMappingEntry(mappingFilePath, s, assignedClass, ds_w, ds_h, ds_thick, ds_source);
                userPromptCount++;
            } else {
                assignedClass = inferred;
                getNominalOutlineDimensions(outline, ds_w, ds_h);
                ds_source = "Auto-fallback";
            }
        }

        comp.assigned_class = assignedClass;
        comp.datasheet_width_mm = ds_w;
        comp.datasheet_height_mm = ds_h;
        comp.datasheet_thickness_mm = ds_thick;
        comp.datasheet_source = ds_source;
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

} // namespace pcb

