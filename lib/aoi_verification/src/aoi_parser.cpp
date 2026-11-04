#include "aoi_internal.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace pcb {

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

} // namespace pcb

