#include "class_mapper.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <regex>
#include <limits>

namespace fs = std::filesystem;

namespace pcb {

namespace {

const std::vector<std::string> PARSA_VICTOR_CLASSES = {
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

// Struttura per associare parole chiave alle classi ParsaVictor
struct KeywordRule {
    std::string keyword;
    std::string parsa_class;
    float weight;
    bool whole_word_only;
};

const std::vector<KeywordRule> KEYWORD_RULES = {
    // Resistor
    {"resistor", "Resistor", 0.95f, false},
    {"resistance", "Resistor", 0.95f, false},
    {"resistenza", "Resistor", 0.95f, false},
    {"potentiometer", "Resistor", 0.95f, false},
    {"potenziometro", "Resistor", 0.95f, false},
    {"trimpot", "Resistor", 0.95f, false},
    {"shunt", "Resistor", 0.90f, true},
    {"resistor network", "Resistor", 0.98f, false},
    {"resnet", "Resistor", 0.95f, false},
    {"resistor jumper", "Resistor", 0.95f, false},
    {"ressistor", "Resistor", 0.95f, false},
    {"resistort", "Resistor", 0.95f, false},
    {"rp", "Resistor", 0.85f, true},
    {"res", "Resistor", 0.85f, true},

    // Electrolytic Capacitor (prima di Capacitor generico)
    {"electrolytic capacitor", "Electrolytic Capacitor", 0.99f, false},
    {"electrolytic", "Electrolytic Capacitor", 0.95f, false},
    {"elettrolitico", "Electrolytic Capacitor", 0.95f, false},
    {"elko", "Electrolytic Capacitor", 0.95f, true},
    {"can capacitor", "Electrolytic Capacitor", 0.95f, false},

    // Capacitor
    {"capacitor", "Capacitor", 0.95f, false},
    {"condensatore", "Capacitor", 0.95f, false},
    {"capacitance", "Capacitor", 0.95f, false},
    {"ceramic capacitor", "Capacitor", 0.95f, false},
    {"multilayer ceramic", "Capacitor", 0.95f, false},
    {"mlcc", "Capacitor", 0.95f, true},
    {"tantalum", "Capacitor", 0.95f, false},
    {"cap", "Capacitor", 0.85f, true},

    // IC
    {"integrated circuit", "IC", 0.98f, false},
    {"circuito integrato", "IC", 0.98f, false},
    {"circuito_integrado", "IC", 0.98f, false},
    {"procesador", "IC", 0.95f, false},
    {"processor", "IC", 0.95f, false},
    {"microcontroller", "IC", 0.95f, false},
    {"mcu", "IC", 0.95f, true},
    {"cpu", "IC", 0.95f, true},
    {"soc", "IC", 0.90f, true},
    {"chip", "IC", 0.90f, true},
    {"fpga", "IC", 0.95f, true},
    {"asic", "IC", 0.95f, true},
    {"soic", "IC", 0.95f, false},
    {"sop", "IC", 0.90f, true},
    {"qfp", "IC", 0.95f, false},
    {"tqfp", "IC", 0.95f, false},
    {"bga", "IC", 0.95f, true},
    {"qfn", "IC", 0.95f, true},
    {"dip", "IC", 0.85f, true},
    {"regulator", "IC", 0.85f, false},
    {"opamp", "IC", 0.95f, true},
    {"ic", "IC", 0.90f, true},

    // Inductor
    {"inductor", "Inductor", 0.95f, false},
    {"induttore", "Inductor", 0.95f, false},
    {"inductance", "Inductor", 0.95f, false},
    {"coil", "Inductor", 0.95f, true},
    {"choke", "Inductor", 0.95f, true},
    {"ferrite bead", "Inductor", 0.98f, false},
    {"ferrite", "Inductor", 0.92f, false},
    {"toroid", "Inductor", 0.95f, false},
    {"ind", "Inductor", 0.85f, true},

    // Diode
    {"zener diode", "Diode", 0.98f, false},
    {"zener", "Diode", 0.95f, false},
    {"schottky", "Diode", 0.95f, false},
    {"rectifier", "Diode", 0.95f, false},
    {"tvs", "Diode", 0.95f, true},
    {"transil", "Diode", 0.95f, false},
    {"diode", "Diode", 0.95f, false},
    {"diodo", "Diode", 0.95f, false},

    // LED
    {"light emitting diode", "LED", 0.99f, false},
    {"diodo led", "LED", 0.98f, false},
    {"led", "LED", 0.95f, true},
    {"indicator", "LED", 0.85f, false},

    // Transistor
    {"transistor", "Transistor", 0.95f, false},
    {"mosfet", "Transistor", 0.98f, false},
    {"bjt", "Transistor", 0.95f, true},
    {"igbt", "Transistor", 0.95f, true},
    {"triac", "Transistor", 0.95f, true},
    {"fet", "Transistor", 0.90f, true},
    {"sot23", "Transistor", 0.90f, false},

    // Oscillator
    {"oscillator", "Oscillator", 0.95f, false},
    {"oscilatore", "Oscillator", 0.95f, false},
    {"oscilador", "Oscillator", 0.95f, false},
    {"crystal", "Oscillator", 0.95f, false},
    {"xtal", "Oscillator", 0.95f, true},
    {"resonator", "Oscillator", 0.95f, false},
    {"quarzo", "Oscillator", 0.95f, false},
    {"clock", "Oscillator", 0.88f, true},

    // Connector
    {"connector", "Connector", 0.95f, false},
    {"connettore", "Connector", 0.95f, false},
    {"header", "Connector", 0.92f, false},
    {"pin header", "Connector", 0.95f, false},
    {"terminal block", "Connector", 0.95f, false},
    {"terminal", "Connector", 0.88f, false},
    {"usb", "Connector", 0.95f, true},
    {"rj45", "Connector", 0.95f, true},
    {"hdmi", "Connector", 0.95f, true},
    {"bnc", "Connector", 0.95f, true},
    {"jack", "Connector", 0.90f, true},
    {"plug", "Connector", 0.90f, true},
    {"socket", "Connector", 0.90f, true},
    {"port", "Connector", 0.85f, true},
    {"slot", "Connector", 0.85f, true},
    {"ket noi", "Connector", 0.95f, false},

    // Switch/Button
    {"switch_button", "Switch/Button", 0.98f, false},
    {"switch", "Switch/Button", 0.95f, false},
    {"button", "Switch/Button", 0.95f, false},
    {"pulsante", "Switch/Button", 0.95f, false},
    {"interruttore", "Switch/Button", 0.95f, false},
    {"tactile switch", "Switch/Button", 0.98f, false},
    {"tact switch", "Switch/Button", 0.98f, false},
    {"dip switch", "Switch/Button", 0.98f, false},
    {"pushbutton", "Switch/Button", 0.95f, false},
    {"buton", "Switch/Button", 0.95f, false},
    {"btn", "Switch/Button", 0.88f, true},
    {"pb", "Switch/Button", 0.80f, true},
    {"sw", "Switch/Button", 0.80f, true},

    // Marker
    {"test point", "Marker", 0.98f, false},
    {"testpoint", "Marker", 0.98f, false},
    {"fiducial", "Marker", 0.98f, false},
    {"marker", "Marker", 0.95f, false},
    {"pads", "Marker", 0.95f, true},
    {"pad", "Marker", 0.90f, true},
    {"pins", "Marker", 0.90f, true},
    {"pin", "Marker", 0.85f, true},
    {"via", "Marker", 0.90f, true},

    // Other
    {"fuse", "Other", 0.95f, false},
    {"fusibile", "Other", 0.95f, false},
    {"relay", "Other", 0.95f, false},
    {"rele", "Other", 0.95f, false},
    {"transformer", "Other", 0.95f, false},
    {"trasformatore", "Other", 0.95f, false},
    {"battery", "Other", 0.95f, false},
    {"batteria", "Other", 0.95f, false},
    {"buzzer", "Other", 0.95f, false},
    {"transducer", "Other", 0.95f, false},
    {"heatsink", "Other", 0.95f, false},
    {"dissipatore", "Other", 0.95f, false},
    {"filter", "Other", 0.90f, false},
    {"emi filter", "Other", 0.95f, false},
    {"filtro", "Other", 0.90f, false},
    {"jumper", "Other", 0.95f, false},
    {"display", "Other", 0.95f, false},
    {"screen", "Other", 0.90f, false},
    {"speaker", "Other", 0.90f, false},
    {"unknown", "Other", 0.80f, false},
    {"unlabeled", "Other", 0.80f, false},
    {"other", "Other", 0.95f, false},
    {"altro", "Other", 0.95f, false}
};

} // namespace

ClassMapper::ClassMapper(const std::string& mapping_file_path)
    : mapping_file_path_(mapping_file_path) {
    // Tenta caricamento dal percorso specificato o dai percorsi standard alternativi
    if (!loadMappingFile(mapping_file_path_)) {
        std::vector<std::string> fallbacks = {
            "data/parameters/class_mapping_weights.txt",
            "../data/parameters/class_mapping_weights.txt",
            "../../data/parameters/class_mapping_weights.txt"
        };
        for (const auto& fb : fallbacks) {
            if (fs::exists(fb) && loadMappingFile(fb)) {
                mapping_file_path_ = fb;
                break;
            }
        }
    }
}

const std::vector<std::string>& ClassMapper::getParsaVictorClasses() {
    return PARSA_VICTOR_CLASSES;
}

bool ClassMapper::isParsaVictorClass(const std::string& label) {
    for (const auto& c : PARSA_VICTOR_CLASSES) {
        if (c == label) return true;
    }
    return false;
}

std::string ClassMapper::normalizeString(const std::string& str) {
    std::string s;
    s.reserve(str.size());
    for (char ch : str) {
        unsigned char uc = static_cast<unsigned char>(ch);
        if (uc == '_' || uc == '-' || uc == '/' || uc == '.' || uc == ',') {
            s.push_back(' ');
        } else if (!std::iscntrl(uc)) {
            s.push_back(static_cast<char>(std::tolower(uc)));
        }
    }
    // Rimozione doppi spazi e trim
    std::string clean;
    clean.reserve(s.size());
    bool in_space = true;
    for (char c : s) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (!in_space) {
                clean.push_back(' ');
                in_space = true;
            }
        } else {
            clean.push_back(c);
            in_space = false;
        }
    }
    if (!clean.empty() && clean.back() == ' ') {
        clean.pop_back();
    }
    return clean;
}

bool ClassMapper::loadMappingFile(const std::string& path) {
    if (!fs::exists(path)) return false;

    std::ifstream file(path);
    if (!file.is_open()) return false;

    std::string line;
    while (std::getline(file, line)) {
        // Rimuove spazi e commenti
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string left = line.substr(0, eq_pos);
        std::string right = line.substr(eq_pos + 1);

        // Trim right
        size_t r_start = right.find_first_not_of(" \t\r\n");
        size_t r_end = right.find_last_not_of(" \t\r\n");
        if (r_start == std::string::npos) continue;
        std::string parsa_target = right.substr(r_start, r_end - r_start + 1);

        // Controllo validità classe ParsaVictor (se valida mantieni, altrimenti standardizza)
        bool valid_target = false;
        for (const auto& pvc : PARSA_VICTOR_CLASSES) {
            if (normalizeString(pvc) == normalizeString(parsa_target)) {
                parsa_target = pvc;
                valid_target = true;
                break;
            }
        }
        if (!valid_target && parsa_target != "?") {
            parsa_target = "Other";
        }

        // Parsing modello:classe
        size_t colon_pos = left.find(':');
        std::string model_name = "*";
        std::string raw_cls = left;

        if (colon_pos != std::string::npos) {
            model_name = normalizeString(left.substr(0, colon_pos));
            raw_cls = left.substr(colon_pos + 1);
        }

        std::string clean_cls = normalizeString(raw_cls);
        if (clean_cls.empty()) continue;

        if (model_name == "*" || model_name.empty()) {
            global_class_map_[clean_cls] = parsa_target;
        } else {
            model_class_map_[{model_name, clean_cls}] = parsa_target;
        }
    }

    return true;
}

bool ClassMapper::saveMappingFile(const std::string& path) const {
    std::string target_path = path.empty() ? mapping_file_path_ : path;
    if (target_path.empty()) return false;

    // Crea cartella genitrice se necessario
    fs::path p(target_path);
    if (p.has_parent_path()) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
    }

    std::ofstream file(target_path);
    if (!file.is_open()) return false;

    file << "# =========================================================================\n";
    file << "# FILE DI TRACCIAMENTO ASSOCIAZIONI CLASSI COMPONENTI PCB\n";
    file << "# =========================================================================\n\n";

    file << "# --- Regole Globali ---\n";
    for (const auto& [cls, target] : global_class_map_) {
        file << cls << " = " << target << "\n";
    }

    file << "\n# --- Regole Specifiche per Modello ---\n";
    for (const auto& [key, target] : model_class_map_) {
        file << key.first << ":" << key.second << " = " << target << "\n";
    }

    return true;
}

void ClassMapper::appendMappingToFile(const std::string& model_name,
                                      const std::string& orig_class,
                                      const std::string& parsa_class) {
    if (mapping_file_path_.empty()) return;

    fs::path p(mapping_file_path_);
    if (p.has_parent_path()) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
    }

    std::ofstream file(mapping_file_path_, std::ios::app);
    if (file.is_open()) {
        if (!model_name.empty() && model_name != "*") {
            file << model_name << ":" << orig_class << " = " << parsa_class << "\n";
        } else {
            file << orig_class << " = " << parsa_class << "\n";
        }
    }
}

void ClassMapper::addMapping(const std::string& model_name,
                             const std::string& orig_class,
                             const std::string& parsa_class) {
    std::string m_norm = normalizeString(model_name);
    std::string c_norm = normalizeString(orig_class);

    if (m_norm.empty() || m_norm == "*") {
        global_class_map_[c_norm] = parsa_class;
    } else {
        model_class_map_[{m_norm, c_norm}] = parsa_class;
    }

    appendMappingToFile(model_name, orig_class, parsa_class);
}

ClassMatchResult ClassMapper::matchClassByKeywords(const std::string& raw_class) const {
    ClassMatchResult res;
    std::string clean = normalizeString(raw_class);
    if (clean.empty()) {
        res.parsa_class = "?";
        res.confidence = 0.0f;
        res.is_certain = false;
        res.match_reason = "EMPTY_CLASS";
        return res;
    }

    // 1. Corrispondenza esatta con una delle 13 classi ParsaVictor
    for (const auto& pvc : PARSA_VICTOR_CLASSES) {
        if (clean == normalizeString(pvc)) {
            res.parsa_class = pvc;
            res.confidence = 1.0f;
            res.is_certain = true;
            res.match_reason = "EXACT_PARSA_MATCH";
            return res;
        }
    }

    // 2. Controllo prefissi standard schemi circuitali (es. R12, C4, U1, Q2, D1, L3)
    std::smatch sm;
    if (std::regex_match(clean, sm, std::regex(R"(^c\s*\d+.*)"))) {
        res.parsa_class = "Capacitor";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_C";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^r\s*\d+.*)"))) {
        res.parsa_class = "Resistor";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_R";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^u\s*\d+.*)")) || clean == "u") {
        res.parsa_class = "IC";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_U";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^l\s*\d+.*)"))) {
        res.parsa_class = "Inductor";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_L";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^d\s*\d+.*)"))) {
        res.parsa_class = "Diode";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_D";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^q\s*\d+.*)"))) {
        res.parsa_class = "Transistor";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_Q";
        return res;
    }
    if (std::regex_match(clean, sm, std::regex(R"(^j\s*\d+.*)")) || std::regex_match(clean, sm, std::regex(R"(^cn\s*\d+.*)"))) {
        res.parsa_class = "Connector";
        res.confidence = 0.88f;
        res.is_certain = true;
        res.match_reason = "REF_PREFIX_J_CN";
        return res;
    }

    // 3. Verifica per parole chiave e sinonimi
    float best_score = 0.0f;
    std::string best_class = "?";
    std::string best_reason = "";

    // Suddivide il testo in token
    std::vector<std::string> tokens;
    std::stringstream ss(clean);
    std::string tok;
    while (ss >> tok) tokens.push_back(tok);

    for (const auto& rule : KEYWORD_RULES) {
        if (rule.whole_word_only) {
            // Cerca parola intera tra i token
            for (const auto& t : tokens) {
                if (t == rule.keyword) {
                    if (rule.weight > best_score) {
                        best_score = rule.weight;
                        best_class = rule.parsa_class;
                        best_reason = "WHOLE_TOKEN_MATCH [" + rule.keyword + "]";
                    }
                }
            }
        } else {
            // Cerca sottostringa o match parziale
            size_t pos = clean.find(rule.keyword);
            if (pos != std::string::npos) {
                float score = rule.weight;
                // Premia se corrisponde all'intera stringa
                if (clean == rule.keyword) score += 0.04f;
                if (score > best_score) {
                    best_score = score;
                    best_class = rule.parsa_class;
                    best_reason = "SUBSTRING_MATCH [" + rule.keyword + "]";
                }
            }
        }
    }

    res.parsa_class = (best_score > 0.0f) ? best_class : "?";
    res.confidence = best_score;
    res.is_certain = (best_score >= 0.75f);
    res.match_reason = best_reason.empty() ? "NO_CONFIDENT_MATCH" : best_reason;
    return res;
}

std::string ClassMapper::promptUserForClass(const std::string& model_name,
                                            const std::string& orig_class,
                                            const std::string& suggested_class) const {
    std::cout << "\n========================================================================" << std::endl;
    std::cout << " [ClassMapper] ASSOCIAZIONE CLASSE INCERTA RILEVATA                     " << std::endl;
    std::cout << "========================================================================" << std::endl;
    std::cout << "  Modello:           " << model_name << std::endl;
    std::cout << "  Classe originale:  '" << orig_class << "'" << std::endl;
    if (!suggested_class.empty() && suggested_class != "?") {
        std::cout << "  Suggerimento:      '" << suggested_class << "' (ipotesi automatica)" << std::endl;
    }
    std::cout << "------------------------------------------------------------------------" << std::endl;
    std::cout << "Seleziona a quale classe di ParsaVictor associare questa classe:" << std::endl;

    for (size_t i = 0; i < PARSA_VICTOR_CLASSES.size(); ++i) {
        std::cout << " " << std::setw(2) << i + 1 << ") " << PARSA_VICTOR_CLASSES[i];
        if (!suggested_class.empty() && suggested_class == PARSA_VICTOR_CLASSES[i]) {
            std::cout << "  <-- Consigliato";
        }
        std::cout << std::endl;
    }
    std::cout << " 14) Sconosciuto (?) / Ignora" << std::endl;

    // Determina scelta predefinita
    int default_choice = 14;
    for (size_t i = 0; i < PARSA_VICTOR_CLASSES.size(); ++i) {
        if (!suggested_class.empty() && suggested_class == PARSA_VICTOR_CLASSES[i]) {
            default_choice = static_cast<int>(i + 1);
            break;
        }
    }

    std::cout << "Inserisci numero (1-14, default " << default_choice << "): ";
    std::string input;
    if (std::cin >> input) {
        try {
            int choice = std::stoi(input);
            if (choice >= 1 && choice <= static_cast<int>(PARSA_VICTOR_CLASSES.size())) {
                return PARSA_VICTOR_CLASSES[choice - 1];
            }
            if (choice == 14) {
                return "?";
            }
        } catch (...) {
            // Input non numerico, controlla se corrisponde a una classe digitata per esteso
            for (const auto& pvc : PARSA_VICTOR_CLASSES) {
                if (normalizeString(pvc) == normalizeString(input)) {
                    return pvc;
                }
            }
        }
    }

    if (default_choice >= 1 && default_choice <= static_cast<int>(PARSA_VICTOR_CLASSES.size())) {
        return PARSA_VICTOR_CLASSES[default_choice - 1];
    }
    return "?";
}

std::string ClassMapper::getOrMapClass(const std::string& model_name,
                                      const std::string& orig_class,
                                      bool allow_interactive) {
    std::string m_norm = normalizeString(model_name);
    std::string c_norm = normalizeString(orig_class);

    // 1. Controllo mappa per modello specifico
    auto it_mod = model_class_map_.find({m_norm, c_norm});
    if (it_mod != model_class_map_.end()) {
        return it_mod->second;
    }

    // 2. Controllo mappa globale
    auto it_glob = global_class_map_.find(c_norm);
    if (it_glob != global_class_map_.end()) {
        return it_glob->second;
    }

    // 3. Algoritmo per parole chiave e somiglianza
    auto match_res = matchClassByKeywords(orig_class);
    if (match_res.is_certain) {
        std::cout << "[ClassMapper] Mappatura automatica: '" << orig_class << "' [" << model_name
                  << "] -> '" << match_res.parsa_class << "' (confidenza: "
                  << static_cast<int>(match_res.confidence * 100.0f) << "%, "
                  << match_res.match_reason << ")" << std::endl;

        addMapping(model_name, orig_class, match_res.parsa_class);
        return match_res.parsa_class;
    }

    // 4. Incertezza: chiedi all'utente se consentito
    if (allow_interactive) {
        std::string user_choice = promptUserForClass(model_name, orig_class, match_res.parsa_class);
        std::cout << "[ClassMapper] Associazione registrata dall'utente: '" << orig_class << "' ["
                  << model_name << "] -> '" << user_choice << "'" << std::endl;

        addMapping(model_name, orig_class, user_choice);
        return user_choice;
    }

    // Fallback automatico se non interattivo
    std::string fallback = (!match_res.parsa_class.empty() && match_res.parsa_class != "?")
                           ? match_res.parsa_class : "Other";
    addMapping(model_name, orig_class, fallback);
    return fallback;
}

} // namespace pcb

