#include "calibration.hpp"
#include "undistort.hpp"
#include "component_detector.hpp"
#include "cv_component_detector.hpp"
#include <string>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <vector>
#include <limits>
#include <set>

namespace fs = std::filesystem;

// Funzione ausiliaria per chiedere all'utente quale lato elaborare (TOP o BOTTOM)
std::string askForBoardSide(const std::string& boardName) {
    std::string selectedSide;
    while (true) {
        std::cout << "\nPer la scheda " << boardName
                  << ", quale lato vuoi elaborare? (1 / TOP oppure 2 / BOTTOM): ";
        if (!(std::cin >> selectedSide)) {
            std::cerr << "ERRORE: impossibile leggere il lato della scheda." << std::endl;
            return {};
        }

        std::transform(selectedSide.begin(), selectedSide.end(), selectedSide.begin(),
                       [](unsigned char character) {
                           return static_cast<char>(std::toupper(character));
                       });

        if (selectedSide == "1" || selectedSide == "TOP") {
            return "TOP";
        }
        if (selectedSide == "2" || selectedSide == "BOTTOM") {
            return "BOTTOM";
        }
        std::cout << "Input non valido. Inserisci 1/TOP oppure 2/BOTTOM."
                  << std::endl;
    }
}

// Funzione per calibrare la telecamera Robot (lasciata invariata)
bool runRobotCameraCalibration() {
    std::cout << "=== AVVIO CALIBRAZIONE TELECAMERA ===" << std::endl;

    std::string output_params_dir = "../data/parameters";
    std::error_code ec;
    fs::create_directories(output_params_dir, ec);
    if (ec) {
        std::cerr << "ERRORE: Impossibile creare la cartella di output " 
                  << output_params_dir << ": " << ec.message() << std::endl;
        return false;
    }

    std::string robot_calib = "../data/inputs/calib_images/calib_images_robot";
    std::string robot_params = output_params_dir + "/camera_params_robot.yml";

    if (!fs::exists(robot_calib)) {
        std::cerr << "ERRORE: Cartella di calibrazione Robot non trovata: " << robot_calib << std::endl;
        return false;
    }

    std::cout << "\n--- Elaborazione in corso ---" << std::endl;
    bool success = calibrateCameraSet(
        robot_calib, 
        ".bmp", 
        12, 9, 
        0.00571f, 
        0.00381f,
        robot_params, 
        0.5, 
        cv::aruco::DICT_6X6_250,
        false
    );

    if (!success) {
        std::cerr << "ERRORE: Calibrazione Robot fallita." << std::endl;
        return false;
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << "Calibrazione completata con successo!" << std::endl;
    std::cout << "Parametri salvati in: " << robot_params << std::endl;
    std::cout << "========================================================" << std::endl;

    return true;
}

// Funzione per raddrizzare le immagini PCB con opzione 'Fai tutto' o loop interattivo (scheda per scheda, lato per lato)
bool runRobotImagesUndistortion() {
    std::cout << "\n=== AVVIO RADDRIZZAMENTO IMMAGINI ROBOT ===" << std::endl;

    std::string robot_params = "../data/parameters/camera_params_robot.yml";
    std::string pcb_base_dir = "../data/inputs/pcb_images";
    std::string output_base_dir = "../data/outputs/pcb_images_undistorted";

    if (!fs::exists(robot_params)) {
        std::cerr << "ERRORE: Parametri di calibrazione non trovati in: " << robot_params
                  << ". Esegui prima la calibrazione." << std::endl;
        return false;
    }

    if (!fs::exists(pcb_base_dir)) {
        std::cout << "Cartella immagini PCB non trovata: " << pcb_base_dir
                  << " (nessuna immagine da raddrizzare, salto la fase)." << std::endl;
        return true;
    }

    // Raccogliamo tutte le schede disponibili (cartelle)
    std::vector<fs::path> board_paths;
    for (const auto& entry : fs::directory_iterator(pcb_base_dir)) {
        if (entry.is_directory()) {
            board_paths.push_back(entry.path());
        }
    }

    if (board_paths.empty()) {
        std::cout << "Nessuna sottocartella scheda trovata in " << pcb_base_dir << "." << std::endl;
        return true;
    }

    // Lambda per elaborare un lato specifico di una scheda
    auto processSide = [&](const fs::path& board_path, const std::string& board_name, const std::string& side) {
        fs::path side_dir = board_path / side;
        fs::path output_side_dir = fs::path(output_base_dir) / board_name / side;

        if (!fs::exists(side_dir)) {
            std::cout << "Lato " << side << " non trovato per " << board_name << ". Salto." << std::endl;
            return;
        }

        fs::create_directories(output_side_dir);

        if (fs::is_empty(side_dir)) {
            std::cout << "Nessuna immagine da raddrizzare per "
                      << board_name << " (" << side << "). Salto." << std::endl;
        } else {
            undistortImages(side_dir.string(), ".bmp", robot_params, output_side_dir.string(), board_name, side);
        }
    };

    // Scelta iniziale della modalità: "Fai tutto" vs "Una scheda alla volta e un lato alla volta"
    bool doAll = false;
    while (true) {
        std::cout << "\nModalità di elaborazione raddrizzamento:" << std::endl;
        std::cout << "1) Fai tutto (elabora automaticamente tutte le schede e tutti i lati)" << std::endl;
        std::cout << "2) Una scheda alla volta e un lato alla volta" << std::endl;
        std::cout << "Seleziona un'opzione (1 / 2): ";

        std::string modeChoice;
        if (!(std::cin >> modeChoice)) {
            std::cerr << "ERRORE: Impossibile leggere la modalità di elaborazione." << std::endl;
            return false;
        }

        std::transform(modeChoice.begin(), modeChoice.end(), modeChoice.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

        if (modeChoice == "1" || modeChoice == "TUTTO" || modeChoice == "ALL" || modeChoice == "FAI TUTTO") {
            doAll = true;
            break;
        } else if (modeChoice == "2" || modeChoice == "MANUALE" || modeChoice == "M") {
            doAll = false;
            break;
        } else {
            std::cout << "Scelta non valida. Inserisci 1 per 'Fai tutto' oppure 2 per 'Una scheda alla volta e un lato alla volta'." << std::endl;
        }
    }

    // SE L'UTENTE HA SCELTO "FAI TUTTO": elabora tutte le schede e tutti i lati senza ulteriori domande
    if (doAll) {
        std::cout << "\n=== MODALITÀ FAI TUTTO: Elaborazione automatica di tutte le schede e lati ===" << std::endl;
        for (const auto& board_path : board_paths) {
            std::string board_name = board_path.filename().string();
            std::cout << "\n--- Elaborazione scheda: " << board_name << " ---" << std::endl;

            bool hasTop = fs::exists(board_path / "TOP");
            bool hasBottom = fs::exists(board_path / "BOTTOM");

            if (!hasTop && !hasBottom) {
                std::cout << "Nessuna cartella TOP o BOTTOM trovata per la scheda " << board_name << ". Salto." << std::endl;
                continue;
            }

            if (hasTop) {
                processSide(board_path, board_name, "TOP");
            }
            if (hasBottom) {
                processSide(board_path, board_name, "BOTTOM");
            }

            std::cout << "Scheda " << board_name << " completata." << std::endl;
        }

        std::cout << "\n========================================================" << std::endl;
        std::cout << "Raddrizzamento di tutte le schede completato!" << std::endl;
        std::cout << "========================================================" << std::endl;
        return true;
    }

    // SE C'È UNA SOLA SCHEDA: non serve chiedere quale scheda scegliere
    if (board_paths.size() == 1) {
        fs::path selected_board_path = board_paths[0];
        std::string board_name = selected_board_path.filename().string();

        std::cout << "\nTrovata un'unica scheda disponibile: " << board_name << "." << std::endl;

        bool hasTop = fs::exists(selected_board_path / "TOP");
        bool hasBottom = fs::exists(selected_board_path / "BOTTOM");

        std::string firstSide;
        if (hasTop && hasBottom) {
            firstSide = askForBoardSide(board_name);
            if (firstSide.empty()) return false;
        } else if (hasTop) {
            firstSide = "TOP";
            std::cout << "Trovato solo il lato TOP per " << board_name << ". Procedo con l'analisi." << std::endl;
        } else if (hasBottom) {
            firstSide = "BOTTOM";
            std::cout << "Trovato solo il lato BOTTOM per " << board_name << ". Procedo con l'analisi." << std::endl;
        } else {
            std::cout << "Nessuna cartella TOP o BOTTOM trovata per la scheda " << board_name << "." << std::endl;
            return true;
        }

        processSide(selected_board_path, board_name, firstSide);

        if (hasTop && hasBottom) {
            std::string otherSide = (firstSide == "TOP") ? "BOTTOM" : "TOP";
            std::cout << "\nVuoi analizzare anche l'altro lato (" << otherSide << ") per la scheda " << board_name << "? (S/N): ";
            std::string answer;
            if (std::cin >> answer) {
                std::transform(answer.begin(), answer.end(), answer.begin(), [](unsigned char c) {
                    return static_cast<char>(std::toupper(c));
                });
                if (answer == "S" || answer == "SI" || answer == "Y" || answer == "YES") {
                    processSide(selected_board_path, board_name, otherSide);
                }
            }
        }

        std::cout << "Scheda " << board_name << " completata." << std::endl;
        return true;
    }

    // SE CI SONO PIÙ SCHEDE: usa il loop interattivo con rimozione delle schede elaborate
    std::set<std::string> processed_boards;

    while (true) {
        std::vector<fs::path> available_boards;
        for (const auto& path : board_paths) {
            if (processed_boards.find(path.filename().string()) == processed_boards.end()) {
                available_boards.push_back(path);
            }
        }

        if (available_boards.empty()) {
            std::cout << "\nTutte le schede disponibili sono state elaborate. Uscita automatica." << std::endl;
            break;
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "           SCHEDE DISPONIBILI           " << std::endl;
        std::cout << "========================================" << std::endl;
        for (size_t i = 0; i < available_boards.size(); ++i) {
            std::cout << i + 1 << ") " << available_boards[i].filename().string() << std::endl;
        }
        std::cout << "0) Termina / Esci" << std::endl;
        std::cout << "Seleziona il numero della scheda da analizzare: ";

        int choice = -1;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Input non valido. Riprova." << std::endl;
            continue;
        }

        if (choice == 0) {
            std::cout << "Uscita dal menu di raddrizzamento." << std::endl;
            break;
        }

        if (choice < 1 || choice > static_cast<int>(available_boards.size())) {
            std::cout << "Scelta non valida. Inserisci un numero tra 0 e " << available_boards.size() << "." << std::endl;
            continue;
        }

        fs::path selected_board_path = available_boards[choice - 1];
        std::string board_name = selected_board_path.filename().string();

        std::cout << "\n--- Elaborazione scheda: " << board_name << " ---" << std::endl;

        bool hasTop = fs::exists(selected_board_path / "TOP");
        bool hasBottom = fs::exists(selected_board_path / "BOTTOM");

        std::string firstSide;

        if (hasTop && hasBottom) {
            firstSide = askForBoardSide(board_name);
            if (firstSide.empty()) return false;
        } else if (hasTop) {
            firstSide = "TOP";
            std::cout << "Trovato solo il lato TOP per " << board_name << ". Procedo con l'analisi." << std::endl;
        } else if (hasBottom) {
            firstSide = "BOTTOM";
            std::cout << "Trovato solo il lato BOTTOM per " << board_name << ". Procedo con l'analisi." << std::endl;
        } else {
            std::cout << "Nessuna cartella TOP o BOTTOM trovata per la scheda " << board_name << "." << std::endl;
            processed_boards.insert(board_name);
            continue;
        }

        processSide(selected_board_path, board_name, firstSide);

        if (hasTop && hasBottom) {
            std::string otherSide = (firstSide == "TOP") ? "BOTTOM" : "TOP";
            std::cout << "\nVuoi analizzare anche l'altro lato (" << otherSide << ") per la scheda " << board_name << "? (S/N): ";
            std::string answer;
            if (std::cin >> answer) {
                std::transform(answer.begin(), answer.end(), answer.begin(), [](unsigned char c) {
                    return static_cast<char>(std::toupper(c));
                });
                if (answer == "S" || answer == "SI" || answer == "Y" || answer == "YES") {
                    processSide(selected_board_path, board_name, otherSide);
                }
            }
        }

        processed_boards.insert(board_name);
        std::cout << "Scheda " << board_name << " completata e rimossa dalla lista delle schede da elaborare." << std::endl;
    }

    return true;
}

// Funzione per eseguire il rilevamento dei componenti PCB (modello ParsaVictor a 2 stadi: any + fine)
bool runPcbComponentDetection() {
    std::cout << "\n=== AVVIO RICONOSCIMENTO COMPONENTI PCB ===" << std::endl;

    std::string undistorted_base_dir = "../data/outputs/pcb_images_undistorted";
    std::string raw_base_dir = "../data/inputs/pcb_images";
    std::string detection_output_base_dir = "../data/outputs/pcb_component_detection";

    std::string input_base_dir = undistorted_base_dir;
    if (!fs::exists(input_base_dir) || fs::is_empty(input_base_dir)) {
        std::cout << "Cartella immagini raddrizzate (" << undistorted_base_dir 
                  << ") non trovata o vuota. Utilizzo immagini originali (" << raw_base_dir << ")." << std::endl;
        input_base_dir = raw_base_dir;
    }

    if (!fs::exists(input_base_dir)) {
        std::cerr << "ERRORE: Cartella immagini PCB non trovata: " << input_base_dir << std::endl;
        return false;
    }

    // Pesi predefiniti del modello ParsaVictor (supporta sia .pt che .onnx)
    std::string any_weights = "../pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/weights/any_best.pt";
    std::string fine_weights = "../pcb_component_detection/ParsaVictor/pcb-component-detection-yolo26/weights/fine_best.pt";

    pcb::DetectionConfig config;
    config.conf_threshold = 0.20f;
    config.nms_threshold = 0.50f;
    config.match_iou = 0.30f;
    config.imgsz = 1024;
    config.tile_size = 0; // 0 = scala intera immagine, es. 1024 o 640 per tiling su immagini ad altissima risoluzione
    config.overlap = 0.25f;
    config.label_display_mode = "auto";
    config.min_label_px = 24;

    std::cout << "\nInizializzazione del rilevatore componenti PCB..." << std::endl;
    std::cout << "- Pesi 'any' (localizzazione): " << any_weights << std::endl;
    std::cout << "- Pesi 'fine' (classificazione 13 classi): " << fine_weights << std::endl;

    pcb::ComponentDetector detector;
    if (!detector.loadAnyModel(any_weights, config.imgsz)) {
        std::cerr << "ERRORE: Impossibile caricare il modello ANY." << std::endl;
        return false;
    }

    if (!detector.loadFineModel(fine_weights, pcb::ComponentDetector::getDefaultClasses(), config.imgsz)) {
        std::cerr << "ERRORE: Impossibile caricare il modello FINE." << std::endl;
        return false;
    }

    std::vector<fs::path> board_paths;
    for (const auto& entry : fs::directory_iterator(input_base_dir)) {
        if (entry.is_directory()) {
            board_paths.push_back(entry.path());
        }
    }

    if (board_paths.empty()) {
        std::cout << "Nessuna sottocartella scheda trovata in " << input_base_dir << "." << std::endl;
        return true;
    }

    auto processSideDetection = [&](const fs::path& side_dir, const std::string& board_name, const std::string& side) {
        fs::path out_side_dir = fs::path(detection_output_base_dir) / board_name / side;
        fs::create_directories(out_side_dir);

        std::vector<std::string> valid_exts = {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"};
        std::vector<fs::path> image_files;
        for (const auto& entry : fs::directory_iterator(side_dir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                if (std::find(valid_exts.begin(), valid_exts.end(), ext) != valid_exts.end()) {
                    image_files.push_back(entry.path());
                }
            }
        }

        std::sort(image_files.begin(), image_files.end());
        if (image_files.empty()) {
            std::cout << "Nessuna immagine trovata in " << side_dir << "." << std::endl;
            return;
        }

        std::cout << "\nElaborazione di " << image_files.size() << " immagini per " << board_name << " [" << side << "]..." << std::endl;

        for (const auto& img_path : image_files) {
            std::cout << "\n-> Esecuzione rilevamento su: " << img_path.filename().string() << " ..." << std::endl;
            cv::Mat img = cv::imread(img_path.string());
            if (img.empty()) {
                std::cerr << "ERRORE: Impossibile leggere l'immagine " << img_path << std::endl;
                continue;
            }

            auto result = detector.detect(img, config, "both");
            std::string stem = img_path.stem().string();

            detector.saveResults(out_side_dir.string(), stem, img, result, "both");

            std::cout << "   Trovati " << result.detections.size() << " componenti." << std::endl;
            std::cout << "   Dettaglio classi:" << std::endl;
            for (const auto& [label, count] : result.counts) {
                std::cout << "     - " << label << ": " << count << std::endl;
            }
        }

        std::cout << "Risultati salvati in: " << out_side_dir << std::endl;
    };

    for (const auto& bpath : board_paths) {
        std::string bname = bpath.filename().string();
        std::cout << "\n--- Elaborazione componenti per scheda: " << bname << " ---" << std::endl;

        fs::path top_dir = bpath / "TOP";
        fs::path bot_dir = bpath / "BOTTOM";

        if (fs::exists(top_dir) && !fs::is_empty(top_dir)) {
            processSideDetection(top_dir, bname, "TOP");
        }
        if (fs::exists(bot_dir) && !fs::is_empty(bot_dir)) {
            processSideDetection(bot_dir, bname, "BOTTOM");
        }
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << "Rilevamento componenti completato con successo!" << std::endl;
    std::cout << "Immagini, CSV e riepiloghi JSON salvati in: " << detection_output_base_dir << std::endl;
    std::cout << "========================================================" << std::endl;

    return true;
}


// Funzione per eseguire il rilevamento di tutti i componenti PCB mediante pura Computer Vision classica
// (senza modelli deep learning, con filtro colore verdognolo PCB, filtri per i bordi, rettangoli ruotati e cerchi aderenti)
bool runPcbCvComponentDetection(bool interactive_windows = true) {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "=== AVVIO RILEVAMENTO COMPONENTI PCB (COMPUTER VISION) ===" << std::endl;
    std::cout << "========================================================" << std::endl;

    std::string undistorted_base_dir = "../data/outputs/pcb_images_undistorted";
    std::string raw_base_dir = "../data/inputs/pcb_images";
    std::string detection_output_base_dir = "../data/outputs/pcb_cv_component_detection";

    std::string input_base_dir = undistorted_base_dir;
    if (!fs::exists(input_base_dir) || fs::is_empty(input_base_dir)) {
        std::cout << "Cartella immagini raddrizzate (" << undistorted_base_dir 
                  << ") non trovata o vuota. Utilizzo immagini originali (" << raw_base_dir << ")." << std::endl;
        input_base_dir = raw_base_dir;
    }

    if (!fs::exists(input_base_dir)) {
        std::cerr << "ERRORE: Cartella immagini PCB non trovata: " << input_base_dir << std::endl;
        return false;
    }

    // Configurazione filtri Computer Vision classica
    pcb::CvDetectionConfig config;
    config.use_green_filter = true;          // Filtro colore verdognolo tipico del PCB (solder mask)
    config.h_min = 20;
    config.h_max = 90;
    config.s_min = 16;
    config.v_min = 20;
    config.use_bilateral = true;             // Riduzione rumore con conservazione bordi
    config.use_clahe = true;                 // Contrasto locale adattivo
    config.use_canny = true;                 // Canny edge detector
    config.auto_canny = true;                // Soglie Canny dinamiche dalla mediana
    config.use_morph_hats = true;            // Top-Hat (parti lucide/pin) e Black-Hat (chip neri)
    config.open_kernel_size = 3;             // Rimuove piste sottili (1-2 px)
    config.close_kernel_size = 3;            // Connette pin/saldature al corpo del componente
    config.min_area = 12.0f;                 // Cattura anche componenti SMD piccolissimi (0402, 0603)
    config.circularity_thresh = 0.74f;       // Soglia forma cerchio
    config.circle_aspect_ratio_thresh = 0.68f;
    config.circle_fill_thresh = 0.48f;
    config.show_windows = interactive_windows;
    config.wait_key_delay = 0;               // 0 = attende pressione tasto per ciascuna finestra

    pcb::CvComponentDetector detector;

    std::vector<fs::path> board_paths;
    for (const auto& entry : fs::directory_iterator(input_base_dir)) {
        if (entry.is_directory()) {
            board_paths.push_back(entry.path());
        }
    }

    if (board_paths.empty()) {
        std::cout << "Nessuna sottocartella scheda trovata in " << input_base_dir << "." << std::endl;
        return true;
    }

    std::sort(board_paths.begin(), board_paths.end());

    auto processSideCvDetection = [&](const fs::path& side_dir, const std::string& board_name, const std::string& side) {
        fs::path out_side_dir = fs::path(detection_output_base_dir) / board_name / side;
        fs::create_directories(out_side_dir);

        std::vector<std::string> valid_exts = {".bmp", ".png", ".jpg", ".jpeg", ".tiff", ".tif"};
        std::vector<fs::path> image_files;
        for (const auto& entry : fs::directory_iterator(side_dir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
                if (std::find(valid_exts.begin(), valid_exts.end(), ext) != valid_exts.end()) {
                    image_files.push_back(entry.path());
                }
            }
        }

        std::sort(image_files.begin(), image_files.end());
        if (image_files.empty()) {
            std::cout << "Nessuna immagine trovata in " << side_dir << "." << std::endl;
            return;
        }

        std::cout << "\nElaborazione Computer Vision per " << board_name << " [" << side << "] ("
                  << image_files.size() << " immagini)..." << std::endl;

        for (const auto& img_path : image_files) {
            std::cout << "\n-> Analisi filtri e bordi su: " << img_path.filename().string() << " ..." << std::endl;
            cv::Mat img = cv::imread(img_path.string());
            if (img.empty()) {
                std::cerr << "ERRORE: Impossibile leggere l'immagine " << img_path << std::endl;
                continue;
            }

            // Esecuzione pipeline Computer Vision
            auto result = detector.detect(img, config);
            std::string stem = img_path.stem().string();

            // Salvataggio dei risultati (immagine annotata, maschere filtri, CSV, JSON)
            detector.saveResults(out_side_dir.string(), stem, img, result, true);

            std::cout << "   Componenti totali trovati: " << result.components.size() << std::endl;
            std::cout << "     - Rettangoli orientati/ruotati: " << result.num_rectangles << std::endl;
            std::cout << "     - Cerchi: " << result.num_circles << std::endl;

            // Mostra la finestra con il risultato (se abilitato)
            if (config.show_windows) {
                std::string win_title = "Riconoscimento CV Componenti: " + board_name + " [" + side + "] - " + stem;
                detector.displayResult(win_title, result, config.wait_key_delay);
            }
        }

        std::cout << "Risultati salvati in: " << out_side_dir << std::endl;
    };

    for (const auto& bpath : board_paths) {
        std::string bname = bpath.filename().string();
        std::cout << "\n--- Elaborazione CV Componenti per scheda: " << bname << " ---" << std::endl;

        fs::path top_dir = bpath / "TOP";
        fs::path bot_dir = bpath / "BOTTOM";

        if (fs::exists(top_dir) && !fs::is_empty(top_dir)) {
            processSideCvDetection(top_dir, bname, "TOP");
        }
        if (fs::exists(bot_dir) && !fs::is_empty(bot_dir)) {
            processSideCvDetection(bot_dir, bname, "BOTTOM");
        }
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << "Rilevamento CV classica completato con successo!" << std::endl;
    std::cout << "Immagini annotate, maschere, CSV e JSON salvati in:\n  " << detection_output_base_dir << std::endl;
    std::cout << "========================================================" << std::endl;

    return true;
}

int main() {
    if (!runRobotCameraCalibration()) {
        return -1;
    }

    if (!runRobotImagesUndistortion()) {
        return -1;
    }

    std::cout << "\n========================================================" << std::endl;
    std::cout << "        MODALITÀ DI RILEVAMENTO COMPONENTI PCB          " << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "1) Computer Vision Classica (Filtro verde PCB, bordi, rettangoli ruotati e cerchi)" << std::endl;
    std::cout << "2) Deep Learning YOLO (Modello a 2 stadi)" << std::endl;
    std::cout << "3) Entrambi (prima CV Classica, poi YOLO)" << std::endl;
    std::cout << "Seleziona un'opzione (1 / 2 / 3, default 1): ";

    std::string choice_str;
    int mode_choice = 1;
    if (std::cin >> choice_str) {
        try {
            mode_choice = std::stoi(choice_str);
        } catch (...) {
            mode_choice = 1;
        }
    }

    bool show_windows = true;
    std::cout << "\nVuoi mostrare le finestre a schermo con i risultati per ciascuna scheda? (S/N, default S): ";
    std::string win_ans;
    if (std::cin >> win_ans) {
        std::transform(win_ans.begin(), win_ans.end(), win_ans.begin(), [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
        if (win_ans == "N" || win_ans == "NO") {
            show_windows = false;
        }
    }

    if (mode_choice == 1 || mode_choice == 3) {
        if (!runPcbCvComponentDetection(show_windows)) {
            return -1;
        }
    }

    if (mode_choice == 2 || mode_choice == 3) {
        if (!runPcbComponentDetection()) {
            return -1;
        }
    }

    std::cout << "\nPipeline complessiva completata con successo." << std::endl;
    return 0;
}
