#include "calibration.hpp"
#include "undistort.hpp"
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

// Funzione per raddrizzare le immagini PCB con loop interattivo (o automatico se c'è una sola scheda)
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

    // SE C'È UNA SOLA SCHEDA: procedi direttamente senza chiedere quale scheda scegliere
    if (board_paths.size() == 1) {
        fs::path selected_board_path = board_paths[0];
        std::string board_name = selected_board_path.filename().string();

        std::cout << "\nTrovata un'unica scheda: " << board_name << ". Procedo automaticamente." << std::endl;

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

int main() {
    if (!runRobotCameraCalibration()) {
        return -1;
    }

    if (!runRobotImagesUndistortion()) {
        return -1;
    }

    std::cout << "\nOperazione completata con successo." << std::endl;
    return 0;
}