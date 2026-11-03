#include "calibration.hpp"
#include <string>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

// Funzione per calibrare la telecamera Robot
bool runRobotCameraCalibration() {
    std::cout << "=== AVVIO CALIBRAZIONE TELECAMERA ===" << std::endl;

    // Creazione della cartella di output per i parametri se non esiste
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

int main() {
    if (!runRobotCameraCalibration()) {
        return -1;
    }

    std::cout << "\nOperazione completata con successo." << std::endl;
    return 0;
}