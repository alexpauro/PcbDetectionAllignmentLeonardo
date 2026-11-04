#include "aoi_verification.hpp"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "TEST STANDALONE LIBRERIA AOI VERIFICATION" << std::endl;
    std::cout << "========================================================" << std::endl;

    std::string undistorted_base_dir = "data/outputs/pcb_images_undistorted";
    std::string processed_output_base_dir = "data/outputs/pcb_processed_output";
    std::string aoi_base_dir = "data/inputs/pcb_coordinates_componentes_from_aoi";
    std::string gerber_base_dir = "data/inputs/pcb_gerber";
    std::string params_dir = "data/parameters";
    std::string verification_output_base_dir = "data/outputs/pcb_aoi_verification";

    // Prova sia da directory corrente che da ../
    if (!fs::exists(aoi_base_dir)) {
        undistorted_base_dir = "../data/outputs/pcb_images_undistorted";
        processed_output_base_dir = "../data/outputs/pcb_processed_output";
        aoi_base_dir = "../data/inputs/pcb_coordinates_componentes_from_aoi";
        gerber_base_dir = "../data/inputs/pcb_gerber";
        params_dir = "../data/parameters";
        verification_output_base_dir = "../data/outputs/pcb_aoi_verification";
    }

    pcb::AoiVerificationProcessor verifier;

    bool success = verifier.processBoardSide(
        "A",
        "TOP",
        undistorted_base_dir,
        processed_output_base_dir,
        aoi_base_dir,
        gerber_base_dir,
        params_dir,
        verification_output_base_dir,
        false // no GUI blocking
    );

    if (success) {
        std::cout << "\n>>> TEST COMPLETATO CON SUCCESSO! <<<" << std::endl;
        return 0;
    } else {
        std::cerr << "\n>>> TEST FALLITO! <<<" << std::endl;
        return 1;
    }
}

