#include "component_segmentation.hpp"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "TEST STANDALONE LIBRERIA COMPONENT SEGMENTATION" << std::endl;
    std::cout << "========================================================" << std::endl;

    std::string undistorted_base_dir = "data/outputs/pcb_images_undistorted";
    std::string aoi_verification_dir = "data/outputs/pcb_aoi_verification";
    std::string params_dir = "data/parameters";
    std::string output_base_dir = "data/outputs/pcb_component_segmentation";

    // Prova sia da directory corrente che da ../
    if (!fs::exists(aoi_verification_dir)) {
        undistorted_base_dir = "../data/outputs/pcb_images_undistorted";
        aoi_verification_dir = "../data/outputs/pcb_aoi_verification";
        params_dir = "../data/parameters";
        output_base_dir = "../data/outputs/pcb_component_segmentation";
    }

    pcb::ComponentSegmentationProcessor segmenter;
    pcb::ComponentSegmentationConfig config;
    config.show_windows = false; // no GUI blocking durante i test

    auto result = segmenter.processBoardSide(
        "A",
        "TOP",
        undistorted_base_dir,
        aoi_verification_dir,
        params_dir,
        output_base_dir,
        config
    );

    if (result.success && result.summary.successfully_segmented > 0) {
        std::cout << "\n>>> TEST COMPONENT SEGMENTATION COMPLETATO CON SUCCESSO! <<<" << std::endl;
        std::cout << "Componenti validati e segmentati: " << result.summary.successfully_segmented << std::endl;
        std::cout << "Componenti eliminati: " << (result.summary.rejected_not_found + result.summary.rejected_overlap_cataloged) << std::endl;
        return 0;
    } else {
        std::cerr << "\n>>> TEST COMPONENT SEGMENTATION FALLITO! <<<" << std::endl;
        return 1;
    }
}

