#include "class_mapper.hpp"
#include "multi_model_detector.hpp"
#include "component_detector.hpp"
#include <iostream>
#include <cassert>
#include <filesystem>

namespace fs = std::filesystem;

void testClassMapper() {
    std::cout << "\n========================================================" << std::endl;
    std::cout << " TEST 1: ClassMapper (Parole Chiave, Similarita', Mappatura)" << std::endl;
    std::cout << "========================================================" << std::endl;

    pcb::ClassMapper mapper("../data/parameters/class_mapping_weights.txt");

    // 1. Test corrispondenze esatte
    std::cout << "[Test] Corrispondenza esatta ParsaVictor..." << std::endl;
    assert(mapper.getOrMapClass("test_model", "Resistor", false) == "Resistor");
    assert(mapper.getOrMapClass("test_model", "Capacitor", false) == "Capacitor");
    assert(mapper.getOrMapClass("test_model", "IC", false) == "IC");
    assert(mapper.getOrMapClass("test_model", "Inductor", false) == "Inductor");
    assert(mapper.getOrMapClass("test_model", "Diode", false) == "Diode");
    assert(mapper.getOrMapClass("test_model", "LED", false) == "LED");
    assert(mapper.getOrMapClass("test_model", "Transistor", false) == "Transistor");
    assert(mapper.getOrMapClass("test_model", "Oscillator", false) == "Oscillator");
    assert(mapper.getOrMapClass("test_model", "Connector", false) == "Connector");
    assert(mapper.getOrMapClass("test_model", "Switch/Button", false) == "Switch/Button");
    assert(mapper.getOrMapClass("test_model", "Other", false) == "Other");
    assert(mapper.getOrMapClass("test_model", "Marker", false) == "Marker");
    std::cout << "  -> Superato con successo!" << std::endl;

    // 2. Test sinonimi e parole chiave
    std::cout << "[Test] Parole chiave e sinonimi avanzati..." << std::endl;
    assert(mapper.getOrMapClass("model_roboflow", "chip_resistor", false) == "Resistor");
    assert(mapper.getOrMapClass("model_roboflow", "multilayer_ceramic_capacitor", false) == "Capacitor");
    assert(mapper.getOrMapClass("model_roboflow", "electrolytic_cap", false) == "Electrolytic Capacitor");
    assert(mapper.getOrMapClass("model_roboflow", "circuito_integrado", false) == "IC");
    assert(mapper.getOrMapClass("model_roboflow", "ferrite_bead", false) == "Inductor");
    assert(mapper.getOrMapClass("model_roboflow", "zener_diode", false) == "Diode");
    assert(mapper.getOrMapClass("model_roboflow", "diodo_led", false) == "LED");
    assert(mapper.getOrMapClass("model_roboflow", "mosfet_sot23", false) == "Transistor");
    assert(mapper.getOrMapClass("model_roboflow", "crystal_oscillator", false) == "Oscillator");
    assert(mapper.getOrMapClass("model_roboflow", "pin_header_usb", false) == "Connector");
    assert(mapper.getOrMapClass("model_roboflow", "tact_switch", false) == "Switch/Button");
    assert(mapper.getOrMapClass("model_roboflow", "test_point_tp", false) == "Marker");
    assert(mapper.getOrMapClass("model_roboflow", "fuse_relay", false) == "Other");
    std::cout << "  -> Superato con successo!" << std::endl;

    // 3. Test tracciamento su file
    std::cout << "[Test] Persistenza e caricamento su file TXT..." << std::endl;
    std::string test_txt = "../data/parameters/test_mapping_output.txt";
    pcb::ClassMapper mapper_file(test_txt);
    mapper_file.addMapping("custom_yolo", "resistore_smd", "Resistor");
    assert(mapper_file.getOrMapClass("custom_yolo", "resistore_smd", false) == "Resistor");

    // Ricarica per verificare persistenza su disco
    pcb::ClassMapper mapper_reload(test_txt);
    assert(mapper_reload.getOrMapClass("custom_yolo", "resistore_smd", false) == "Resistor");
    fs::remove(test_txt);
    std::cout << "  -> Superato con successo!" << std::endl;
}

void testMultiModelScanningAndFusion() {
    std::cout << "\n========================================================" << std::endl;
    std::cout << " TEST 2: MultiModelDetector (Scansione, Inferenza e Fusione)" << std::endl;
    std::cout << "========================================================" << std::endl;

    pcb::MultiModelDetector multi_detector(
        "../pcb_component_detection",
        "../data/parameters/class_mapping_weights.txt"
    );

    int loaded = multi_detector.scanAndLoadModels(false);
    std::cout << "Modelli caricati: " << loaded << std::endl;
    assert(loaded >= 1);

    const auto& models = multi_detector.getDiscoveredModels();
    bool parsa_found = false;
    for (const auto& m : models) {
        if (m.name == "ParsaVictor") {
            parsa_found = true;
            assert(m.is_parsa_victor == true);
            assert(m.priority_weight > 1.0f);
        }
    }
    assert(parsa_found);
    (void)parsa_found;
    std::cout << "  -> ParsaVictor correttamente riconosciuto e prioritizzato!" << std::endl;

    // Test inferenza con immagine sintetica
    std::cout << "\n[Test] Inferenza e fusione su immagine di test..." << std::endl;
    cv::Mat test_img(800, 1000, CV_8UC3, cv::Scalar(40, 70, 30)); // Sfondo verde PCB
    pcb::DetectionConfig cfg;
    cfg.conf_threshold = 0.15f;
    cfg.nms_threshold = 0.40f;

    auto result = multi_detector.detect(test_img, cfg);
    std::cout << "Rilevamenti totali trovati su immagine sintetica: " << result.detections.size() << std::endl;
    std::cout << "Rendering elements vis: " << result.elements_vis.size() << std::endl;
    std::cout << "Rendering types vis: " << result.types_vis.size() << std::endl;
    assert(!result.elements_vis.empty());
    std::cout << "  -> Superato con successo!" << std::endl;
}

int main() {
    try {
        testClassMapper();
        testMultiModelScanningAndFusion();
        std::cout << "\n========================================================" << std::endl;
        std::cout << " TUTTI I TEST DEGLI ALGORITMI DL COMPLETATI CON SUCCESSO! " << std::endl;
        std::cout << "========================================================" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERRORE NEL TEST: " << e.what() << std::endl;
        return -1;
    }
}
