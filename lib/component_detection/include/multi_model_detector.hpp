#pragma once

#include "component_detector.hpp"
#include "class_mapper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

namespace pcb {

/**
 * @brief Inventario di qualsiasi file di pesi individuato durante la scansione.
 */
struct WeightFileInfo {
    std::string full_path;                ///< Percorso assoluto/canonico del file
    std::string relative_path;            ///< Percorso relativo alla root di ricerca
    uintmax_t size_bytes = 0;             ///< Dimensione del file in byte
    bool is_loadable = false;             ///< True se > 0 byte e formato valido
    std::string model_name;               ///< Nome dedotto per il modello
    std::string status_note;              ///< Nota diagnostica dettagliata (es. "0 byte", "pronto", ecc.)
};

/**
 * @brief Informazioni su un modello di component detection caricato e pronto.
 */
struct DiscoveredModelInfo {
    std::string name;                     ///< Nome identificativo del modello (es. "ParsaVictor", "custom_yolo")
    std::string directory;                ///< Cartella in cui risiede il modello
    std::string weights_path;             ///< Percorso al file pesi (.onnx o .pt)
    bool is_parsa_victor = false;         ///< True se è il modello di riferimento ParsaVictor
    bool is_two_stage = false;            ///< True se utilizza pipeline a 2 stadi (any + fine)
    std::string any_weights_path;         ///< Pesi ANY (se 2 stadi)
    std::string fine_weights_path;        ///< Pesi FINE (se 2 stadi)
    std::vector<std::string> raw_classes; ///< Nomi delle classi originali del modello
    std::vector<std::string> mapped_classes; ///< Nomi delle classi mappate su ParsaVictor
    int imgsz = 1024;                     ///< Risoluzione input della rete
    float priority_weight = 1.0f;         ///< Moltiplicatore di affidabilità (ParsaVictor = 1.25f, altri = 1.0f)
    std::shared_ptr<ComponentDetector> two_stage_detector; ///< Detector per ParsaVictor
    cv::dnn::Net single_stage_net;        ///< Rete OpenCV DNN per modelli a singolo stadio
    int num_classes = 0;                  ///< Numero di classi originali
    bool is_loaded = false;               ///< True se caricato e pronto per l'inferenza
};

/**
 * @brief Orchestratore per il rilevamento avanzato multi-modello.
 *
 * Scansiona approfonditamente pcb_component_detection, compila l'inventario completo
 * di tutti i file pesi presenti, fornisce diagnostica dettagliata per file non validi/vuoti,
 * unifica le classi basandosi su ParsaVictor tramite ClassMapper, esegue l'inferenza
 * con tutti i modelli attivi e fonde i risultati risolvendo eventuali contrasti
 * a favore della rilevazione con maggiore affidabilità (ParsaVictor > altri DL > CV).
 */
class MultiModelDetector {
public:
    explicit MultiModelDetector(
        const std::string& base_models_dir = "../pcb_component_detection",
        const std::string& mapping_file_path = "../data/parameters/class_mapping_weights.txt");
    ~MultiModelDetector() = default;

    /**
     * @brief Aggiunge una directory di ricerca addizionale per i pesi.
     */
    void addSearchDirectory(const std::string& dir);

    /**
     * @brief Aggiunge esplicitamente un percorso a un file pesi (.pt o .onnx).
     */
    bool addExplicitWeightFile(const std::string& weights_path, const std::string& custom_model_name = "");

    /**
     * @brief Scansiona la cartella base e tutte le cartelle aggiuntive,
     * producendo un report dettagliato di tutti i file pesi trovati.
     * @param interactive_mapping Se true, chiede all'utente da console in caso di classi incerte.
     * @return int Numero di modelli validi caricati con successo.
     */
    int scanAndLoadModels(bool interactive_mapping = true);

    /**
     * @brief Restituisce l'inventario completo di tutti i file pesi individuati sul disco.
     */
    const std::vector<WeightFileInfo>& getAllDiscoveredFiles() const { return discovered_files_; }

    /**
     * @brief Restituisce l'elenco dei modelli validi caricati e pronti all'inferenza.
     */
    const std::vector<DiscoveredModelInfo>& getDiscoveredModels() const { return models_; }

    /**
     * @brief Esegue il rilevamento unificato combinando tutti i modelli DL disponibili.
     * In caso di contrasto di classificazione sullo stesso componente,
     * mantiene quello con maggiore affidabilità (reliability = confidence * priority_weight).
     * @param image Immagine BGR di input.
     * @param config Parametri di rilevamento (soglie, imgsz, NMS, ecc.).
     * @return DetectionResult Risultato complessivo con componenti unificati, conteggi e visualizzazioni.
     */
    DetectionResult detect(const cv::Mat& image,
                           const DetectionConfig& config = DetectionConfig());

    /**
     * @brief Salva i risultati unificati dell'ensemble (CSV, JSON, immagini).
     */
    static bool saveResults(const std::string& output_dir,
                            const std::string& file_stem,
                            const cv::Mat& orig_img,
                            const DetectionResult& result);

    /**
     * @brief Imposta se utilizzare backend CUDA se disponibile.
     */
    void setUseCuda(bool use_cuda) { use_cuda_ = use_cuda; }

    /**
     * @brief Restituisce il gestore delle associazioni classi.
     */
    ClassMapper& getClassMapper() { return class_mapper_; }

private:
    std::string base_models_dir_;
    std::vector<std::string> additional_search_dirs_;
    ClassMapper class_mapper_;
    std::vector<WeightFileInfo> discovered_files_;
    std::vector<DiscoveredModelInfo> models_;
    bool use_cuda_ = false;

    // Risolve la directory radice dei modelli in modo robusto
    std::string resolveBaseModelsDir() const;

    // Esegue inferenza su un modello a singolo stadio (YOLOv5, YOLOv8, YOLO26, ecc.)
    std::vector<ComponentDetection> runSingleStageModel(
        DiscoveredModelInfo& model,
        const cv::Mat& img,
        const DetectionConfig& config);

    // Esegue inferenza su un tassello per modelli a singolo stadio
    void inferSingleStageTile(
        cv::dnn::Net& net,
        const cv::Mat& tile,
        int offset_x,
        int offset_y,
        float conf_threshold,
        int imgsz,
        int num_classes,
        const std::vector<std::string>& mapped_classes,
        std::vector<cv::Rect2f>& out_boxes,
        std::vector<float>& out_scores,
        std::vector<int>& out_classes);

    // Unisce e risolve i contrasti tra le rilevazioni dei diversi modelli
    std::vector<ComponentDetection> fuseModelDetections(
        const std::vector<std::pair<DiscoveredModelInfo*, std::vector<ComponentDetection>>>& all_dets,
        float match_iou_thresh = 0.30f);
};

} // namespace pcb
