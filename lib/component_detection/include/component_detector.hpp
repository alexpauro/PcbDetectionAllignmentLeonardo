#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace pcb {

/**
 * @brief Configurazione dell'inferenza e della visualizzazione per il rilevamento componenti PCB.
 */
struct DetectionConfig {
    float conf_threshold = 0.20f;       ///< Soglia minima di confidenza
    float nms_threshold = 0.50f;        ///< Soglia IoU per NMS (agnostico o a tasselli)
    float match_iou = 0.30f;            ///< Soglia minima IoU per associare la classe 'fine' alla box 'any'
    int imgsz = 1024;                   ///< Dimensione di input della rete YOLO (default 1024)
    int tile_size = 0;                  ///< Dimensione tassello in px (0 = disattivato, es. 640 o 1024)
    float overlap = 0.25f;              ///< Sovrapposizione tra tasselli (es. 0.25 = 25%)
    int max_det = 1000;                 ///< Numero massimo di rilevamenti consentiti
    float scale = 1.0f;                 ///< Fattore di scala iniziale per l'immagine
    std::string label_display_mode = "auto"; ///< Modalità etichette: "auto", "ids", "names", "none"
    int min_label_px = 24;              ///< Pixel minimi del lato corto della box per disegnare il testo (a scala di riferimento)
    bool use_cuda = false;              ///< Utilizza OpenCV DNN con backend CUDA se disponibile
};

/**
 * @brief Informazioni su un singolo componente rilevato.
 */
struct ComponentDetection {
    int id = 0;                         ///< ID progressivo (1-based)
    cv::Rect2f box;                     ///< Bounding box [x, y, width, height] in coordinate dell'immagine
    std::string label = "?";            ///< Nome della classe (es. "Resistor", "IC", "?")
    float box_conf = 0.0f;              ///< Confidenza della bounding box (modello 'any' o fine)
    float label_conf = 0.0f;            ///< Confidenza di classificazione del tipo
    int class_id = -1;                  ///< ID numerico della classe (-1 se sconosciuto)
};

/**
 * @brief Risultato complessivo dell'inferenza su un'immagine.
 */
struct DetectionResult {
    std::vector<ComponentDetection> detections;   ///< Lista di tutti i componenti identificati
    std::map<std::string, int> counts;            ///< Conteggio dei componenti per ciascuna classe
    cv::Mat elements_vis;                         ///< Immagine Step 1 (WHERE): tutti i componenti evidenziati in verde con legenda
    cv::Mat types_vis;                            ///< Immagine Step 2 (WHAT): componenti colorati per tipo con etichette e legenda
};

/**
 * @brief Classe principale per il rilevamento e la classificazione di componenti PCB.
 * 
 * Supporta la pipeline a due stadi:
 * - "any": rilevatore di posizione componenti (class-agnostic)
 * - "fine": classificatore multi-classe a 13 categorie
 * - "both": combinazione dei due stadi (any individua la posizione, fine assegna la classe)
 * 
 * Può caricare pesi ONNX (.onnx) o convertire/individuare automaticamente pesi PyTorch (.pt).
 */
class ComponentDetector {
public:
    ComponentDetector();
    ~ComponentDetector() = default;

    /**
     * @brief Costruttore con percorsi dei pesi.
     */
    ComponentDetector(const std::string& any_weights_path,
                      const std::string& fine_weights_path = "",
                      const std::vector<std::string>& custom_classes = {});

    /**
     * @brief Carica il modello per il rilevamento elementi generici ('any').
     * @param weights_path Percorso al file .onnx o .pt.
     * @param imgsz Dimensione input per eventuale export .pt.
     * @return true se il caricamento ha successo.
     */
    bool loadAnyModel(const std::string& weights_path, int imgsz = 1024);

    /**
     * @brief Carica il modello multi-classe ('fine').
     * @param weights_path Percorso al file .onnx o .pt.
     * @param class_names Vettore dei nomi delle classi (se vuoto, usa le 13 classi standard).
     * @param imgsz Dimensione input per eventuale export .pt.
     * @return true se il caricamento ha successo.
     */
    bool loadFineModel(const std::string& weights_path,
                       const std::vector<std::string>& class_names = {},
                       int imgsz = 1024);


    /**
     * @brief Esegue il rilevamento dei componenti su un'immagine.
     * @param image Immagine BGR di input.
     * @param config Parametri di rilevamento.
     * @param mode Modalità: "both" (default), "any", "fine".
     * @return Struttura DetectionResult contenente box, conteggi e immagini renderizzate.
     */
    DetectionResult detect(const cv::Mat& image,
                           const DetectionConfig& config = DetectionConfig(),
                           const std::string& mode = "both");

    /**
     * @brief Genera l'immagine dello Step 1 (WHERE): componenti in verde brillante con riempimento e legenda.
     */
    static cv::Mat drawElements(const cv::Mat& img,
                                const std::vector<ComponentDetection>& detections,
                                int target = 2400,
                                float fill = 0.28f);

    /**
     * @brief Genera l'immagine dello Step 2 (WHAT): componenti con codice colore per tipo, etichette e pannello legenda.
     */
    static std::pair<cv::Mat, std::map<std::string, int>> drawTypes(
        const cv::Mat& img,
        const std::vector<ComponentDetection>& detections,
        const std::string& label_mode = "auto",
        int min_label_px = 24,
        int target = 2400);

    /**
     * @brief Salva i risultati (immagini per ogni step, file CSV e file JSON).
     * @param output_dir Cartella di destinazione.
     * @param file_stem Nome base del file (senza estensione).
     * @param orig_img Immagine originale.
     * @param result Risultati dell'inferenza.
     * @param mode Modalità utilizzata ("both", "any", "fine").
     * @return true se il salvataggio ha avuto successo.
     */
    static bool saveResults(const std::string& output_dir,
                            const std::string& file_stem,
                            const cv::Mat& orig_img,
                            const DetectionResult& result,
                            const std::string& mode = "both");

    /**
     * @brief Assicura che un percorso a un modello sia in formato ONNX.
     * Se il percorso termina in .pt, cerca un file .onnx corrispondente o tenta la conversione.
     */
    static std::string ensureOnnxModel(const std::string& weights_path, int imgsz = 1024);

    /**
     * @brief Restituisce la palette di colori deterministica per le etichette fornite.
     */
    static std::map<std::string, cv::Scalar> getPalette(const std::vector<std::string>& labels);

    /**
     * @brief Restituisce il codice breve (abbreviazione) per un tipo di componente.
     */
    static std::string getShortLabel(const std::string& full_label);

    /**
     * @brief Restituisce l'elenco delle 13 classi predefinite del dataset ParsaVictor.
     */
    static std::vector<std::string> getDefaultClasses();

    /**
     * @brief Calcola l'Intersection over Union (IoU) tra due rettangoli.
     */
    static float calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b);

private:
    cv::dnn::Net net_any_;
    cv::dnn::Net net_fine_;
    bool has_any_ = false;
    bool has_fine_ = false;

    std::vector<std::string> fine_classes_;

    // Inferenza interna YOLOv8 / YOLO26
    std::vector<ComponentDetection> runInference(
        cv::dnn::Net& net,
        const cv::Mat& img,
        float conf_threshold,
        float nms_threshold,
        int imgsz,
        int tile_size,
        float overlap,
        int max_det,
        int num_classes,
        const std::vector<std::string>& class_names,
        bool agnostic_nms = false);

    // Inferenza su singolo tassello o immagine ridimensionata
    void inferTile(
        cv::dnn::Net& net,
        const cv::Mat& tile,
        int offset_x,
        int offset_y,
        float conf_threshold,
        int imgsz,
        int num_classes,
        const std::vector<std::string>& class_names,
        std::vector<cv::Rect2f>& out_boxes,
        std::vector<float>& out_scores,
        std::vector<int>& out_classes);
};

} // namespace pcb

