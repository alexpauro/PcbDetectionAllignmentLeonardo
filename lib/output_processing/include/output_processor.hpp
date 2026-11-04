#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <filesystem>
#include "component_detector.hpp"
#include "cv_component_detector.hpp"

namespace pcb {

namespace fs = std::filesystem;

/**
 * @brief Origine della rilevazione per il componente finale fuso.
 */
enum class ComponentSource {
    DL_ONLY,                ///< Individuato solo dal Deep Learning (non intercettato da CV)
    CV_ONLY,                ///< Individuato solo da Computer Vision e classificato con pesi DL
    FUSED_DL_CV,            ///< Confermato sia da DL che da CV (geometria CV + classificazione DL)
    SIMILARITY_DISCOVERED   ///< Scoperto tramite ricerca visiva globale su intera scheda
};

/**
 * @brief Singola osservazione multi-esposizione da una specifica foto.
 */
struct MultiExposureObservation {
    std::string image_stem;            ///< Nome file/stem dell'immagine (es. "A_TOP_003")
    ComponentDetection detection;       ///< Rilevamento originale
};

/**
 * @brief Componente risultante dalla fusione multi-esposizione DL (Step 1).
 */
struct FusedDlComponent {
    int id = 0;                         ///< ID progressivo (1-based)
    cv::Rect2f box;                     ///< Bounding box rappresentativa
    std::string label = "?";            ///< Classe vincente
    float box_conf = 0.0f;              ///< Confidenza della bounding box
    float label_conf = 0.0f;            ///< Confidenza di classificazione della classe vincente
    int class_id = -1;                  ///< ID numerico della classe vincente
    int vote_count = 0;                 ///< Numero di voti per la classe vincente
    int total_observations = 0;         ///< Numero di foto in cui è stato osservato
    std::map<std::string, int> votes;   ///< Dettaglio voti per ciascuna classe
    std::string best_source_image;      ///< Immagine con la confidenza più alta per questo componente
};

/**
 * @brief Componente geometrico CV classificato tramite pesi Deep Learning (Step 2).
 */
struct ClassifiedCvComponent {
    int id = 0;                         ///< ID progressivo (1-based)
    CvComponent cv_geom;                ///< Geometria CV (cerchio, rettangolo ruotato, centro, ecc.)
    std::string label = "?";            ///< Classe assegnata tramite pesi DL
    float label_conf = 0.0f;            ///< Confidenza assegnata dal modello DL
    int class_id = -1;                  ///< ID numerico della classe
    bool matched_with_dl = false;       ///< True se abbinato ad un rilevamento DL multi-esposizione
    std::string classification_method;  ///< "DL_SPATIAL_MATCH", "DL_PATCH_INFERENCE", "UNCLASSIFIED"
};

/**
 * @brief Componente finale unificato completo (Step 3).
 */
struct FinalComponent {
    int id = 0;                                 ///< ID finale progressivo (1-based)
    ComponentSource source = ComponentSource::DL_ONLY; ///< Origine della rilevazione
    std::string label = "?";                    ///< Etichetta finale di classe
    float confidence = 0.0f;                    ///< Confidenza finale
    cv::Rect2f box;                             ///< Bounding box allineata agli assi
    ShapeType shape = ShapeType::RECTANGLE;     ///< Forma geometrica (RECTANGLE o CIRCLE)
    cv::RotatedRect rotated_box;                ///< Rettangolo orientato aderente (se disponibile da CV)
    cv::Point2f center;                         ///< Coordinate del centro (x, y)
    float radius = 0.0f;                        ///< Raggio (se shape == CIRCLE)
    float area = 0.0f;                          ///< Area geometrica
    std::string notes;                          ///< Note dettagliate sulla fusione
};

/**
 * @brief Informazioni su un cluster di componenti visivamente identici rilevati via Computer Vision.
 */
struct SimilarityClusterInfo {
    int cluster_id = 0;                         ///< ID del cluster di similarità
    std::string winning_label = "?";            ///< Classe vincente assegnata al cluster
    float consensus_confidence = 0.0f;          ///< Confidenza di consenso
    int member_count = 0;                       ///< Numero totale di componenti identici nel cluster
    int harmonized_count = 0;                   ///< Quanti componenti hanno cambiato etichetta/sono stati promossi da '?'
    std::vector<int> member_ids;                ///< ID dei componenti appartenenti a questo cluster
    std::map<std::string, int> original_labels; ///< Conteggio etichette originali prima dell'armonizzazione
};

/**
 * @brief Configurazione della pipeline di elaborazione e fusione output.
 */
struct OutputProcessingConfig {
    float dl_merge_iou_thresh = 0.35f;   ///< IoU minimo tra foto per considerare lo stesso componente
    float cv_dl_match_iou_thresh = 0.25f;///< IoU minimo per associare componente CV a componente DL
    float min_conf_for_label = 0.15f;    ///< Confidenza minima per assegnare una classe invece di "?"
    bool enable_patch_inference = false; ///< Se abilitare l'inferenza fine locale su crop per CV non abbinati (default false per velocità)
    int patch_padding = 24;              ///< Pixel di padding attorno al componente per inferenza su crop
    int patch_target_size = 1024;        ///< Dimensione input YOLO per la patch

    // Modalità di rilevamento (1 = Solo CV Classica, 2 = Solo Deep Learning YOLO, 3 = Entrambi)
    int detection_mode = 3;              ///< Modalità attiva per l'elaborazione dei dati

    // Selezione immagine base
    bool interactive_image_selection = false; ///< Chiede all'utente quale immagine usare come base per l'output finale
    std::string preferred_base_image = "";    ///< Nome/stem dell'immagine base desiderata (se vuoto, usa la consigliata)

    // Filtro elementi CV minuscoli non catalogati
    float tiny_cv_max_area = 45.0f;      ///< Area massima per considerare un elemento CV "molto molto piccolo"
    float tiny_cv_max_dim = 7.5f;        ///< Dimensione massima (lato corto/lungo) per considerare un elemento minuscolo

    // Risoluzione componenti sovrapposti/inglobati (no sovrapposizioni su PCB)
    float enclosure_overlap_thresh = 0.80f; ///< Soglia di inclusione per considerare un componente completamente compreso in un altro

    // Armonizzazione componenti identici tramite Computer Vision (Fase 1: componenti noti)
    bool enable_similarity_harmonization = true; ///< Attiva l'armonizzazione della classificazione per componenti identici
    float similarity_geom_tol = 0.18f;           ///< Tolleranza percentuale sulle dimensioni geometriche (18%)
    float similarity_ncc_thresh = 0.78f;         ///< Soglia minima di Normalized Cross-Correlation (NCC)
    float similarity_hist_thresh = 0.35f;        ///< Soglia massima di distanza Bhattacharyya per istogramma colore HSV

    // Ricerca visiva globale su intera scheda per scoprire nuovi componenti simili (Fase 2: estensione su scheda)
    bool enable_whole_board_search = true;       ///< Cerca porzioni simili sull'intera scheda per scoprire nuovi componenti
    float search_ncc_thresh = 0.78f;             ///< Soglia minima di correlazione NCC per la ricerca visiva globale
    float search_hist_thresh = 0.38f;            ///< Soglia massima distanza Bhattacharyya per verifica colore
    float search_max_existing_iou = 0.20f;       ///< IoU massimo con componenti esistenti per evitare duplicati

    bool save_intermediate_images = true;///< Se salvare tutte le immagini dei passaggi intermedi
    std::string output_base_dir = "../data/outputs/pcb_processed_output"; ///< Cartella base di salvataggio
};

/**
 * @brief Risultato completo dell'elaborazione per un lato della scheda PCB.
 */
struct OutputProcessingResult {
    std::string board_name;
    std::string side;
    std::string best_image_stem;         ///< Nome/stem dell'immagine di riferimento con più componenti
    fs::path best_image_path;            ///< Percorso al file dell'immagine di riferimento
    cv::Mat best_image;                  ///< Immagine di riferimento caricata

    // Step 1: Fusione multi-esposizione DL
    std::vector<FusedDlComponent> step1_dl_merged;
    std::map<std::string, int> step1_counts;
    cv::Mat step1_elements_vis;
    cv::Mat step1_types_vis;

    // Step 2: Componenti CV classificati tramite pesi DL
    std::vector<ClassifiedCvComponent> step2_cv_classified;
    std::map<std::string, int> step2_counts;
    cv::Mat step2_cv_elements_vis;
    cv::Mat step2_cv_types_vis;

    // Step 3: Risultato finale unificato
    std::vector<FinalComponent> step3_final_components;
    std::map<std::string, int> step3_final_counts;
    std::map<std::string, int> source_counts; // "FUSED_DL_CV", "DL_ONLY", "CV_ONLY", "SIMILARITY_DISCOVERED"
    cv::Mat step3_final_elements_vis;
    cv::Mat step3_final_types_vis;
    cv::Mat step3_final_geometric_vis;

    // Armonizzazione similarità visiva CV & Ricerca globale
    std::vector<SimilarityClusterInfo> similarity_clusters; ///< Dettaglio cluster di componenti visivamente identici
    int harmonized_components_count = 0;                    ///< Numero di componenti ri-classificati per consenso di similarità
    int discovered_components_count = 0;                    ///< Numero di nuovi componenti scoperti dalla ricerca globale su scheda
};

/**
 * @brief Libreria per l'elaborazione, armonizzazione e fusione avanzata dei risultati di rilevamento.
 *
 * Svolge in sequenza:
 * 1. Selezione automatica della foto con più componenti rilevati come riferimento/canvas.
 * 2. Fusione multi-esposizione dei rilevamenti Deep Learning (priorità no-?, voto di maggioranza con tie-break).
 * 3. Classificazione dei componenti geometrici Computer Vision tramite pesi Deep Learning (matching spaziale e inferenza su patch).
 * 4. Fusione unificata di DL + CV per combinare precisione geometrica e potere classificativo.
 * 5. Salvataggio completo delle foto e dei report di tutti i passaggi intermedi.
 */
class OutputProcessor {
public:
    OutputProcessor() = default;
    ~OutputProcessor() = default;

    /**
     * @brief Esegue l'intera pipeline di elaborazione output per un lato della scheda.
     * @param board_name Nome della scheda (es. "A").
     * @param side Lato della scheda ("TOP" o "BOTTOM").
     * @param images_dir Cartella contenente le immagini della scheda (es. pcb_images_undistorted/A/TOP).
     * @param dl_output_dir Cartella contenente gli output DL (es. pcb_component_detection/A/TOP).
     * @param cv_output_dir Cartella contenente gli output CV (es. pcb_cv_component_detection/A/TOP).
     * @param dl_detector Puntatore facoltativo al ComponentDetector per patch inference diretta.
     * @param cv_detector Puntatore facoltativo a CvComponentDetector.
     * @param config Configurazione della pipeline.
     * @return OutputProcessingResult Struttura completa con componenti, conteggi e immagini intermedie.
     */
    OutputProcessingResult processBoardSide(
        const std::string& board_name,
        const std::string& side,
        const std::string& images_dir,
        const std::string& dl_output_dir,
        const std::string& cv_output_dir,
        ComponentDetector* dl_detector = nullptr,
        CvComponentDetector* cv_detector = nullptr,
        const OutputProcessingConfig& config = OutputProcessingConfig());

    // =========================================================================
    // STEP 1: Fusione multi-esposizione DL
    // =========================================================================
    /**
     * @brief Unisce i componenti DL provenienti da foto a diversa esposizione/luminosità.
     * Applica le regole:
     * - Priorità assoluta ai componenti etichettati senza punto di domanda.
     * - Per classi concorrenti: priorità alla classe catalogata più volte (maggioranza).
     * - In caso di parità: classe con confidenza maggiore.
     */
    static std::vector<FusedDlComponent> mergeMultiExposureDl(
        const std::map<std::string, std::vector<ComponentDetection>>& all_image_detections,
        const std::string& best_image_stem,
        float iou_threshold = 0.35f);

    // =========================================================================
    // STEP 2: Riconoscimento componenti CV tramite pesi DL
    // =========================================================================
    /**
     * @brief Classifica i componenti rilevati via Computer Vision usando i pesi del Deep Learning.
     * Utilizza matching spaziale con le detection DL e, facoltativamente, inferenza su crop locali con net_fine.
     */
    static std::vector<ClassifiedCvComponent> classifyCvComponentsWithDl(
        const std::vector<CvComponent>& cv_components,
        const std::vector<FusedDlComponent>& merged_dl,
        const cv::Mat& best_image,
        ComponentDetector* dl_detector = nullptr,
        float match_iou_threshold = 0.25f,
        float min_conf = 0.15f,
        bool enable_patch_inference = true);

    // =========================================================================
    // STEP 3: Fusione finale completa (DL + CV) & Risoluzione Sovrapposizioni
    // =========================================================================
    /**
     * @brief Unisce i risultati DL fusi (Step 1) con i risultati CV classificati (Step 2).
     * Mantiene la geometria precisa di CV (rettangoli ruotati e cerchi) e la catalogazione DL.
     * Filtra elementi CV microscopici non catalogati ('?').
     */
    static std::vector<FinalComponent> fuseDlAndCv(
        const std::vector<FusedDlComponent>& merged_dl,
        const std::vector<ClassifiedCvComponent>& classified_cv,
        float match_iou_threshold = 0.25f,
        float tiny_cv_max_area = 45.0f,
        float tiny_cv_max_dim = 7.5f);

    /**
     * @brief Risolve sovrapposizioni e componenti inglobati:
     * Su un PCB non ci sono componenti sovrapposti (possono essere vicini ma non sovrapposti).
     * Elimina componenti più grandi che racchiudono componenti più piccoli, conservando
     * gli elementi con maggiore confidenza e più piccoli.
     */
    static std::vector<FinalComponent> resolveEnclosingAndOverlaps(
        const std::vector<FinalComponent>& comps,
        float enclosure_thresh = 0.45f);

    /**
     * @brief Verifica se un elemento CV è microscopico e non catalogato da DL ('?').
     */
    static bool isTinyUnclassified(
        const CvComponent& cv_c,
        const std::string& label,
        float max_area = 45.0f,
        float max_dim = 7.5f);

    /**
     * @brief Identifica componenti visivamente e geometricamente identici usando esclusivamente
     * Computer Vision (forma, dimensioni geometriche, istogramma colore HSV, template matching NCC con rotazione)
     * e ne armonizza la classificazione assegnando a tutti la classe più probabile (consenso/maggioranza).
     * @param comps Vettore di componenti finali (Step 3).
     * @param image Immagine base di riferimento per l'estrazione dei crop visuali.
     * @param geom_tol Tolleranza relativa sulle dimensioni geometriche (default 0.18 = 18%).
     * @param ncc_thresh Soglia minima di correlazione normalizzata NCC (default 0.78).
     * @param hist_thresh Soglia massima di distanza Bhattacharyya tra istogrammi colore (default 0.35).
     * @param clusters_out Output opzionale per le informazioni dettagliate sui cluster identificati.
     * @return Vettore di componenti con classificazioni armonizzate.
     */
    static std::vector<FinalComponent> harmonizeIdenticalComponents(
        const std::vector<FinalComponent>& comps,
        const cv::Mat& image,
        float geom_tol = 0.18f,
        float ncc_thresh = 0.78f,
        float hist_thresh = 0.35f,
        std::vector<SimilarityClusterInfo>* clusters_out = nullptr);

    /**
     * @brief Cerca porzioni visivamente identiche/simili ai cluster noti sull'intera superficie del PCB.
     * Quando trova corrispondenze valide non ancora coperte da altri componenti, le classifica
     * secondo la classe del cluster e introduce i nuovi componenti trovati.
     * @param existing_comps Componenti già riconosciuti e armonizzati.
     * @param image Immagine base di riferimento per la scansione.
     * @param clusters Elenco dei cluster di componenti simili con classe vincente nota.
     * @param search_ncc_thresh Soglia minima correlazione NCC (default 0.83).
     * @param search_hist_thresh Soglia massima distanza Bhattacharyya istogramma colore (default 0.35).
     * @param max_existing_iou IoU massimo per considerare una posizione già occupata da componenti noti.
     * @return Vettore di nuovi componenti scoperti.
     */
    static std::vector<FinalComponent> searchSimilarComponentsOnBoard(
        const std::vector<FinalComponent>& existing_comps,
        const cv::Mat& image,
        const std::vector<SimilarityClusterInfo>& clusters,
        float search_ncc_thresh = 0.78f,
        float search_hist_thresh = 0.38f,
        float max_existing_iou = 0.20f);

    // =========================================================================
    // Salvataggio di tutti i passaggi intermedi
    // =========================================================================
    /**
     * @brief Salva su disco tutte le immagini, CSV e JSON di tutti i passaggi intermedi.
     */
    static bool saveAllStepResults(
        const OutputProcessingResult& res,
        const std::string& output_dir);

    // =========================================================================
    // Funzioni di Rendering Grafico Intermedio e Finale
    // =========================================================================
    static cv::Mat renderStep1Elements(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps);
    static cv::Mat renderStep1Types(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps);
    static cv::Mat renderStep2CvElements(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps);
    static cv::Mat renderStep2CvTypes(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps);
    static cv::Mat renderStep3FinalElements(const cv::Mat& base_img, const std::vector<FinalComponent>& comps);
    static cv::Mat renderStep3FinalTypes(const cv::Mat& base_img, const std::vector<FinalComponent>& comps);
    static cv::Mat renderStep3Geometric(const cv::Mat& base_img, const std::vector<FinalComponent>& comps);

    // =========================================================================
    // Parsing / Caricamento da file esistenti
    // =========================================================================
    static std::vector<ComponentDetection> loadDlDetectionsCsv(const std::string& csv_path);
    static std::vector<CvComponent> loadCvComponentsCsv(const std::string& csv_path);
    static std::map<std::string, std::vector<ComponentDetection>> loadAllDlDetectionsInDir(const std::string& dir);
    static float calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b);
    static float calculateOverlap(const cv::Rect2f& a, const cv::Rect2f& b);
    static std::string sourceToString(ComponentSource src);

private:
    static cv::Mat createSideLegendPanel(
        const std::vector<std::pair<std::string, int>>& label_counts,
        int total_count,
        const std::map<std::string, cv::Scalar>& palette,
        int target_height,
        const std::string& title,
        const std::vector<std::pair<std::string, int>>& extra_stats = {});
};

} // namespace pcb
