#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <filesystem>
#include "aoi_verification.hpp"

namespace pcb {

namespace fs = std::filesystem;

/**
 * @brief Risultato dell'allineamento preciso e modellazione metrologica di un singolo componente PCB.
 */
struct AlignedComponent {
    int id = 0;                                 ///< ID progressivo (1-based)
    std::string ref_des;                        ///< Reference designator / codice componente (es. "R301", "IC204")
    std::string label;                          ///< Tipologia / classe associata (es. "resistor", "ic", "capacitor_smd")
    std::string part_number;                    ///< Codice produttore / Part Number (es. "4001794M0015")
    std::string outline;                        ///< Package / Footprint CAD (es. "SOT553", "0603")

    // Contorno reale estratto via algoritmi di Computer Vision
    std::vector<cv::Point> contour;             ///< Punti del contorno reale in coordinate assolute immagine
    bool found = false;                         ///< True se allineato con successo
    int search_expansion_level = 0;             ///< 0: trovato in ROI base, 1: trovato in ROI espansa

    // Bounding box rettangolare aderente in pixel (solo rettangoli, senza spazi vuoti)
    cv::RotatedRect rotated_box;                ///< Rettangolo minimo orientato circoscritto
    cv::Rect2f bounding_box;                    ///< Bounding box allineato agli assi
    cv::Point2f pixel_center;                   ///< Centroide del componente in coordinate pixel
    float box_width_px = 0.0f;                  ///< Larghezza box in pixel
    float box_height_px = 0.0f;                 ///< Altezza box in pixel
    float pixel_angle_deg = 0.0f;               ///< Orientamento geometrico del box in pixel (gradi)
    float area_px = 0.0f;                       ///< Area del contorno in pixel quadrati

    // Dimensioni box in millimetri reali sulla scheda
    double box_width_mm = 0.0;                  ///< Larghezza box in mm
    double box_height_mm = 0.0;                 ///< Altezza box in mm

    // Dimensioni fisiche reali da datasheet (da component_cv_mapping.txt)
    double datasheet_width_mm = 0.0;            ///< Larghezza reale da datasheet (mm)
    double datasheet_height_mm = 0.0;           ///< Altezza reale da datasheet (mm)
    double datasheet_thickness_mm = 0.0;        ///< Spessore reale da datasheet (mm)
    std::string datasheet_source;               ///< Fonte/sito web del datasheet

    // Coordinate nominali teoriche (da distinta AOI / CAD)
    double nominal_x_mm = 0.0;                  ///< Posizione nominale X (mm)
    double nominal_y_mm = 0.0;                  ///< Posizione nominale Y (mm)
    double nominal_rot_deg = 0.0;               ///< Rotazione nominale CAD (gradi)
    cv::Point2f nominal_center_px;              ///< Centro nominale proiettato in pixel

    // Reale posizione del componente in mm (tenendo conto della distorsione ottica e prospettica)
    double real_x_mm = 0.0;                     ///< Coordinata reale X in mm (corretta con omografia inversa)
    double real_y_mm = 0.0;                     ///< Coordinata reale Y in mm (corretta con omografia inversa)
    double real_rot_deg = 0.0;                  ///< Orientamento reale in gradi nel sistema CAD

    // Errori di posizionamento e rotazione (rispetto alla posizione nominale)
    double error_x_mm = 0.0;                    ///< Errore lungo X: real_x_mm - nominal_x_mm
    double error_y_mm = 0.0;                    ///< Errore lungo Y: real_y_mm - nominal_y_mm
    double error_dist_mm = 0.0;                 ///< Errore euclideo di posizionamento in millimetri
    double error_dist_px = 0.0;                 ///< Errore euclideo di posizionamento in pixel
    double error_rot_deg = 0.0;                 ///< Errore angolare di rotazione (pochi decimi/gradi)

    std::string notes;                          ///< Note diagnostiche sul rilevamento o allineamento
};

/**
 * @brief Parametri di configurazione per la stima precisa di micro-traslazioni,
 * micro-rotazioni e allineamento rettangoli mediante fusione Caliper + Edge-Template multi-esposizione.
 */
struct ComponentAlignmentConfig {
    float max_translation_px = 4.0f;           ///< Escursione massima micro-traslazione in pixel (~0.35 mm)
    float max_rotation_deg = 3.0f;             ///< Escursione massima micro-rotazione in gradi (<= 3.0°)
    float translation_step_px = 0.5f;          ///< Passo di ricerca su griglia per la micro-traslazione (pixel)
    float rotation_step_deg = 0.25f;           ///< Passo di ricerca su griglia per la micro-rotazione (gradi)
    int caliper_rays_per_edge = 5;             ///< Numero di raggi caliper per ciascun bordo del rettangolo
    float caliper_ray_length_px = 5.0f;        ///< Semi-lunghezza del raggio di campionamento (pixel)
    bool fill_internal_holes = true;           ///< Riempimento fori interni
    float initial_margin_ratio = 0.25f;        ///< Margine iniziale intorno alla casella nota
    float expanded_margin_ratio = 0.70f;       ///< Margine espanso di ricerca
    float min_contour_area = 8.0f;             ///< Area minima del contorno in pixel
    float min_solidity = 0.40f;                ///< Solidita' minima
    float min_rect_fill = 0.35f;               ///< Riempimento minimo del rettangolo
    bool harmonize_part_number_sizes = true;   ///< Impone stesso ingombro in mm per componenti con stesso codice produttore
    int contour_thickness = 1;                 ///< Spessore dei bordi sottili colorati per il rendering (1 o 2 px)
    bool save_intermediate_filters = true;     ///< Salva le immagini dei filtri intermedi su disco
    bool show_windows = false;                 ///< Mostra finestre interattive OpenCV
};

/**
 * @brief Statistiche aggregate sul processo di allineamento e stima posa componenti.
 */
struct ComponentAlignmentSummary {
    int total_input_components = 0;             ///< Totale componenti riconosciuti in ingresso
    int successfully_aligned = 0;               ///< Componenti con micro-orientamento e posizione allineati
    int from_processed_output = 0;              ///< Componenti preservati da processed_output con etichette AOI
    int recovered_by_aoi = 0;                   ///< Nuove box recuperate da AOI non presenti in precedenza
    int preserved_original_box = 0;             ///< Componenti mantenuti con box originario validato
    int found_in_initial_roi = 0;               ///< Trovati con margine iniziale di tolleranza
    int found_in_expanded_roi = 0;              ///< Trovati espandendo il margine di tolleranza
    int rejected_not_found = 0;                 ///< Non confermati (0)
    int rejected_overlap_cataloged = 0;         ///< Duplicati scartati (0)
    
    double mean_error_dist_mm = 0.0;            ///< Errore medio di posizionamento (mm)
    double mean_error_dist_px = 0.0;            ///< Errore medio di posizionamento (px)
    double mean_error_rot_deg = 0.0;            ///< Errore medio di rotazione (gradi)

    std::map<std::string, cv::Size2f> canonical_part_sizes_px; ///< Ingombro medio pixel per codice produttore
    std::map<std::string, cv::Size2d> canonical_part_sizes_mm; ///< Ingombro millimetri unificato per codice produttore
    std::map<std::string, int> class_distribution;             ///< Distribuzione per classe
};

/**
 * @brief Risultato completo dell'elaborazione di allineamento componenti.
 */
struct ComponentAlignmentResult {
    bool success = false;
    std::string board_name;
    std::string side;
    std::string reference_image_name;

    std::vector<AlignedComponent> aligned_components;   ///< Componenti validati e allineati
    std::vector<AlignedComponent> rejected_components;  ///< Componenti scartati/eliminati
    ComponentAlignmentSummary summary;

    cv::Mat contours_vis;       ///< Immagine con indicazione del contorno reale di tutti i componenti (bordi sottili e colorati)
    cv::Mat tight_boxes_vis;    ///< Immagine con i rettangoli minimi orientati aderenti senza spazi
    cv::Mat cataloged_mask;     ///< Maschera binaria delle aree occupate dai componenti catalogati
    std::map<std::string, cv::Mat> intermediate_filter_masks; ///< Risultati intermedi con i diversi filtri
};

/**
 * @brief Classe principale per l'allineamento fine, stima micro-rotazioni e micro-traslazioni,
 * adattamento rettangoli circoscritti e verifica metrologica con correzione della distorsione.
 */
class ComponentAlignmentProcessor {
public:
    ComponentAlignmentProcessor();

    /**
     * @brief Esegue la pipeline di allineamento su una lista di componenti già riconosciuti
     * (es. provenienti da AOI verification o step finale di fusione).
     *
     * @param pcbImage Immagine PCB di riferimento non distorta.
     * @param inputComponents Vettore dei componenti riconosciuti in ingresso.
     * @param transform Trasformazione e modello di distorsione omografica AOI <-> pixel.
     * @param config Parametri di configurazione dell'allineamento.
     * @param multiExposureImages Vettore opzionale di 10 immagini multi-esposizione per la stima robusta.
     * @return ComponentAlignmentResult Risultato completo con contorni, box ed errori metrologici.
     */
    ComponentAlignmentResult process(
        const cv::Mat& pcbImage,
        const std::vector<VerifiedPcbComponent>& inputComponents,
        const AoiAlignmentTransform& transform,
        const ComponentAlignmentConfig& config = ComponentAlignmentConfig(),
        const std::vector<cv::Mat>& multiExposureImages = {});

    /**
     * @brief Esegue l'elaborazione ad alto livello per una scheda e lato, caricando
     * automaticamente i dati da disco (AOI verified components, parametri di allineamento e foto PCB).
     */
    ComponentAlignmentResult processBoardSide(
        const std::string& boardName,
        const std::string& side,
        const std::string& undistortedDir,
        const std::string& aoiVerificationDir,
        const std::string& paramsDir,
        const std::string& outputBaseDir,
        const ComponentAlignmentConfig& config = ComponentAlignmentConfig());

    /**
     * @brief Salva su disco i file prodotti dall'allineamento componenti:
     * - CSV con tutte le associazioni (RefDes, classe, codice produttore, coordinate px, reale posizione mm con distorsione, errori)
     * - JSON riepilogativo con statistiche e ingombri canonici
     * - Immagine con i contorni sottili e colorati
     * - Immagine con i rettangoli aderenti minimi
     */
    bool saveResults(
        const std::string& outputDir,
        const std::string& fileStem,
        const ComponentAlignmentResult& result);

    /**
     * @brief Mappa una coordinata pixel (u, v) nella reale coordinata millimetrica (x_mm, y_mm),
     * tenendo conto della distorsione della lente tramite l'inversa dell'omografia 3x3.
     */
    static cv::Point2d pixelToAoi(const cv::Point2f& pt_px, const AoiAlignmentTransform& transform);

    /**
     * @brief Mappa una coordinata millimetrica (x_mm, y_mm) in coordinate pixel (u, v).
     */
    static cv::Point2f aoiToPixel(const cv::Point2d& pt_mm, const AoiAlignmentTransform& transform);

    /**
     * @brief Calcola la differenza d'angolo minima (in gradi normalizzata tra -45 e +45 o -90 e +90),
     * tenendo conto delle simmetrie geometriche rettangolari e dell'orientamento nominale.
     */
    static double normalizeAngleDiff(double realAngleDeg, double nominalAngleDeg);
};

} // namespace pcb
