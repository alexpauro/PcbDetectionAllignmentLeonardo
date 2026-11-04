#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <filesystem>
#include "output_processor.hpp"
#include "cv_component_detector.hpp"
#include "gerber_parser.hpp"

namespace pcb {

namespace fs = std::filesystem;

/**
 * @brief Stato del matching del singolo componente dopo la verifica con AOI.
 */
enum class AoiMatchStatus {
    MATCH_EXACT_CLASS,      ///< Componente reale presente, con classe identica al file AOI
    CLASS_CORRECTED,        ///< Componente reale presente, ma classe errata o sconosciuta corretta da AOI
    RECOVERED_BY_AOI,       ///< Componente non rilevato in precedenza, recuperato dalle coordinate AOI
    UNMATCHED_DETECTION     ///< Componente reale rilevato non presente nel file teorico AOI (non rappresentato nei render)
};

/**
 * @brief Rappresentazione di un singolo componente teorico letto dal file AOI (.asc).
 */
struct AoiComponent {
    std::string ref_des;             ///< Reference designator (es. "IC204", "R301", "C210")
    double x_mm = 0.0;               ///< Coordinata nominale X (mm)
    double y_mm = 0.0;               ///< Coordinata nominale Y (mm)
    double rot_deg = 0.0;            ///< Angolo di orientamento nominale Q (gradi)
    std::string side = "T";          ///< Lato: "T" (Top) o "B" (Bottom)
    std::string part_number;         ///< Codice componente interno (es. "4001794M0015")
    std::string outline;             ///< Footprint / Package (es. "SOT553", "RH0603B", "D2PAK")
    std::string description;         ///< Descrizione testuale (es. "RES 10kohm...", "OP AMP...")
    std::string raw_line;            ///< Riga originale completa del file .asc
    std::string assigned_class;      ///< Classe associata (es. "Resistor", "Capacitor", "IC", ...)

    // Campi calcolati dopo l'allineamento geometrico e modellazione distorsione
    cv::Point2f nominal_px;          ///< Posizione teorica mappata in pixel sull'immagine PCB
    cv::RotatedRect nominal_box_px;  ///< Bounding box orientato nominale in pixel (dalle piazzole Gerber)
    double pad_box_w_mm = 0.0;       ///< Larghezza ingombro piazzole Gerber (mm)
    double pad_box_h_mm = 0.0;       ///< Altezza ingombro piazzole Gerber (mm)
    bool matched = false;            ///< True se è stato abbinato a un componente rilevato
    int matched_detection_id = -1;   ///< ID del componente rilevato abbinato
    double match_distance_px = 0.0;  ///< Distanza dal centro reale (pixel)
    double match_distance_mm = 0.0;  ///< Distanza dal centro reale (mm)
};

/**
 * @brief Trasformazione geometrica ottima per l'allineamento tra coordinate AOI (mm) e immagine PCB (px),
 * con supporto alla correzione di distorsione e prospettiva ottica.
 */
struct AoiAlignmentTransform {
    bool valid = false;
    std::string orientation_name;    ///< Es. "rot180", "rot0", "flip_x", ...
    double mxx = 1.0, mxy = 0.0;     ///< Matrice 2x2 rotazione/riflessione
    double myx = 0.0, myy = 1.0;
    double tx = 0.0, ty = 0.0;       ///< Traslazione origine in pixel
    double scale_px_per_mm = 1.0;    ///< Scala pixel per millimetro (da board_alignment)
    double residual_angle_deg = 0.0; ///< Piccola rotazione fine residua stimata (gradi)

    // Modello di correzione distorsione ottica e prospettica (Omografia proiettiva 3x3)
    bool has_distortion_model = false;
    cv::Mat homography_3x3;          ///< Matrice 3x3 per correzione distorsione ottica (mm -> px)
    cv::Mat homography_inv_3x3;      ///< Inversa (px -> mm)

    int inlier_count = 0;            ///< Numero di componenti AOI coincidenti con rilevamenti reali
    int total_aoi_count = 0;         ///< Numero totale componenti teorici AOI
    double inlier_ratio = 0.0;       ///< Percentuale inlier
    double avg_error_px = 0.0;       ///< Errore medio di allineamento (pixel)
    double avg_error_mm = 0.0;       ///< Errore medio di allineamento (millimetri)

    /**
     * @brief Converte coordinate teoriche AOI (mm) in coordinate pixel dell'immagine PCB,
     * tenendo conto della distorsione ottica della lente e della prospettiva.
     */
    cv::Point2f aoiToPixel(double x_mm, double y_mm) const;

    /**
     * @brief Converte un punto pixel dell'immagine nelle coordinate millimetriche del file AOI.
     */
    cv::Point2f pixelToAoi(const cv::Point2f& ptPx) const;
};

/**
 * @brief Componente verificato finale dopo il confronto incrociato con i dati AOI e le piazzole Gerber.
 */
struct VerifiedPcbComponent {
    int id = 0;                                          ///< ID progressivo (1-based)
    std::string ref_des;                                 ///< Reference designator (es. "R301") o "N/A" se extra
    std::string label;                                   ///< Classe verificata finale
    std::string original_label;                          ///< Classe originale pre-verifica AOI
    float confidence = 0.0f;                             ///< Confidenza finale
    AoiMatchStatus status = AoiMatchStatus::MATCH_EXACT_CLASS; ///< Stato della verifica

    cv::Point2f center;                                  ///< Centro del componente in pixel (reale o rifinito)
    cv::Rect2f box;                                      ///< Bounding box allineato agli assi
    cv::RotatedRect rotated_box;                         ///< Bounding box orientato aderente (ricostruito da piazzole Gerber)
    ShapeType shape = ShapeType::RECTANGLE;              ///< Forma (RECTANGLE o CIRCLE)
    float radius = 0.0f;                                 ///< Raggio (se CIRCLE)
    float area = 0.0f;                                   ///< Area in pixel quadrati

    // Informazioni nominali da AOI e Gerber
    std::string aoi_part_number;                         ///< Codice componente AOI
    std::string aoi_outline;                             ///< Package/Outline AOI
    std::string aoi_description;                         ///< Descrizione testuale AOI
    double nominal_width_mm = 0.0;                       ///< Larghezza nominale dalle piazzole Gerber (mm)
    double nominal_height_mm = 0.0;                      ///< Altezza nominale dalle piazzole Gerber (mm)
    cv::Point2f nominal_center_px;                       ///< Centro nominale teorico proiettato con distorsione
    double offset_error_px = 0.0;                        ///< Scostamento reale vs nominale (pixel)
    double offset_error_mm = 0.0;                        ///< Scostamento reale vs nominale (millimetri)

    std::string source_name;                             ///< Origine (DL_ONLY, CV_ONLY, FUSED, RECOVERED_AOI, ecc.)
    std::string notes;                                   ///< Note esplicative
};

/**
 * @brief Statistiche complessive di verifica AOI.
 */
struct AoiVerificationSummary {
    int total_aoi_nominal = 0;                           ///< Componenti teorici nel file AOI
    int total_detections_input = 0;                      ///< Componenti rilevati in ingresso da DL/CV
    int total_verified_output = 0;                       ///< Componenti totali verificati in uscita

    int exact_matches = 0;                               ///< Componenti rilevati con classe già esatta
    int class_corrected = 0;                             ///< Componenti la cui classe è stata corretta con AOI
    int recovered_by_aoi = 0;                            ///< Componenti mancanti recuperati tramite AOI
    int unmatched_extra_detections = 0;                  ///< Rilevamenti reali senza corrispondenza AOI (non rappresentati)

    double mean_offset_error_px = 0.0;                   ///< Errore medio di posizionamento reale-nominale (px)
    double mean_offset_error_mm = 0.0;                   ///< Errore medio di posizionamento reale-nominale (mm)
    std::map<std::string, int> class_distribution;       ///< Distribuzione finale per ciascuna classe
};

/**
 * @brief Risultato completo dell'elaborazione di verifica AOI.
 */
struct AoiVerificationResult {
    bool success = false;
    std::string board_name;
    std::string side;

    std::vector<AoiComponent> aoi_components;
    std::vector<VerifiedPcbComponent> verified_components;
    AoiAlignmentTransform transform;
    AoiVerificationSummary summary;

    cv::Mat best_alignment_vis;                          ///< Immagine con la migliore corrispondenza coordinate-foto
    cv::Mat verified_types_vis;                          ///< Immagine con riquadri colorati per classe e RefDes (no extra)
    cv::Mat verified_elements_vis;                       ///< Immagine con riquadri colorati per stato di verifica (no extra)
};

/**
 * @brief Modulo per la verifica, allineamento coordinate AOI e correzione di classificazione dei componenti PCB.
 */
class AoiVerificationProcessor {
public:
    AoiVerificationProcessor();

    /**
     * @brief Restituisce l'elenco delle 13 classi standard supportate dal sistema.
     */
    static std::vector<std::string> getSupportedClasses();

    /**
     * @brief Carica il file delle associazioni classe <-> AOI (component_cv_mapping.txt).
     */
    bool loadMappingFile(const std::string& mappingFilePath);

    /**
     * @brief Salva una nuova associazione in coda al file component_cv_mapping.txt.
     */
    bool saveMappingEntry(const std::string& mappingFilePath, const std::string& rawLine, const std::string& assignedClass);

    /**
     * @brief Tenta di inferire automaticamente la classe corretta da RefDes, package e descrizione.
     */
    std::string inferClassFromKeywords(const std::string& ref,
                                       const std::string& outline,
                                       const std::string& description,
                                       bool& isConfident) const;

    /**
     * @brief Chiede interattivamente all'utente da terminale a quale classe associare la riga AOI non riconosciuta.
     */
    std::string askUserForClass(const std::string& ref,
                                const std::string& partNumber,
                                const std::string& outline,
                                const std::string& description) const;

    /**
     * @brief Esegue il parsing completo del file AOI (.asc) associando ciascun componente alla sua classe.
     */
    std::vector<AoiComponent> parseAoiFile(const std::string& aoiFilePath,
                                          const std::string& mappingFilePath,
                                          bool interactive = true);

    /**
     * @brief Calcola le dimensioni fisiche dell'ingombro dei componenti a partire dalle piazzole di saldatura
     * del file Gerber (estratte tramite GerberParser). Raggruppa e armonizza per codice componente:
     * tutti i componenti con lo stesso part_number avranno rigorosamente le stesse dimensioni di riquadro!
     */
    static std::map<std::string, std::pair<double, double>> computePartNumberDimensionsFromGerber(
        const std::vector<AoiComponent>& aoiComps,
        const std::vector<GerberPad>& pads);

    /**
     * @brief Trova la trasformazione geometrica di base per l'origine (R, tx, ty) e successivamente
     * stima il modello di correzione distorsione ottica (omografia proiettiva 3x3) per mappare esattamente
     * le coordinate su tutta l'estensione dell'immagine.
     */
    AoiAlignmentTransform findBestOriginAndDistortionAlignment(
        const std::vector<AoiComponent>& aoiComps,
        const std::vector<FinalComponent>& detections,
        double pixels_per_mm,
        double imgWidth,
        double imgHeight,
        double initial_tolerance_mm = 3.5) const;

    /**
     * @brief Carica e sovrappone tutti i componenti geometrici rilevati da Computer Vision
     * attraverso tutte le foto multi-esposizione del lato indicato.
     */
    static std::vector<CvComponent> loadAllCvDetectionsForBoardSide(
        const std::string& cvOutputDir,
        const std::string& boardName,
        const std::string& side);

    /**
     * @brief Risolve eventuali sovrapposizioni tra i componenti verificati.
     * I componenti rilevati da Deep Learning (DL) hanno priorità assoluta ("DL non sbaglia mai").
     * Se riquadri provenienti da CV o recuperati si sovrappongono a componenti DL, vengono ridimensionati.
     */
    static void resolveOverlapsWithDl(std::vector<VerifiedPcbComponent>& verifiedComps);

    /**
     * @brief Verifica i componenti rilevati, corregge le classi errate e recupera i componenti non rilevati,
     * utilizzando i riquadri trovati in DL per i componenti riconosciuti e la sovrapposizione multi-foto CV
     * per recuperare i componenti non riconosciuti, garantendo l'assenza di sovrapposizioni.
     */
    AoiVerificationResult verifyAndClassify(
        const cv::Mat& pcbImage,
        const std::vector<AoiComponent>& aoiComps,
        const std::vector<FinalComponent>& detections,
        const std::vector<CvComponent>& allCvSuperposition,
        const AoiAlignmentTransform& transform,
        double tolerance_mm = 3.0) const;

    /**
     * @brief Overload di compatibilità per verifyAndClassify.
     */
    AoiVerificationResult verifyAndClassify(
        const cv::Mat& pcbImage,
        const std::vector<AoiComponent>& aoiComps,
        const std::vector<FinalComponent>& detections,
        const std::vector<GerberPad>& gerberPads,
        const AoiAlignmentTransform& transform,
        double tolerance_mm = 3.0) const;

    /**
     * @brief Genera l'immagine con sovraimpressione della corrispondenza migliore tra coordinate teoriche e immagine.
     * (I componenti extra non vengono disegnati).
     */
    cv::Mat renderBestAlignmentOverlay(
        const cv::Mat& pcbImage,
        const std::vector<AoiComponent>& aoiComps,
        const std::vector<FinalComponent>& detections,
        const AoiAlignmentTransform& transform) const;

    /**
     * @brief Genera l'immagine annotata finale con classi verificate, Ref Designator e leggenda laterale.
     * (I componenti extra non vengono disegnati).
     */
    cv::Mat renderVerifiedTypesOverlay(
        const cv::Mat& pcbImage,
        const std::vector<VerifiedPcbComponent>& verifiedComps) const;

    /**
     * @brief Genera l'immagine annotata con codifica a colori per lo stato di verifica (Esatto, Corretto, Recuperato).
     * (I componenti extra non vengono disegnati).
     */
    cv::Mat renderVerifiedElementsOverlay(
        const cv::Mat& pcbImage,
        const std::vector<VerifiedPcbComponent>& verifiedComps) const;

    /**
     * @brief Salva tutti i risultati (CSV dettagliato, JSON di riepilogo, parametri YAML, immagini annotate).
     */
    bool saveResults(
        const std::string& outputDir,
        const std::string& paramsDir,
        const std::string& baseStem,
        const AoiVerificationResult& result) const;

    /**
     * @brief Esegue la pipeline completa di verifica AOI per un determinato lato di una scheda.
     */
    bool processBoardSide(
        const std::string& boardName,
        const std::string& side,
        const std::string& undistortedDir,
        const std::string& processedOutputDir,
        const std::string& aoiBaseDir,
        const std::string& gerberBaseDir,
        const std::string& paramsDir,
        const std::string& verificationOutputDir,
        bool showWindows = false,
        const std::string& preferredBaseImage = "",
        const std::string& cvDetectionOutputDir = "");

private:
    std::map<std::string, std::string> mapping_table_;
    std::vector<std::string> mapping_lines_raw_;

    static void getNominalOutlineDimensions(const std::string& outline, double& width_mm, double& height_mm);

    static cv::Mat createSideLegend(
        int targetHeight,
        const std::map<std::string, int>& counts,
        int totalComps,
        const std::map<std::string, cv::Scalar>& palette,
        const std::string& title,
        const std::vector<std::pair<std::string, int>>& extraStats);
};

} // namespace pcb
