#pragma once

#include "gerber_parser.hpp"
#include "board_edge_detector.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace pcb {

/**
 * @brief Risultato dell'allineamento tra l'immagine fotografica (in pixel) e il disegno Gerber (in mm).
 */
struct AlignmentResult {
    bool success = false;
    std::string board_name;
    std::string side;

    GerberBoardOutline gerber_outline;      ///< Bordo e strato del Gerber (in mm)
    DetectedBoardEdge detected_edge;        ///< Bordo rilevato nelle foto (in pixel)

    double pixels_per_mm = 0.0;             ///< Scala: numero di pixel per ogni millimetro
    double mm_per_pixel = 0.0;              ///< Scala inversa: millimetri per pixel
    double rotation_angle_deg = 0.0;        ///< Orientamento angolare del PCB nell'immagine
    double aspect_ratio_error = 0.0;        ///< Errore relativo di aspect ratio (immagine vs gerber)

    cv::Mat homography_px_to_mm;            ///< Matrice di omografia 3x3: mappa punto pixel -> coordinate mm Gerber
    cv::Mat homography_mm_to_px;            ///< Matrice di omografia 3x3 inversa: mappa mm Gerber -> punto pixel
    cv::Mat affine_px_to_mm;                ///< Matrice di trasformazione affine 2x3: pixel -> mm
    cv::Mat affine_mm_to_px;                ///< Matrice di trasformazione affine 2x3 inversa: mm -> pixel

    cv::Rect crop_rect_px;                  ///< Bounding box del ritaglio in pixel

    /**
     * @brief Converte un punto dall'immagine in pixel a coordinate millimetriche del disegno Gerber.
     */
    cv::Point2f pixelToMillimeter(const cv::Point2f& ptPx) const;

    /**
     * @brief Converte un punto millimetrico del disegno Gerber in coordinate pixel dell'immagine.
     */
    cv::Point2f millimeterToPixel(const cv::Point2f& ptMm) const;
};

/**
 * @brief Modulo per l'allineamento tra il disegno Gerber e le immagini della scheda elettronica,
 * la generazione di immagini di visualizzazione e il ritaglio lungo il bordo con archiviazione delle originali in ok/.
 */
class BoardAlignmentProcessor {
public:
    BoardAlignmentProcessor();

    /**
     * @brief Calcola l'allineamento geometrico e l'associazione tra pixel e mm,
     * trovando il bordo migliore che si avvicina al disegno Gerber.
     */
    AlignmentResult computeAlignment(
        const std::vector<cv::Mat>& images,
        const GerberBoardOutline& gerberOutline,
        const std::string& boardName,
        const std::string& side) const;

    /**
     * @brief Genera l'immagine con sovraimpressione del riquadro del bordo, gli assi orientati,
     * i 4 vertici e le informazioni di allineamento (dimensioni px, mm e scala).
     */
    cv::Mat renderAlignmentOverlay(
        const cv::Mat& srcImage,
        const AlignmentResult& alignment,
        const std::string& imageName = "") const;

    /**
     * @brief Ritaglia l'immagine esattamente lungo il bordo trovato.
     * Se la scheda presenta un lieve disallineamento angolare, la raddrizza ortogonalmente.
     */
    cv::Mat cropAlongBoardEdge(
        const cv::Mat& srcImage,
        const AlignmentResult& alignment,
        bool straighten = true) const;

    /**
     * @brief Esegue l'elaborazione completa dell'allineamento per un lato di una scheda:
     * 1. Cerca il file Gerber nella cartella di input ed estrae lo strato con area massima (bordo scheda).
     * 2. Riconosce il bordo PCB nelle foto unendo le diverse esposizioni.
     * 3. Associa il bordo immagine al bordo Gerber e calcola la trasformazione pixel <-> mm.
     * 4. Salva le foto con il riquadro del bordo e l'allineamento in output.
     * 5. Ritaglia tutte le immagini lungo il bordo, salvando i ritagli in undistorted e spostando le originali in ok/.
     * 6. Salva i parametri di allineamento (.yml) in parameters.
     */
    bool processBoardSide(
        const std::string& boardName,
        const std::string& side,
        const std::string& undistortedDir,
        const std::string& gerberDir,
        const std::string& alignmentOutputDir,
        const std::string& paramsOutputDir,
        bool showWindows = false) const;

    /**
     * @brief Salva la trasformazione e i dati di calibrazione in formato YAML.
     */
    bool saveAlignmentParams(
        const std::string& filePath,
        const AlignmentResult& alignment) const;

private:
    BoardEdgeDetector edgeDetector_;
};

} // namespace pcb

