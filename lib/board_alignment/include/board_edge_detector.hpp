#pragma once

#include "gerber_parser.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace pcb {

/**
 * @brief Struttura che descrive un contorno o rettangolo del bordo PCB rilevato nell'immagine fotografica (in pixel).
 */
struct DetectedBoardEdge {
    bool valid = false;                     ///< True se il candidato è geometricamente valido
    std::vector<cv::Point2f> contour_px;    ///< Punti del contorno in coordinate pixel
    cv::RotatedRect rotated_rect_px;        ///< Rettangolo orientato minimo
    cv::Rect bounding_box_px;               ///< Bounding box allineato agli assi
    std::vector<cv::Point2f> corners_px;    ///< 4 angoli ordinati: TL (0), TR (1), BR (2), BL (3)
    double width_px = 0.0;                  ///< Dimensione maggiore in pixel
    double height_px = 0.0;                 ///< Dimensione minore in pixel
    double aspect_ratio = 1.0;              ///< Rapporto d'aspetto (dimensione_max / dimensione_min)
    double area_px = 0.0;                   ///< Area in pixel^2
    double solidity = 0.0;                  ///< Rapporto area contorno / area rettangolo orientato
    double score = 0.0;                     ///< Punteggio di confidenza geometrica e similarità con Gerber
    std::string method_tag;                 ///< Tag del metodo che ha generato il candidato

    // Margini di vuoto/sfondo rilevati rispetto ai confini del frame
    int margin_top = 0;
    int margin_bottom = 0;
    int margin_left = 0;
    int margin_right = 0;
};

/**
 * @brief Parametri di configurazione per il rilevatore del bordo della scheda.
 */
struct BoardEdgeConfig {
    double min_board_area_ratio = 0.15;     ///< Area minima del PCB rispetto all'intera immagine (15%)
    double max_board_area_ratio = 0.98;     ///< Area massima rispetto all'immagine (esclude frame completo)
    double min_solidity = 0.65;             ///< Solidità minima (area_contorno / area_minAreaRect)
    int morph_close_size = 35;              ///< Dimensione kernel chiusura morfologica
    int canny_thresh1 = 30;                 ///< Soglia bassa Canny
    int canny_thresh2 = 90;                 ///< Soglia alta Canny
    bool use_mertens_fusion = true;         ///< Abilita fusione Mertens HDR multi-esposizione
};

/**
 * @brief Classe per il rilevamento del bordo fisico della scheda elettronica nelle foto,
 * combinando esposizioni multiple per estrarre il perimetro migliore e più nitido.
 */
class BoardEdgeDetector {
public:
    explicit BoardEdgeDetector(const BoardEdgeConfig& config = BoardEdgeConfig());

    /**
     * @brief Fonde le immagini prese dalla stessa posizione con diverse esposizioni luminose,
     * generando un'immagine ad alta dinamica e contrasto ottimale.
     */
    cv::Mat combineMultiExposure(const std::vector<cv::Mat>& images) const;

    /**
     * @brief Rileva tutti i contorni/rettangoli candidati che rappresentano il perimetro del PCB,
     * sfruttando l'unione multi-esposizione, gradienti massimi congiunti e maschera substrato verdognolo.
     */
    std::vector<DetectedBoardEdge> detectCandidates(const std::vector<cv::Mat>& images) const;

    /**
     * @brief Seleziona il contorno candidato che si avvicina di più al contorno del disegno Gerber.
     */
    DetectedBoardEdge selectBestCandidate(
        const std::vector<DetectedBoardEdge>& candidates,
        const GerberBoardOutline& gerberOutline) const;

    /**
     * @brief Ordina i 4 angoli di un rettangolo in ordine orario partendo da top-left:
     * 0 = Top-Left, 1 = Top-Right, 2 = Bottom-Right, 3 = Bottom-Left.
     */
    static std::vector<cv::Point2f> orderCorners(const std::vector<cv::Point2f>& pts);

    /**
     * @brief Genera un'immagine di diagnostica visiva che mostra tutti i bordi candidati rilevati
     * dal software e il bordo finale selezionato.
     */
    cv::Mat renderCandidatesVisualization(
        const cv::Mat& fusedImg,
        const std::vector<DetectedBoardEdge>& candidates,
        const DetectedBoardEdge& selected) const;

private:
    BoardEdgeConfig config_;

    cv::Mat computeSubstrateMask(const cv::Mat& fusedImg) const;
    cv::Mat unifyDisconnectedEdges(const cv::Mat& greenMask) const;
};

} // namespace pcb
