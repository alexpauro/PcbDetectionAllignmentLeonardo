#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace pcb {

/**
 * @brief Struttura che descrive un contorno o strato estratto dal file Gerber (in coordinate millimetriche).
 */
struct GerberContour {
    std::string layer_name;                 ///< Nome del layer o identificativo (es. %LN..., nome file, ecc.)
    std::string source_file;                ///< Nome o percorso del file sorgente
    std::vector<cv::Point2f> polygon_mm;    ///< Punti del perimetro/contorno in millimetri
    double area_mm2 = 0.0;                  ///< Area della superficie in mm^2
    cv::Rect2d bounding_box_mm;             ///< Bounding box orizzontale in mm
    cv::RotatedRect min_area_rect_mm;       ///< Rettangolo orientato minimo circoscritto in mm
    double width_mm = 0.0;                  ///< Dimensione maggiore in mm
    double height_mm = 0.0;                 ///< Dimensione minore in mm
    double aspect_ratio = 1.0;              ///< Rapporto d'aspetto (dimensione_max / dimensione_min)
};

/**
 * @brief Risultato del parsing Gerber con selezione automatica dello strato con area massima (bordo scheda).
 */
struct GerberBoardOutline {
    bool valid = false;                     ///< True se è stato individuato almeno un contorno/bordo valido
    GerberContour best_contour;             ///< Il contorno con area massima, che rappresenta il bordo della scheda
    std::vector<GerberContour> all_contours;///< Tutti gli strati/contorni rilevati nel file o nella cartella
    double width_mm = 0.0;                  ///< Larghezza del bordo scheda in mm
    double height_mm = 0.0;                 ///< Altezza del bordo scheda in mm
    double area_mm2 = 0.0;                  ///< Area totale del bordo scheda in mm^2
    double aspect_ratio = 1.0;              ///< Rapporto d'aspetto del bordo scheda
    std::string file_path;                  ///< File o cartella analizzata
};

/**
 * @brief Rappresentazione di una piazzola di saldatura (pad SMD / through-hole) estratta dal Gerber (in mm).
 */
struct GerberPad {
    cv::Point2f center_mm;                  ///< Posizione del centro della piazzola in millimetri
    double width_mm = 0.0;                  ///< Larghezza della piazzola in mm
    double height_mm = 0.0;                 ///< Altezza della piazzola in mm
    int dcode = 0;                          ///< DCode apertura associata
    std::string aperture_type;              ///< Tipo apertura (VB_RECTANGLE, C, ecc.)
};

/**
 * @brief Parser per file Gerber RS-274X standard.
 * Supporta unità in Pollici (%MOIN*%) e Millimetri (%MOMM*%), specificatori di formato coordinate (%FSLA...%),
 * comandi di disegno G01/G02/G03, aperture, blocchi contorno G36/G37, comandi layer %LN...% e piazzole di saldatura (D03).
 */
class GerberParser {
public:
    /**
     * @brief Esegue il parsing di un file Gerber o di una cartella contenente layer multipli.
     * Cerca tra tutti gli strati quello con area massima che rappresenta il bordo della scheda.
     * 
     * @param path Percorso del file Gerber (es. .gbr, .gko, .gm1) o della cartella con i vari strati.
     * @return GerberBoardOutline Contenente il bordo con area massima e tutti i contorni estratti.
     */
    static GerberBoardOutline parse(const std::string& path);

    /**
     * @brief Cerca e analizza il file o cartella Gerber per una specifica scheda all'interno della cartella base gerber.
     * Supporta:
     *   - <baseDir>/<boardName>/ (cartella con uno o più file layer)
     *   - <baseDir>/<boardName>.gbr (o estensioni affini)
     *   - <baseDir>/ (se contiene un unico file Gerber)
     * 
     * @param gerberBaseDir Cartella base di input dei file gerber (es. "data/inputs/pcb_gerber")
     * @param boardName Nome della scheda (es. "A", "B", ...)
     * @return GerberBoardOutline
     */
    static GerberBoardOutline findAndParseForBoard(const std::string& gerberBaseDir, const std::string& boardName);

private:
    static bool parseSingleFile(const std::string& filePath, std::vector<GerberContour>& outContours);
};

} // namespace pcb

