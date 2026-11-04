#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

namespace pcb {

/**
 * @brief Tipologia di forma geometrica approssimata per il componente.
 */
enum class ShapeType {
    RECTANGLE,  ///< Bounding box rettangolare (può essere orientato/ruotato)
    CIRCLE      ///< Cerchio circoscritto/approssimato
};

/**
 * @brief Informazioni geometriche di un singolo componente rilevato via Computer Vision classica.
 */
struct CvComponent {
    int id = 0;                         ///< ID progressivo del componente (1-based)
    ShapeType shape = ShapeType::RECTANGLE; ///< Forma geometrica ("RECTANGLE" o "CIRCLE")
    cv::Point2f center;                 ///< Coordinate del centro (x, y)
    cv::RotatedRect rotated_box;        ///< Rettangolo minimo orientato circoscritto (centro, w, h, angolo in gradi)
    float radius = 0.0f;                ///< Raggio effettivo/circoscritto (valido se shape == CIRCLE)
    float area = 0.0f;                  ///< Area del contorno in pixel quadrati
    float perimeter = 0.0f;             ///< Lunghezza del perimetro del contorno
    float circularity = 0.0f;           ///< Indice di circolarità: 4*pi*Area / (Perimetro^2) (1.0 = cerchio perfetto)
    float aspect_ratio = 1.0f;          ///< Rapporto d'aspetto: min(w, h) / max(w, h)
};

/**
 * @brief Configurazione dei filtri di Computer Vision classica per il rilevamento bordi componenti.
 */
struct CvDetectionConfig {
    // --- Filtro Colore Substrato PCB Verdognolo ---
    bool use_green_filter = true;       ///< Attiva il filtro basato sul colore verdognolo del PCB
    int h_min = 20;                     ///< Tonalita' minima HSV (tonalita' verde-oliva)
    int h_max = 90;                     ///< Tonalita' massima HSV (tonalita' verde/azzurrino)
    int s_min = 16;                     ///< Saturazione minima per substrato verde
    int s_max = 255;                    ///< Saturazione massima
    int v_min = 20;                     ///< Valore/Luminosita' minima per substrato
    int v_max = 250;                    ///< Valore/Luminosita' massima per substrato
    float green_ratio_thresh = -0.05f;  ///< Soglia indice di verdura BGR: (2*G - R - B) / (R + G + B)

    // --- Filtri per Bordi e Gradienti ---
    bool use_bilateral = true;          ///< Filtro bilaterale per ridurre rumore preservando i bordi
    int bilateral_d = 5;                ///< Diametro pixel vicinato per filtro bilaterale
    double bilateral_sigma_color = 35.0;///< Sigma spazio colore
    double bilateral_sigma_space = 35.0;///< Sigma spazio coordinate

    bool use_clahe = true;              ///< Contrasto adattivo locale CLAHE
    double clahe_clip_limit = 2.5;      ///< Limite soglia contrasto CLAHE
    int clahe_grid_size = 8;            ///< Dimensione griglia tessere CLAHE

    bool use_canny = true;              ///< Riconoscimento bordi Canny
    double canny_thresh1 = 35.0;        ///< Soglia inferiore Canny
    double canny_thresh2 = 110.0;       ///< Soglia superiore Canny
    bool auto_canny = true;             ///< Calcola automaticamente le soglie Canny in base alla mediana

    bool use_color_gradient = true;     ///< Rilevamento bordi cromatici multi-canale (anche con stessa luminanza)
    bool use_scharr_gradient = true;    ///< Operatore di Scharr per micro-bordi ad altissima precisione
    bool use_laplacian_sharp = true;    ///< Esaltazione dettagli ad alta frequenza (Unsharp Masking)

    bool use_morph_hats = true;         ///< Top-Hat (parti riflettenti/chiare) e Black-Hat (chip scuri)
    int hat_kernel_size = 5;            ///< Dimensione del kernel per Top-Hat e Black-Hat (es. 5x5)
    int hat_threshold = 18;             ///< Soglia di binarizzazione per Top-Hat / Black-Hat

    // --- Banco Filtri Colore per Componenti Multi-Colore ---
    bool use_multi_color_filters = true;///< Attiva il banco filtri colore differenziati per componenti
    bool detect_dark_chips = true;      ///< Chip neri IC, resistori SMD, diodi, SOIC/QFP (bassa luminosità)
    bool detect_silver_metal = true;    ///< Saldature a stagno, pin argentati lucidi, quarzi metallici
    bool detect_ceramic_capacitors = true; ///< Condensatori ceramici SMD (beige, marroncino, ocra, giallo)
    bool detect_gold_pads = true;       ///< Piazzole dorate/rame (ENIG, testpoint, connettori)
    bool detect_blue_components = true; ///< Componenti blu/azzurri (condensatori, resistenze di precisione)
    bool detect_red_components = true;  ///< Componenti rossi/arancio (condensatori al tantalio, diodi, LED)
    bool detect_white_components = true;///< Componenti ceramici bianchi e connettori chiari

    // --- Pulizia e Connessione Morfologica ---
    int open_kernel_size = 3;           ///< Kernel di apertura (Open): elimina piste di rame sottili (1-2 px)
    int close_kernel_size = 3;          ///< Kernel di chiusura (Close): raggruppa pin/saldature al corpo del componente
    bool filter_thin_traces = true;     ///< Rimuove piste di rame residue (rapporto d'aspetto allungato e spessore sottile)
    float max_trace_aspect_ratio = 6.0f;///< Rapporto lunghezza/larghezza massimo per identificare una pista (non componente)
    float max_trace_thickness = 3.5f;   ///< Spessore massimo in px per considerare un elemento come pista

    // --- Selezione Dimensioni Componenti ---
    float min_area = 12.0f;             ///< Area minima (px^2) per includere componenti piccolissimi (SMD 0402/0603)
    float max_area_ratio = 0.40f;       ///< Area massima (% dell'immagine) per escludere il telaio del PCB
    float min_dimension = 2.5f;         ///< Dimensione minima del lato corto per escludere rumore 1D

    // --- Classificazione Forma: Cerchio vs Rettangolo ---
    float circularity_thresh = 0.74f;   ///< Soglia circolarità per considerare un contorno come cerchio
    float circle_aspect_ratio_thresh = 0.68f; ///< Soglia minima rapporto d'aspetto per cerchio
    float circle_fill_thresh = 0.48f;   ///< Rapporto minimo riempimento area contorno / area cerchio

    // --- Visualizzazione e Finestre ---
    bool show_windows = true;           ///< Mostra a schermo le finestre con il risultato interattivo
    int wait_key_delay = 0;             ///< Delay per cv::waitKey (0 = attende pressione tasto utente)
    int window_width = 1280;            ///< Larghezza preferita della finestra ridimensionata
    int window_height = 960;            ///< Altezza preferita della finestra ridimensionata
};

/**
 * @brief Risultato del rilevamento componenti mediante Computer Vision classica.
 */
struct CvDetectionResult {
    std::vector<CvComponent> components; ///< Lista dei componenti individuati
    int num_rectangles = 0;              ///< Numero di componenti rettangolari (ruotati)
    int num_circles = 0;                 ///< Numero di componenti circolari

    cv::Mat annotated_image;             ///< Immagine con riquadri ruotati, cerchi e ID sovrimpressi
    cv::Mat green_substrate_mask;        ///< Maschera binaria del substrato verdognolo del PCB
    cv::Mat multi_color_mask;            ///< Maschera binaria dei componenti multi-colore (chip neri, ceramici, metallo, ecc.)
    cv::Mat gradient_mask;               ///< Maschera dei gradienti cromatici e micro-bordi Scharr/Canny
    cv::Mat edges_mask;                  ///< Maschera combinata di tutti i bordi
    cv::Mat components_mask;             ///< Maschera binaria pulita e chiusa dei componenti
};

/**
 * @brief Classe dedicata al riconoscimento di tutti i componenti su PCB mediante sola Computer Vision classica.
 *
 * Utilizza una combinazione di:
 * 1. Filtro di segmentazione del colore verdognolo tipico del substrato PCB (solder mask).
 * 2. Filtro bilaterale e contrasto CLAHE per esaltare i componenti mantenendo i bordi netti.
 * 3. Filtraggio avanzato dei bordi (Canny adattivo, gradiente morfologico, Top-Hat/Black-Hat).
 * 4. Morfologia matematica di apertura (per rimuovere piste sottili) e chiusura (per sigillare i componenti).
 * 5. Fit geometrico di precisione: approssimazione con rettangoli ruotati (RotatedRect) o cerchi (minEnclosingCircle),
 *    aderenti alle dimensioni reali del componente.
 * 6. Visualizzazione interattiva con finestre OpenCV e salvataggio automatico (immagini, maschere, CSV e JSON).
 */
class CvComponentDetector {
public:
    CvComponentDetector() = default;
    ~CvComponentDetector() = default;

    /**
     * @brief Esegue la pipeline di Computer Vision per rilevare tutti i componenti.
     * @param input_image Immagine BGR della scheda PCB (ad es. undistorted).
     * @param config Parametri di configurazione della pipeline e dei filtri.
     * @return CvDetectionResult Risultato con componenti, metriche e maschere.
     */
    CvDetectionResult detect(const cv::Mat& input_image,
                             const CvDetectionConfig& config = CvDetectionConfig());

    /**
     * @brief Mostra il risultato a video in una finestra ridimensionabile e gestisce l'input utente.
     * @param window_title Titolo della finestra.
     * @param result Risultato del rilevamento.
     * @param wait_ms Millisecondi di attesa (0 = blocca fino a pressione di un tasto).
     */
    void displayResult(const std::string& window_title,
                       const CvDetectionResult& result,
                       int wait_ms = 0);

    /**
     * @brief Salva su disco l'immagine annotata, le maschere dei filtri e i report CSV / JSON.
     * @param output_dir Cartella di destinazione.
     * @param image_stem Nome base dell'immagine (senza estensione).
     * @param original_image Immagine originale in ingresso.
     * @param result Risultato contenente i componenti e le maschere.
     * @param save_debug_masks Se true, salva anche le immagini delle maschere intermedie dei filtri.
     */
    void saveResults(const std::string& output_dir,
                     const std::string& image_stem,
                     const cv::Mat& original_image,
                     const CvDetectionResult& result,
                     bool save_debug_masks = true);

private:
    /**
     * @brief Disegna i componenti (rettangoli ruotati e cerchi aderenti) sull'immagine di output.
     */
    void drawComponents(cv::Mat& output, const std::vector<CvComponent>& components) const;

    /**
     * @brief Calcola le soglie Canny ottimali a partire dalla mediana dell'immagine in scala di grigi.
     */
    void computeAutoCannyThresholds(const cv::Mat& gray, double& lower, double& upper) const;
};

} // namespace pcb

