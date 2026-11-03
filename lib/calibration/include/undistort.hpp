#ifndef UNDISTORT_HPP
#define UNDISTORT_HPP

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Apre una finestra interattiva "Prima e Dopo" per consentire all'utente di regolare
 *        manualmente da tastiera la luminosità e il contrasto dell'immagine raddrizzata
 *        (con illuminazione preventivamente uniformata per eliminare il flash centrale).
 * 
 * @param undistorted Immagine raddrizzata originale.
 * @param imageName Nome identificativo dell'immagine visualizzato nella finestra e a console.
 * @param brightness Valore iniziale e parametro di ritorno della luminosità scelta (default: 128.0).
 * @param contrast Valore iniziale e parametro di ritorno del contrasto scelto (default: 2.5).
 * @return cv::Mat Immagine finale modificata e confermata dall'utente.
 */
cv::Mat adjustLightingAndContrastInteractive(const cv::Mat& undistorted, 
                                             const std::string& imageName,
                                             double& brightness, 
                                             double& contrast);

/**
 * @brief Uniforma completamente la luminosità su tutta la superficie della scheda PCB,
 *        azzerando il dislivello di luce (flash centrale e ombre periferiche) e consentendo
 *        di impostare un livello di luminosità omogeneo e un valore di contrasto personalizzati.
 * 
 * @param src Immagine sorgente (BGR a 3 canali o scala di grigi a 1 canale).
 * @param targetBrightness Livello di luminosità uniforme per tutta la scheda (default: 128.0).
 * @param contrast Intensità del contrasto per risaltare piste, pad e dettagli (default: 2.5).
 * @param sigma Deviazione standard per la stima del gradiente di luce a bassa frequenza (default: 40.0).
 * @param tileGridSize Dimensione delle celle per CLAHE (default: 8x8).
 * @return cv::Mat Immagine con luminosità uniformata e contrasto regolato.
 */
cv::Mat uniformLightingAndContrast(const cv::Mat& src, 
                                   double targetBrightness = 128.0, 
                                   double contrast = 2.5, 
                                   double sigma = 80.0, 
                                   cv::Size tileGridSize = cv::Size(8, 8));

/**
 * @brief Esegue il raddrizzamento (undistort) delle immagini acquisite dalla telecamera Robot.
 *        Dopo il raddrizzamento, uniforma la luminosità, apre una finestra con anteprima
 *        "Prima e Dopo" per la regolazione manuale di luminosità e contrasto da tastiera,
 *        stampa a terminale i parametri scelti e salva l'immagine finale.
 * 
 * @param inputFolder Cartella contenente le immagini originali (.bmp) da raddrizzare.
 * @param extension Estensione dei file immagine (es. ".bmp").
 * @param paramsFile Percorso del file YAML contenente i parametri di calibrazione.
 * @param outputFolder Cartella di destinazione in cui salvare le immagini raddrizzate.
 * @param boardName Nome identificativo della scheda PCB (es. "AA900789820001").
 * @param cameraName Nome/identificativo della telecamera (default "robot").
 * @param interactive Se true, attiva la finestra interattiva con visualizzazione prima/dopo (default: true).
 */
void undistortImages(const std::string& inputFolder, const std::string& extension, 
                     const std::string& paramsFile, const std::string& outputFolder,
                     const std::string& boardName, const std::string& cameraName = "robot",
                     bool interactive = true);

#endif // UNDISTORT_HPP
