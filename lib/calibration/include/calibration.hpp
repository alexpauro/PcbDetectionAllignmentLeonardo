#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

/**
 * @brief Sposta un file nella sottocartella "ok" all'interno della sua cartella sorgente.
 * 
 * @param sourcePath Percorso del file da spostare.
 * @return true se lo spostamento ha avuto successo, false altrimenti.
 */
bool moveToOkFolder(const std::filesystem::path& sourcePath);

/**
 * @brief Calibra la telecamera utilizzando un pattern ChArUco in modo iterativo.
 * 
 * @param folderPath Percorso della cartella contenente le immagini di calibrazione.
 * @param extension Estensione dei file immagine (es. ".bmp").
 * @param squaresX Numero di quadrati lungo l'asse X della scacchiera.
 * @param squaresY Numero di quadrati lungo l'asse Y della scacchiera.
 * @param squareLength Lato dei quadrati (es. in metri o millimetri).
 * @param markerLength Lato dei marker ArUco.
 * @param outputFilename Percorso e nome del file YAML in cui salvare i parametri intrinseci.
 * @param targetRms Soglia RMS desiderata per terminare l'ottimizzazione.
 * @param dictType Tipo di dizionario ArUco predefinito (es. cv::aruco::DICT_6X6_250).
 * @param forceCalibration Se true, ricalibra anche se il file di output esiste già.
 * @return true se la calibrazione è riuscita ed è stata salvata, false altrimenti.
 */
bool calibrateCameraSet(const std::string& folderPath, const std::string& extension,
                        int squaresX, int squaresY, float squareLength, float markerLength,
                        const std::string& outputFilename, double targetRms,
                        cv::aruco::PredefinedDictionaryType dictType,
                        bool forceCalibration = false);

#endif // CALIBRATION_HPP