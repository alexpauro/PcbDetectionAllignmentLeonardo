#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <filesystem>

namespace pcb {
namespace common {

/**
 * @brief Sposta un file nella sottocartella "ok" all'interno della sua cartella sorgente.
 * 
 * @param sourcePath Percorso del file da spostare.
 * @return true se lo spostamento ha avuto successo, false altrimenti.
 */
bool moveToOkFolder(const std::filesystem::path& sourcePath);

/**
 * @brief Calcola l'Intersection over Union (IoU) tra due rettangoli.
 * 
 * @param a Primo rettangolo.
 * @param b Secondo rettangolo.
 * @return float Valore IoU compreso tra 0.0 e 1.0.
 */
float calculateIoU(const cv::Rect2f& a, const cv::Rect2f& b);

/**
 * @brief Calcola il rapporto di sovrapposizione asimmetrico: Area(A intersecato B) / min(Area(A), Area(B)).
 * 
 * @param a Primo rettangolo.
 * @param b Secondo rettangolo.
 * @return float Valore compreso tra 0.0 e 1.0.
 */
float calculateOverlap(const cv::Rect2f& a, const cv::Rect2f& b);

/**
 * @brief Restituisce l'elenco ufficiale delle 13 classi standard supportate dal sistema.
 * 
 * @return std::vector<std::string> Elenco dei nomi delle classi.
 */
std::vector<std::string> getDefaultClasses();

/**
 * @brief Restituisce il codice breve (abbreviazione) per un tipo di componente.
 * 
 * @param full_label Nome completo della classe.
 * @return std::string Sigla abbreviata (es. "R", "C", "IC").
 */
std::string getShortLabel(const std::string& full_label);

/**
 * @brief Restituisce la palette di colori deterministica per le etichette fornite.
 * 
 * @param labels Vettore di etichette di classe.
 * @return std::map<std::string, cv::Scalar> Mappa classe -> colore BGR.
 */
std::map<std::string, cv::Scalar> getPalette(const std::vector<std::string>& labels);

/**
 * @brief Genera una barra laterale a sfondo scuro con la legenda grafica delle classi e statistiche.
 * 
 * @param targetHeight Altezza dell'immagine canvas.
 * @param label_counts Vettore di coppie (etichetta, conteggio).
 * @param total_count Conteggio complessivo dei componenti.
 * @param palette Mappa dei colori per classe.
 * @param title Titolo dell'intestazione della legenda.
 * @param extra_stats Statistiche addizionali facoltative.
 * @param panelWidth Larghezza desiderata del pannello (default 440 px).
 * @return cv::Mat Pannello disegnato a 3 canali BGR.
 */
cv::Mat renderSideLegend(
    int targetHeight,
    const std::vector<std::pair<std::string, int>>& label_counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats = {},
    int panelWidth = 440);

/**
 * @brief Overload per legenda da std::map<std::string, int>.
 */
cv::Mat renderSideLegend(
    int targetHeight,
    const std::map<std::string, int>& counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats = {},
    int panelWidth = 360);

} // namespace common
} // namespace pcb

