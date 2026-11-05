#pragma once

#include <string>
#include <vector>
#include <map>
#include <utility>

namespace pcb {

/**
 * @brief Risultato dell'algoritmo di similarità e ricerca per parole chiave.
 */
struct ClassMatchResult {
    std::string parsa_class;        ///< Classe ParsaVictor stimata (o "?")
    float confidence = 0.0f;        ///< Livello di confidenza (0.0 .. 1.0)
    bool is_certain = false;        ///< True se il matching è considerato certo (>= 0.75f)
    std::string match_reason;       ///< Descrizione della regola applicata
};

/**
 * @brief Gestore dell'unificazione e tracciamento persistente delle classi
 * dei vari modelli verso le 13 classi di riferimento di ParsaVictor.
 */
class ClassMapper {
public:
    /**
     * @brief Costruttore con percorso al file TXT di tracciamento.
     */
    explicit ClassMapper(const std::string& mapping_file_path = "../data/parameters/class_mapping_weights.txt");
    ~ClassMapper() = default;

    /**
     * @brief Carica le associazioni dal file di testo specificato.
     * @param path Percorso al file TXT.
     * @return true se il caricamento ha successo.
     */
    bool loadMappingFile(const std::string& path);

    /**
     * @brief Salva le associazioni correnti nel file TXT.
     * @param path Percorso opzionale (se vuoto usa mapping_file_path_).
     * @return true se salvato con successo.
     */
    bool saveMappingFile(const std::string& path = "") const;

    /**
     * @brief Ottiene o determina la classe ParsaVictor associata a una classe originale.
     * 1. Verifica se l'associazione è già registrata per questo modello o globalmente.
     * 2. In caso contrario, applica l'algoritmo per parole chiave e similarità.
     * 3. Se l'algoritmo non è certo (confidence < 0.75) e allow_interactive è true,
     *    chiede all'utente tramite console a quale classe ParsaVictor associarla.
     * 4. Registra la nuova associazione nel file TXT.
     * 
     * @param model_name Nome identificativo del modello/pesi (es. "yolov5s", "custom_pcb").
     * @param orig_class Nome originale della classe nel modello.
     * @param allow_interactive Se true, consente l'interazione con l'utente in caso di incertezza.
     * @return std::string Nome della classe ParsaVictor unificata.
     */
    std::string getOrMapClass(const std::string& model_name,
                              const std::string& orig_class,
                              bool allow_interactive = true);

    /**
     * @brief Algoritmo di matching basato su parole chiave, dizionario sinonimi e token similarity.
     * @param raw_class Nome della classe da analizzare.
     * @return ClassMatchResult Struttura con classe stimata, punteggio e certezza.
     */
    ClassMatchResult matchClassByKeywords(const std::string& raw_class) const;

    /**
     * @brief Chiede all'utente via console la classe ParsaVictor corretta per una classe incerta.
     * @param model_name Nome del modello.
     * @param orig_class Nome della classe originale.
     * @param suggested_class Eventuale classe suggerita dall'algoritmo.
     * @return std::string Classe ParsaVictor scelta dall'utente.
     */
    std::string promptUserForClass(const std::string& model_name,
                                   const std::string& orig_class,
                                   const std::string& suggested_class = "") const;

    /**
     * @brief Restituisce l'elenco delle 13 classi di riferimento ufficiali di ParsaVictor.
     */
    static const std::vector<std::string>& getParsaVictorClasses();

    /**
     * @brief Verifica se una stringa corrisponde a una delle 13 classi ParsaVictor.
     */
    static bool isParsaVictorClass(const std::string& label);

    /**
     * @brief Aggiunge manualmente un'associazione e la salva nel file.
     */
    void addMapping(const std::string& model_name,
                    const std::string& orig_class,
                    const std::string& parsa_class);

    /**
     * @brief Restituisce il percorso corrente del file TXT.
     */
    const std::string& getMappingFilePath() const { return mapping_file_path_; }

private:
    std::string mapping_file_path_;
    // (modello_lower, classe_lower) -> parsa_class
    std::map<std::pair<std::string, std::string>, std::string> model_class_map_;
    // classe_lower -> parsa_class (regole globali)
    std::map<std::string, std::string> global_class_map_;

    static std::string normalizeString(const std::string& str);
    void appendMappingToFile(const std::string& model_name,
                             const std::string& orig_class,
                             const std::string& parsa_class);
};

} // namespace pcb

