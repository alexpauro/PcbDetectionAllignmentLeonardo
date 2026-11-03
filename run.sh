#!/bin/bash
set -o pipefail

# Directory principale del progetto
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$ROOT_DIR/logs"
MAX_LOGS=10

mkdir -p "$LOG_DIR"

# Gestione FIFO: conserva solo gli ultimi 10 file di log eliminando i più vecchi
rotate_fifo_logs() {
    local logs=($(ls -1t "$LOG_DIR"/run_*.txt 2>/dev/null))
    local count=${#logs[@]}
    if [ $count -ge $MAX_LOGS ]; then
        for ((i=$((MAX_LOGS-1)); i<$count; i++)); do
            rm -f "${logs[$i]}"
        done
    fi
}
rotate_fifo_logs

# Nome del file di log con timestamp per questa esecuzione (estensione .txt)
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOG_FILE="$LOG_DIR/run_${TIMESTAMP}.txt"

run_pipeline() {
    echo "======================================================"
    echo " Avvio run.sh: $(date +"%Y-%m-%d %H:%M:%S")"
    echo " Log file: logs/run_${TIMESTAMP}.txt"
    echo "======================================================"

    # Crea la cartella build se non esiste
    mkdir -p "$ROOT_DIR/build"
    cd "$ROOT_DIR/build"

    # Genera i file Make
    cmake ..

    # Compila sfruttando tutti i core disponibili (-j)
    make -j$(nproc)

    # Se la compilazione ha successo, esegue il programma
    if [ $? -eq 0 ]; then
        echo -e "\n=== ESECUZIONE ==="
        ./app/pcb_detection_alignment
        local exit_code=$?
        echo -e "\n=== PROGRAMMA TERMINATO (Codice: $exit_code) ==="
        return $exit_code
    else
        echo -e "\n=== ERRORE DI COMPILAZIONE ==="
        return 1
    fi
}

# Esegue la pipeline duplicando l'output sia a terminale sia nel file di log
run_pipeline 2>&1 | tee "$LOG_FILE"
EXIT_STATUS=${PIPESTATUS[0]}

# Aggiorna il collegamento simbolico all'ultimo log salvato (puntando al .txt)
ln -sf "run_${TIMESTAMP}.txt" "$LOG_DIR/latest.txt"

echo -e "\nOutput di esecuzione salvato in: $LOG_FILE"
echo "Totale log memorizzati (max $MAX_LOGS, FIFO): $(ls -1 "$LOG_DIR"/run_*.txt 2>/dev/null | wc -l)"

exit $EXIT_STATUS