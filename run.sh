#!/bin/bash
# Crea la cartella build se non esiste
mkdir -p build
cd build

# Genera i file Make
cmake ..

# Compila sfruttando tutti i core disponibili (-j)
make -j$(nproc)

# Se la compilazione ha successo, esegue il programma
if [ $? -eq 0 ]; then
    echo -e "\n=== ESECUZIONE ==="
    ./app/pcb_detection_alignment
else
    echo -e "\n=== ERRORE DI COMPILAZIONE ==="
fi