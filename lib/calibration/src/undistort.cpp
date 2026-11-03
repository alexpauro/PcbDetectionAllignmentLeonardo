#include "undistort.hpp"
#include "calibration.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <algorithm>
#include <cmath>

namespace {

// Funzione ausiliaria interna per livellare l'illuminazione ed attenuare fortemente i picchi/flash
cv::Mat flattenLightingAndPeaks(const cv::Mat& lChannel, double sigma = 80.0) {
    const double scale = 0.25;
    cv::Mat smallL, smallIllum, illum;
    cv::resize(lChannel, smallL, cv::Size(), scale, scale, cv::INTER_AREA);
    double scaledSigma = std::max(1.0, sigma * scale);
    cv::GaussianBlur(smallL, smallIllum, cv::Size(0, 0), scaledSigma, scaledSigma);
    cv::resize(smallIllum, illum, lChannel.size(), 0, 0, cv::INTER_LINEAR);

    cv::Mat lFloat, illumFloat;
    lChannel.convertTo(lFloat, CV_32FC1);
    illum.convertTo(illumFloat, CV_32FC1);
    cv::max(illumFloat, 1.0f, illumFloat);

    float meanIllum = static_cast<float>(cv::mean(illumFloat)[0]);

    // 1. Divisione con attenuazione non lineare: dove c'è il flash centrale (illum > meanIllum),
    //    la divisione è accentuata (esponente 1.35) per schiacciare il picco di sovraesposizione
    cv::Mat ratio = illumFloat / meanIllum;
    cv::Mat ratioPowered;
    cv::pow(ratio, 1.35, ratioPowered);
    cv::max(ratioPowered, 0.1f, ratioPowered);

    cv::Mat flatL = (lFloat / ratioPowered);

    // 2. Compressione morbida (soft-knee) dei picchi residui per eliminare bruciature e riflessi speculari
    for (int r = 0; r < flatL.rows; ++r) {
        float* p = flatL.ptr<float>(r);
        for (int c = 0; c < flatL.cols; ++c) {
            if (p[c] > 190.0f) {
                p[c] = 190.0f + 40.0f * std::tanh((p[c] - 190.0f) / 35.0f);
            }
        }
    }

    cv::Mat flatL8u;
    flatL.convertTo(flatL8u, CV_8UC1);
    return flatL8u;
}

} // namespace

cv::Mat uniformLightingAndContrast(const cv::Mat& src, 
                                   double targetBrightness, 
                                   double contrast, 
                                   double sigma, 
                                   cv::Size tileGridSize) {
    if (src.empty()) {
        return cv::Mat();
    }

    bool isColor = (src.channels() == 3);
    cv::Mat lab;
    cv::Mat lChannel;
    std::vector<cv::Mat> labPlanes;

    if (isColor) {
        cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);
        cv::split(lab, labPlanes);
        lChannel = labPlanes[0];
    } else {
        lChannel = src.clone();
    }

    // 1. Uniformazione della luminosità e rimozione dei picchi
    cv::Mat uniformL8u = flattenLightingAndPeaks(lChannel, sigma);

    // 2. Regolazione del contrasto locale tramite CLAHE (range ampio fino a 16.0)
    cv::Mat contrastL;
    if (contrast > 0.0) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(contrast, tileGridSize);
        clahe->apply(uniformL8u, contrastL);
    } else {
        contrastL = uniformL8u;
    }

    // 3. Regolazione della luminosità target uniforme (range ampio)
    cv::Mat finalL;
    if (std::abs(targetBrightness - 128.0) > 0.5) {
        cv::Mat floatAdj;
        contrastL.convertTo(floatAdj, CV_32FC1);
        floatAdj += static_cast<float>(targetBrightness - 128.0);
        cv::min(cv::max(floatAdj, 0.0f), 255.0f, floatAdj);
        floatAdj.convertTo(finalL, CV_8UC1);
    } else {
        finalL = contrastL;
    }

    // 4. Ricomposizione dell'immagine finale
    cv::Mat dst;
    if (isColor) {
        labPlanes[0] = finalL;
        cv::merge(labPlanes, lab);
        cv::cvtColor(lab, dst, cv::COLOR_Lab2BGR);
    } else {
        dst = finalL;
    }

    return dst;
}

cv::Mat adjustLightingAndContrastInteractive(const cv::Mat& undistorted, 
                                             const std::string& imageName,
                                             double& brightness, 
                                             double& contrast) {
    if (undistorted.empty()) {
        return cv::Mat();
    }

    bool isColor = (undistorted.channels() == 3);
    cv::Mat lab;
    cv::Mat lChannel;
    std::vector<cv::Mat> labPlanes;

    if (isColor) {
        cv::cvtColor(undistorted, lab, cv::COLOR_BGR2Lab);
        cv::split(lab, labPlanes);
        lChannel = labPlanes[0];
    } else {
        lChannel = undistorted.clone();
    }

    // 1. Uniformazione preventiva e rimozione picchi (calcolata una volta sola per max fluidità)
    cv::Mat uniformL8u = flattenLightingAndPeaks(lChannel, 40.0);

    // Valori correnti (range estesi)
    double curBrightness = (brightness > 0.0) ? brightness : 128.0;
    double curContrast = (contrast > 0.0) ? contrast : 2.5;

    std::string winName = "Regolazione Luminosita' e Contrasto - " + imageName;
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::resizeWindow(winName, 1280, 540);

    std::cout << "\n====================================================================\n";
    std::cout << "FINESTRA REGOLAZIONE ATTIVA: " << imageName << "\n";
    std::cout << "Comandi (modificabili entrambi indipendentemente):\n";
    std::cout << "  [W] / [Freccia SU] / [+]       : Aumenta Luminosita' (+8)  [Range: 10 - 250]\n";
    std::cout << "  [S] / [Freccia GIU'] / [-]     : Diminuisce Luminosita' (-8) [Range: 10 - 250]\n";
    std::cout << "  [D] / [Freccia DESTRA]         : Aumenta Contrasto (+0.50) [Range: 0.2 - 16.0]\n";
    std::cout << "  [A] / [Freccia SINISTRA]       : Diminuisce Contrasto (-0.50)[Range: 0.2 - 16.0]\n";
    std::cout << "  [R]                            : Reset (Lum: 128, Cont: 2.5)\n";
    std::cout << "  [INVIO] o [SPAZIO]             : CONFERMA (OK) e procedi\n";
    std::cout << "====================================================================\n" << std::endl;

    cv::Mat resultImg;

    while (true) {
        // Applica contrasto CLAHE con valore corrente
        cv::Mat contrastL;
        if (curContrast > 0.0) {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(curContrast, cv::Size(8, 8));
            clahe->apply(uniformL8u, contrastL);
        } else {
            contrastL = uniformL8u;
        }

        // Applica luminosità con valore corrente
        cv::Mat finalL;
        if (std::abs(curBrightness - 128.0) > 0.5) {
            cv::Mat floatAdj;
            contrastL.convertTo(floatAdj, CV_32FC1);
            floatAdj += static_cast<float>(curBrightness - 128.0);
            cv::min(cv::max(floatAdj, 0.0f), 255.0f, floatAdj);
            floatAdj.convertTo(finalL, CV_8UC1);
        } else {
            finalL = contrastL;
        }

        // Ricomponi immagine finale a colori BGR
        if (isColor) {
            labPlanes[0] = finalL;
            cv::Mat mergedLab;
            cv::merge(labPlanes, mergedLab);
            cv::cvtColor(mergedLab, resultImg, cv::COLOR_Lab2BGR);
        } else {
            resultImg = finalL;
        }

        // Crea vista affiancata Prima (sinistra) e Dopo (destra) a metà risoluzione
        cv::Mat viewBefore, viewAfter;
        cv::resize(undistorted, viewBefore, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cv::resize(resultImg, viewAfter, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

        // Etichette grafiche a schermo
        cv::putText(viewBefore, "PRIMA (Originale)", cv::Point(20, 35), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        std::stringstream ssAfter;
        ssAfter << "DOPO [Lum: " << std::fixed << std::setprecision(1) << curBrightness 
                << " | Cont: " << curContrast << "]";
        cv::putText(viewAfter, ssAfter.str(), cv::Point(20, 35), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        cv::Mat combined;
        cv::hconcat(viewBefore, viewAfter, combined);

        // Barra comandi inferiore
        cv::Mat banner(45, combined.cols, combined.type(), cv::Scalar(30, 30, 30));
        std::string helpText = "W/S/SU/GIU: Lum (+-8) | A/D/SX/DX: Cont (+-0.5) | R: Reset | INVIO: OK";
        cv::putText(banner, helpText, cv::Point(20, 28), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(240, 240, 240), 1);

        cv::Mat displayView;
        cv::vconcat(combined, banner, displayView);

        cv::imshow(winName, displayView);

        int key = cv::waitKey(30);
        if (key != -1) {
            int ascii = key & 0xFF;
            bool changed = false;

            // Conferma
            if (ascii == 13 || ascii == 10 || ascii == 32) {
                break;
            }
            // Aumento Luminosità
            else if (ascii == 'w' || ascii == 'W' || key == 65362 || key == 0xff52 || ascii == '+' || ascii == '=') {
                curBrightness = std::min(250.0, curBrightness + 8.0);
                changed = true;
            }
            // Diminuzione Luminosità
            else if (ascii == 's' || ascii == 'S' || key == 65364 || key == 0xff54 || ascii == '-' || ascii == '_') {
                curBrightness = std::max(10.0, curBrightness - 8.0);
                changed = true;
            }
            // Aumento Contrasto
            else if (ascii == 'd' || ascii == 'D' || key == 65363 || key == 0xff53) {
                curContrast = std::min(16.0, curContrast + 0.50);
                changed = true;
            }
            // Diminuzione Contrasto
            else if (ascii == 'a' || ascii == 'A' || key == 65361 || key == 0xff51) {
                curContrast = std::max(0.20, curContrast - 0.50);
                changed = true;
            }
            // Reset
            else if (ascii == 'r' || ascii == 'R') {
                curBrightness = 128.0;
                curContrast = 2.5;
                changed = true;
            }

            if (changed) {
                std::cout << "\r>> Modifica attiva: Luminosita' = " << std::fixed << std::setprecision(1) 
                          << curBrightness << " | Contrasto = " << curContrast << "        " << std::flush;
            }
        }
    }

    cv::destroyWindow(winName);
    brightness = curBrightness;
    contrast = curContrast;

    std::cout << "\n\n======================================================\n";
    std::cout << "PARAMETRI CONFERMATI PER " << imageName << ":\n";
    std::cout << "  - Luminosita' target: " << brightness << "\n";
    std::cout << "  - Contrasto (CLAHE) : " << contrast << "\n";
    std::cout << "======================================================\n" << std::endl;

    return resultImg;
}

void undistortImages(const std::string& inputFolder, const std::string& extension, 
                     const std::string& paramsFile, const std::string& outputFolder,
                     const std::string& boardName, const std::string& cameraName,
                     bool interactive) {
    
    std::vector<cv::String> imageFiles;
    cv::glob(inputFolder + "/*" + extension, imageFiles);

    if (imageFiles.empty()) {
        std::cout << "\nNessuna immagine " << extension << " trovata in " << inputFolder << ". Salto la telecamera." << std::endl;
        return;
    }

    std::cout << "\n======================================================\n";
    std::cout << "INIZIO RADDRIZZAMENTO: " << inputFolder << "\n";
    std::cout << "======================================================\n";

    cv::FileStorage fs(paramsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Errore: impossibile aprire il file di calibrazione " << paramsFile << ". Esegui prima la calibrazione." << std::endl;
        return;
    }

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    std::error_code ec;
    std::filesystem::create_directories(outputFolder, ec);
    if (ec) {
        std::cerr << "Errore: impossibile creare la cartella di output " << outputFolder << ": " << ec.message() << std::endl;
        return;
    }

    for (size_t i = 0; i < imageFiles.size(); i++) {
        cv::Mat img = cv::imread(imageFiles[i]);
        if (img.empty()) {
            std::cerr << "Attenzione: impossibile leggere l'immagine " << imageFiles[i] << ", salto." << std::endl;
            continue;
        }

        cv::Mat undistorted;
        cv::undistort(img, undistorted, cameraMatrix, distCoeffs);

        std::stringstream ss;
        ss << outputFolder << "/" << boardName << "_" << cameraName << "_" 
           << std::setfill('0') << std::setw(3) << (i + 1) << extension;
        std::string outPath = ss.str();

        std::string displayName = boardName + "_" + cameraName + "_" + std::to_string(i + 1);

        cv::Mat processed;
        if (interactive) {
            double brightness = 128.0;
            double contrast = 2.5;
            processed = adjustLightingAndContrastInteractive(undistorted, displayName, brightness, contrast);
        } else {
            processed = uniformLightingAndContrast(undistorted, 128.0, 2.5);
        }

        if (!cv::imwrite(outPath, processed)) {
            std::cerr << "Errore: impossibile salvare l'immagine in "
                      << outPath << ". L'originale non verra' spostato." << std::endl;
            continue;
        }

        if (!moveToOkFolder(imageFiles[i])) {
            std::cerr << "Errore: immagine elaborata ma l'originale resta nella cartella "
                      << imageFiles[i] << std::endl;
            continue;
        }
        std::cout << "Elaborata e salvata con successo: " << outPath
                  << ". Originale spostato in ok/." << std::endl;
    }
}
