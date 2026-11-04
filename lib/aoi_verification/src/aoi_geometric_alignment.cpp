#include "aoi_internal.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace pcb {

cv::Point2f AoiAlignmentTransform::aoiToPixel(double x_mm, double y_mm) const {
    if (!valid) return cv::Point2f(0.0f, 0.0f);

    // 1. Modello di correzione distorsione ottica e prospettiva (Omografia 3x3)
    if (has_distortion_model && !homography_3x3.empty()) {
        std::vector<cv::Point2f> src = { cv::Point2f(static_cast<float>(x_mm), static_cast<float>(y_mm)) };
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, homography_3x3);
        if (!dst.empty()) {
            return dst[0];
        }
    }

    // 2. Modello lineare rigido base (fallback)
    double xr = mxx * x_mm + mxy * y_mm;
    double yr = myx * x_mm + myy * y_mm;

    if (std::abs(residual_angle_deg) > 1e-4) {
        double rad = residual_angle_deg * CV_PI / 180.0;
        double cos_r = std::cos(rad);
        double sin_r = std::sin(rad);
        double x_fine = cos_r * xr - sin_r * yr;
        double y_fine = sin_r * xr + cos_r * yr;
        xr = x_fine;
        yr = y_fine;
    }

    double px = scale_px_per_mm * xr + tx;
    double py = scale_px_per_mm * yr + ty;

    return cv::Point2f(static_cast<float>(px), static_cast<float>(py));
}

AoiAlignmentTransform AoiVerificationProcessor::findBestOriginAndDistortionAlignment(
    const std::vector<AoiComponent>& aoiComps,
    const std::vector<FinalComponent>& detections,
    double pixels_per_mm,
    double imgWidth,
    double imgHeight,
    double initial_tolerance_mm) const {

    AoiAlignmentTransform bestTransform;
    if (aoiComps.empty() || detections.empty()) {
        return bestTransform;
    }

    struct OrientationCandidate {
        std::string name;
        double mxx, mxy;
        double myx, myy;
    };

    std::vector<OrientationCandidate> orientations = {
        {"rot0",          1.0,  0.0,   0.0,  1.0},
        {"rot90",         0.0, -1.0,   1.0,  0.0},
        {"rot180",       -1.0,  0.0,   0.0, -1.0},
        {"rot270",        0.0,  1.0,  -1.0,  0.0},
        {"flip_x",       -1.0,  0.0,   0.0,  1.0},
        {"flip_y",        1.0,  0.0,   0.0, -1.0},
        {"flip_diag1",    0.0,  1.0,   1.0,  0.0},
        {"flip_diag2",    0.0, -1.0,  -1.0,  0.0}
    };

    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& a : aoiComps) {
        if (a.x_mm < minX) minX = a.x_mm;
        if (a.x_mm > maxX) maxX = a.x_mm;
        if (a.y_mm < minY) minY = a.y_mm;
        if (a.y_mm > maxY) maxY = a.y_mm;
    }
    double aoiCenterXMm = (minX + maxX) * 0.5;
    double aoiCenterYMm = (minY + maxY) * 0.5;

    double bestScore = -1.0;
    OrientationCandidate bestOri = orientations[0];
    double bestTx = 0.0, bestTy = 0.0;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> bestMatchedPairs;

    double match_tol_px = initial_tolerance_mm * pixels_per_mm;

    // FASE 1: Trova l'orientazione di massima verosimiglianza globale
    for (const auto& ori : orientations) {
        double rotCx = ori.mxx * aoiCenterXMm + ori.mxy * aoiCenterYMm;
        double rotCy = ori.myx * aoiCenterXMm + ori.myy * aoiCenterYMm;

        double tx = (imgWidth * 0.5) - (rotCx * pixels_per_mm);
        double ty = (imgHeight * 0.5) - (rotCy * pixels_per_mm);

        int inliers = 0;
        double errorSum = 0.0;
        std::vector<std::pair<cv::Point2f, cv::Point2f>> currentPairs;

        for (const auto& a : aoiComps) {
            double rx = ori.mxx * a.x_mm + ori.mxy * a.y_mm;
            double ry = ori.myx * a.x_mm + ori.myy * a.y_mm;

            double px = pixels_per_mm * rx + tx;
            double py = pixels_per_mm * ry + ty;

            double minDist = 1e9;
            cv::Point2f bestDetPt;
            for (const auto& d : detections) {
                double dist = std::hypot(d.center.x - px, d.center.y - py);
                if (dist < minDist) {
                    minDist = dist;
                    bestDetPt = d.center;
                }
            }

            if (minDist <= match_tol_px) {
                inliers++;
                errorSum += minDist;
                currentPairs.push_back({cv::Point2f(static_cast<float>(a.x_mm), static_cast<float>(a.y_mm)), bestDetPt});
            }
        }

        double score = inliers * 100.0 - (errorSum / (inliers > 0 ? inliers : 1));

        if (score > bestScore) {
            bestScore = score;
            bestOri = ori;
            bestTx = tx;
            bestTy = ty;
            bestMatchedPairs = currentPairs;
        }
    }

    std::cout << "[AOI] Migliore orientazione teorica: " << bestOri.name
              << " (Inlier iniziali: " << bestMatchedPairs.size() << "/" << aoiComps.size() << ")" << std::endl;

    bestTransform.valid = true;
    bestTransform.orientation_name = bestOri.name;
    bestTransform.mxx = bestOri.mxx;
    bestTransform.mxy = bestOri.mxy;
    bestTransform.myx = bestOri.myx;
    bestTransform.myy = bestOri.myy;
    bestTransform.tx = bestTx;
    bestTransform.ty = bestTy;
    bestTransform.scale_px_per_mm = pixels_per_mm;
    bestTransform.total_aoi_count = static_cast<int>(aoiComps.size());

    // FASE 2: Stima del modello di distorsione e prospettiva tramite Omografia RANSAC 3x3
    if (bestMatchedPairs.size() >= 8) {
        std::vector<cv::Point2f> srcPointsMm;
        std::vector<cv::Point2f> dstPointsPx;
        srcPointsMm.reserve(bestMatchedPairs.size());
        dstPointsPx.reserve(bestMatchedPairs.size());

        for (const auto& pair : bestMatchedPairs) {
            srcPointsMm.push_back(pair.first);
            dstPointsPx.push_back(pair.second);
        }

        cv::Mat inlierMask;
        cv::Mat H = cv::findHomography(srcPointsMm, dstPointsPx, cv::RANSAC, 5.5, inlierMask);

        if (!H.empty()) {
            bestTransform.has_distortion_model = true;
            bestTransform.homography_3x3 = H;
            cv::invert(H, bestTransform.homography_inv_3x3);

            // Ricalcoliamo inlier ed errore residuo finale con la correzione di distorsione
            int finalInliers = 0;
            double finalErrorPx = 0.0;
            double eval_tol_px = 3.0 * pixels_per_mm;

            for (const auto& a : aoiComps) {
                cv::Point2f proj = bestTransform.aoiToPixel(a.x_mm, a.y_mm);
                double minDist = 1e9;
                for (const auto& d : detections) {
                    double dist = std::hypot(d.center.x - proj.x, d.center.y - proj.y);
                    if (dist < minDist) minDist = dist;
                }

                if (minDist <= eval_tol_px) {
                    finalInliers++;
                    finalErrorPx += minDist;
                }
            }

            bestTransform.inlier_count = finalInliers;
            bestTransform.inlier_ratio = static_cast<double>(finalInliers) / aoiComps.size();
            bestTransform.avg_error_px = finalErrorPx / (finalInliers > 0 ? finalInliers : 1);
            bestTransform.avg_error_mm = bestTransform.avg_error_px / pixels_per_mm;

            std::cout << "[AOI] Modello di correzione distorsione ottica e prospettica stimato con successo!" << std::endl;
            std::cout << "  - Corrispondenza inlier: " << finalInliers << "/" << aoiComps.size()
                      << " (" << std::fixed << std::setprecision(1) << (bestTransform.inlier_ratio * 100.0) << "%)" << std::endl;
            std::cout << "  - Errore medio di allineamento: " << std::fixed << std::setprecision(2)
                      << bestTransform.avg_error_px << " px (" << bestTransform.avg_error_mm << " mm)" << std::endl;

            return bestTransform;
        }
    }

    // Fallback: senza modello di omografia 3x3
    bestTransform.has_distortion_model = false;
    bestTransform.inlier_count = static_cast<int>(bestMatchedPairs.size());
    bestTransform.inlier_ratio = static_cast<double>(bestMatchedPairs.size()) / aoiComps.size();
    bestTransform.avg_error_px = match_tol_px * 0.5;
    bestTransform.avg_error_mm = (match_tol_px * 0.5) / pixels_per_mm;

    return bestTransform;
}

} // namespace pcb

