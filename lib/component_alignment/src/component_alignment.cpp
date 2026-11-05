#include "component_alignment.hpp"
#include "cv_component_detector.hpp"
#include "common_utils.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace pcb {

struct DatasheetEntry {
    double w_mm = 0.0;
    double h_mm = 0.0;
    double thick_mm = 0.0;
    std::string source;
};

static std::map<std::string, DatasheetEntry> loadDatasheetMapping(const std::string& mappingFilePath) {
    std::map<std::string, DatasheetEntry> table;
    std::ifstream file(mappingFilePath);
    if (!file.is_open()) return table;
    std::string line;
    while (std::getline(file, line)) {
        size_t arrowPos = line.find("--> Class:");
        if (arrowPos == std::string::npos) continue;
        std::string left = line.substr(0, arrowPos);
        std::string right = line.substr(arrowPos + 10);
        
        size_t dimPos = right.find("Dimensions (mm):");
        if (dimPos == std::string::npos) continue;
        std::string dimStr = right.substr(dimPos + 16);
        size_t pipePos = dimStr.find('|');
        std::string srcStr = "";
        if (pipePos != std::string::npos) {
            std::string rem = dimStr.substr(pipePos + 1);
            size_t sPos = rem.find("Source:");
            if (sPos != std::string::npos) srcStr = rem.substr(sPos + 7);
            dimStr = dimStr.substr(0, pipePos);
        }
        
        std::vector<std::string> parts;
        std::stringstream ss(dimStr);
        std::string item;
        while (std::getline(ss, item, 'x')) {
            auto trimStr = [](std::string s) {
                while (!s.empty() && std::isspace((unsigned char)s.front())) s.erase(s.begin());
                while (!s.empty() && std::isspace((unsigned char)s.back())) s.pop_back();
                return s;
            };
            std::string t = trimStr(item);
            if (!t.empty()) parts.push_back(t);
        }
        DatasheetEntry entry;
        try {
            if (parts.size() >= 1) entry.w_mm = std::stod(parts[0]);
            if (parts.size() >= 2) entry.h_mm = std::stod(parts[1]);
            if (parts.size() >= 3) entry.thick_mm = std::stod(parts[2]);
        } catch (...) {}
        while (!srcStr.empty() && std::isspace((unsigned char)srcStr.front())) srcStr.erase(srcStr.begin());
        while (!srcStr.empty() && std::isspace((unsigned char)srcStr.back())) srcStr.pop_back();
        entry.source = srcStr;

        std::istringstream iss(left);
        std::string ref, x, y, q, side, code;
        if (iss >> ref >> x >> y >> q >> side >> code) {
            table["REF:" + ref] = entry;
            table["CODE:" + code] = entry;
        }
    }
    return table;
}

ComponentAlignmentProcessor::ComponentAlignmentProcessor() = default;

// =========================================================================
// MAPPATURA COORDINATE CON CORREZIONE DELLA DISTORSIONE OTTICA
// =========================================================================

cv::Point2d ComponentAlignmentProcessor::pixelToAoi(
    const cv::Point2f& pt_px,
    const AoiAlignmentTransform& transform) {

    if (!transform.valid) {
        return cv::Point2d(0.0, 0.0);
    }

    // 1. Modello di distorsione ottica e prospettica tramite inversa dell'omografia 3x3
    if (transform.has_distortion_model && !transform.homography_3x3.empty()) {
        cv::Mat H_inv = transform.homography_inv_3x3;
        if (H_inv.empty()) {
            cv::invert(transform.homography_3x3, H_inv);
        }
        if (!H_inv.empty()) {
            std::vector<cv::Point2f> src = { pt_px };
            std::vector<cv::Point2f> dst;
            cv::perspectiveTransform(src, dst, H_inv);
            if (!dst.empty()) {
                return cv::Point2d(static_cast<double>(dst[0].x), static_cast<double>(dst[0].y));
            }
        }
    }

    // 2. Modello lineare rigido base inverso (fallback se omografia non attiva)
    const double s = (transform.scale_px_per_mm > 1e-5) ? transform.scale_px_per_mm : 1.0;
    double xr = (pt_px.x - transform.tx) / s;
    double yr = (pt_px.y - transform.ty) / s;

    if (std::abs(transform.residual_angle_deg) > 1e-4) {
        double rad = -transform.residual_angle_deg * CV_PI / 180.0;
        double cos_r = std::cos(rad);
        double sin_r = std::sin(rad);
        double x_unrot = cos_r * xr - sin_r * yr;
        double y_unrot = sin_r * xr + cos_r * yr;
        xr = x_unrot;
        yr = y_unrot;
    }

    double det = transform.mxx * transform.myy - transform.mxy * transform.myx;
    if (std::abs(det) < 1e-6) det = 1.0;

    double aoi_x = (transform.myy * xr - transform.mxy * yr) / det;
    double aoi_y = (-transform.myx * xr + transform.mxx * yr) / det;

    return cv::Point2d(aoi_x, aoi_y);
}

cv::Point2f ComponentAlignmentProcessor::aoiToPixel(
    const cv::Point2d& pt_mm,
    const AoiAlignmentTransform& transform) {
    return transform.aoiToPixel(pt_mm.x, pt_mm.y);
}

double ComponentAlignmentProcessor::normalizeAngleDiff(double realAngleDeg, double nominalAngleDeg) {
    double diff = realAngleDeg - nominalAngleDeg;

    // Riduci in [-180, 180)
    while (diff <= -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;

    // Simmetria del package rettangolare a 180°
    if (diff > 90.0) diff -= 180.0;
    else if (diff < -90.0) diff += 180.0;

    // Se l'orientamento è invertito di 90° (per via dell'ordine w/h di minAreaRect), riduci a [-45, 45]
    if (diff > 45.0) diff -= 90.0;
    else if (diff < -45.0) diff += 90.0;

    return diff;
}

// =========================================================================
// STRUTTURE E FUNZIONI PER FUSIONE CALIPER (OPZ. 2) + EDGE TEMPLATE (OPZ. 1)
// =========================================================================

struct CaliperEdgeResult {
    float dx = 0.0f;
    float dy = 0.0f;
    float dtheta = 0.0f;
    float confidence = 0.0f;
    bool has_reliable_angle = false;
    std::vector<cv::Point2f> edgePoints;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> rays;
};

/**
 * @brief Caliper 1D Edge Profiler Multi-Esposizione: campiona raggi perpendicolari lungo i 4 bordi del box,
 * rileva i picchi di gradiente direzionale in ciascuna esposizione, calcola il consenso mediano
 * per abbattere riflessi e ombre speculari, e adatta rette robuste con M-stimatore Huber.
 */
static CaliperEdgeResult runCaliperProfilingMultiExposure(
    const cv::Point2f& center0,
    float W_px,
    float H_px,
    float baseAngleDeg,
    const std::vector<cv::Mat>& expGradX,
    const std::vector<cv::Mat>& expGradY,
    const ComponentAlignmentConfig& config) {

    CaliperEdgeResult res;
    if (expGradX.empty()) return res;

    double rad = baseAngleDeg * CV_PI / 180.0;
    cv::Point2f u(static_cast<float>(std::cos(rad)), static_cast<float>(std::sin(rad)));
    cv::Point2f v(static_cast<float>(-std::sin(rad)), static_cast<float>(std::cos(rad)));

    struct EdgeSide {
        cv::Point2f start;
        cv::Point2f end;
        cv::Point2f normal;
        cv::Point2f tangent;
        float length;
        bool isWidth;
    };

    std::vector<EdgeSide> sides = {
        { center0 - 0.5f*W_px*u - 0.5f*H_px*v, center0 + 0.5f*W_px*u - 0.5f*H_px*v, -v, u, W_px, true },  // Top
        { center0 - 0.5f*W_px*u + 0.5f*H_px*v, center0 + 0.5f*W_px*u + 0.5f*H_px*v,  v, u, W_px, true },  // Bottom
        { center0 - 0.5f*W_px*u - 0.5f*H_px*v, center0 - 0.5f*W_px*u + 0.5f*H_px*v, -u, v, H_px, false }, // Left
        { center0 + 0.5f*W_px*u - 0.5f*H_px*v, center0 + 0.5f*W_px*u + 0.5f*H_px*v,  u, v, H_px, false }  // Right
    };

    float halfRayLen = config.caliper_ray_length_px;
    int numExp = static_cast<int>(expGradX.size());
    int totalRays = 0;
    int foundPoints = 0;

    std::vector<std::vector<cv::Point2f>> sideEdgePoints(4);

    for (int sIdx = 0; sIdx < 4; ++sIdx) {
        const auto& s = sides[sIdx];
        int numRays = std::max(4, std::min(9, static_cast<int>(std::round(s.length / 8.0f))));

        for (int r = 1; r <= numRays; ++r) {
            totalRays++;
            float t = static_cast<float>(r) / (numRays + 1);
            cv::Point2f rayCenter = s.start + t * s.length * s.tangent;
            cv::Point2f rStart = rayCenter - halfRayLen * s.normal;
            cv::Point2f rEnd = rayCenter + halfRayLen * s.normal;
            res.rays.push_back({rStart, rEnd});

            const int sampleSteps = 21;
            float step = (2.0f * halfRayLen) / (sampleSteps - 1);

            // Raccogli i picchi sub-pixel trovati in ciascuna esposizione
            std::vector<float> peakDists;

            for (int m = 0; m < numExp; ++m) {
                const cv::Mat& gx = expGradX[m];
                const cv::Mat& gy = expGradY[m];
                std::vector<float> profile(sampleSteps, 0.0f);
                float maxDeriv = 0.0f;
                int maxIdx = -1;

                for (int k = 0; k < sampleSteps; ++k) {
                    float dist = -halfRayLen + k * step;
                    cv::Point2f p = rayCenter + dist * s.normal;

                    if (p.x >= 1.0f && p.x < gx.cols - 1.0f && p.y >= 1.0f && p.y < gx.rows - 1.0f) {
                        int x0 = static_cast<int>(p.x);
                        int y0 = static_cast<int>(p.y);
                        float fx = p.x - x0;
                        float fy = p.y - y0;

                        float gxv = (1.0f-fx)*(1.0f-fy)*gx.at<float>(y0, x0) + fx*(1.0f-fy)*gx.at<float>(y0, x0+1) +
                                    (1.0f-fx)*fy*gx.at<float>(y0+1, x0) + fx*fy*gx.at<float>(y0+1, x0+1);
                        float gyv = (1.0f-fx)*(1.0f-fy)*gy.at<float>(y0, x0) + fx*(1.0f-fy)*gy.at<float>(y0, x0+1) +
                                    (1.0f-fx)*fy*gy.at<float>(y0+1, x0) + fx*fy*gy.at<float>(y0+1, x0+1);

                        float deriv = std::abs(gxv * s.normal.x + gyv * s.normal.y);
                        profile[k] = deriv;
                        if (deriv > maxDeriv) {
                            maxDeriv = deriv;
                            maxIdx = k;
                        }
                    }
                }

                if (maxIdx > 1 && maxIdx < sampleSteps - 2 && maxDeriv > 18.0f) {
                    float y1 = profile[maxIdx - 1];
                    float y2 = profile[maxIdx];
                    float y3 = profile[maxIdx + 1];
                    float denom = 2.0f * (y1 - 2.0f * y2 + y3);
                    float offset = (std::abs(denom) > 1e-4f) ? ((y1 - y3) / denom) : 0.0f;
                    offset = std::max(-0.5f, std::min(0.5f, offset));

                    float bestDist = -halfRayLen + (maxIdx + offset) * step;
                    peakDists.push_back(bestDist);
                }
            }

            // Consenso multi-esposizione: mediana dei picchi confermati
            if (peakDists.size() >= 2) {
                std::sort(peakDists.begin(), peakDists.end());
                float medianDist = peakDists[peakDists.size() / 2];

                // Conta quanti concordano con la mediana entro 0.8 pixel
                int consensusCount = 0;
                for (float d : peakDists) {
                    if (std::abs(d - medianDist) <= 0.8f) consensusCount++;
                }

                if (consensusCount >= 2) {
                    cv::Point2f subPixEdge = rayCenter + medianDist * s.normal;
                    sideEdgePoints[sIdx].push_back(subPixEdge);
                    res.edgePoints.push_back(subPixEdge);
                    foundPoints++;
                }
            }
        }
    }

    res.confidence = (totalRays > 0) ? (static_cast<float>(foundPoints) / totalRays) : 0.0f;

    std::vector<float> sideAngles(4, 0.0f);
    std::vector<cv::Point2f> sideCenters(4, cv::Point2f(0.0f, 0.0f));
    std::vector<bool> sideValid(4, false);

    for (int sIdx = 0; sIdx < 4; ++sIdx) {
        if (sideEdgePoints[sIdx].size() >= 3) {
            cv::Vec4f line;
            cv::fitLine(sideEdgePoints[sIdx], line, cv::DIST_HUBER, 0, 0.01, 0.01);
            float lineAngle = std::atan2(line[1], line[0]) * 180.0f / static_cast<float>(CV_PI);
            float expectedLineAngle = (sides[sIdx].isWidth) ? baseAngleDeg : (baseAngleDeg + 90.0f);
            float dAng = static_cast<float>(ComponentAlignmentProcessor::normalizeAngleDiff(lineAngle, expectedLineAngle));

            // Calcola errore quadratico medio dei punti rispetto alla retta
            float sumErr2 = 0.0f;
            for (const auto& pt : sideEdgePoints[sIdx]) {
                float dist = std::abs((pt.x - line[2]) * line[1] - (pt.y - line[3]) * line[0]);
                sumErr2 += dist * dist;
            }
            float rmse = std::sqrt(sumErr2 / sideEdgePoints[sIdx].size());

            // La retta è valida solo se l'angolo è entro la tolleranza e l'RMSE è basso (< 0.65 px)
            if (std::abs(dAng) <= config.max_rotation_deg && rmse < 0.65f) {
                sideAngles[sIdx] = dAng;
                sideValid[sIdx] = true;

                cv::Point2f m(0.0f, 0.0f);
                for (const auto& pt : sideEdgePoints[sIdx]) m += pt;
                m *= (1.0f / sideEdgePoints[sIdx].size());
                sideCenters[sIdx] = m;
            }
        }
    }

    std::vector<float> confirmedAngles;
    // Verifica consistenza tra bordi opposti paralleli
    if (sideValid[0] && sideValid[1] && std::abs(sideAngles[0] - sideAngles[1]) <= 1.2f) {
        confirmedAngles.push_back(0.5f * (sideAngles[0] + sideAngles[1]));
    }
    if (sideValid[2] && sideValid[3] && std::abs(sideAngles[2] - sideAngles[3]) <= 1.2f) {
        confirmedAngles.push_back(0.5f * (sideAngles[2] + sideAngles[3]));
    }

    if (!confirmedAngles.empty()) {
        std::sort(confirmedAngles.begin(), confirmedAngles.end());
        res.dtheta = confirmedAngles[confirmedAngles.size() / 2];
        res.has_reliable_angle = true;
        res.confidence = std::max(res.confidence, 0.85f);
    } else {
        // Se non abbiamo bordi opposti concordi, controlla se c'è almeno un bordo molto lungo con molti punti
        float bestLen = 0.0f;
        float bestSingleAng = 0.0f;
        bool foundSingle = false;
        for (int sIdx = 0; sIdx < 4; ++sIdx) {
            if (sideValid[sIdx] && sides[sIdx].length > 30.0f && sideEdgePoints[sIdx].size() >= 5) {
                if (sides[sIdx].length > bestLen) {
                    bestLen = sides[sIdx].length;
                    bestSingleAng = sideAngles[sIdx];
                    foundSingle = true;
                }
            }
        }
        if (foundSingle) {
            res.dtheta = bestSingleAng * 0.7f; // attenuazione prudente per bordo singolo
            res.has_reliable_angle = true;
            res.confidence = std::max(res.confidence, 0.60f);
        } else {
            // Nessuna evidenza fisica pulita: rotazione nominale (0.0°)
            res.dtheta = 0.0f;
            res.has_reliable_angle = false;
        }
    }

    res.dtheta = std::max(-config.max_rotation_deg, std::min(config.max_rotation_deg, res.dtheta));

    // Stima della traslazione baricentrica se bordi opposti validi
    cv::Point2f estCenter = center0;
    bool hasPair = false;
    if (sideValid[0] && sideValid[1]) {
        cv::Point2f midY = 0.5f * (sideCenters[0] + sideCenters[1]);
        estCenter.x = 0.5f * (estCenter.x + midY.x);
        estCenter.y = 0.5f * (estCenter.y + midY.y);
        hasPair = true;
    }
    if (sideValid[2] && sideValid[3]) {
        cv::Point2f midX = 0.5f * (sideCenters[2] + sideCenters[3]);
        estCenter.x = 0.5f * (estCenter.x + midX.x);
        estCenter.y = 0.5f * (estCenter.y + midX.y);
        hasPair = true;
    }

    if (hasPair) {
        res.dx = estCenter.x - center0.x;
        res.dy = estCenter.y - center0.y;
        res.dx = std::max(-config.max_translation_px, std::min(config.max_translation_px, res.dx));
        res.dy = std::max(-config.max_translation_px, std::min(config.max_translation_px, res.dy));
    }

    return res;
}

struct FusedPoseResult {
    float dx = 0.0f;
    float dy = 0.0f;
    float dtheta = 0.0f;
    float score = 0.0f;
};

/**
 * @brief Fusione MAP Multi-Esposizione: valuta l'allineamento dei bordi chiusi
 * con gradiente direzionale normalizzato sulle esposizioni multiple per eliminare
 * la trazione spuria di riflessi o piste adiacenti ad alto gradiente.
 */
static FusedPoseResult runFusedPoseEstimationMultiExposure(
    const cv::Point2f& center0,
    float W_px,
    float H_px,
    float baseAngleDeg,
    const std::vector<cv::Mat>& expGradX,
    const std::vector<cv::Mat>& expGradY,
    const std::vector<cv::Mat>& expGradMag,
    const ComponentAlignmentConfig& config,
    const CaliperEdgeResult& caliperRes) {

    FusedPoseResult res;
    if (expGradX.empty()) return res;

    // Discretizzazione perimetro del rettangolo
    std::vector<cv::Point2f> localPts;
    std::vector<cv::Point2f> localNormals;

    float halfW = W_px * 0.5f;
    float halfH = H_px * 0.5f;

    int nW = std::max(4, std::min(24, static_cast<int>(std::round(W_px * 0.5f))));
    for (int i = 0; i < nW; ++i) {
        float x = -halfW + (static_cast<float>(i) + 0.5f) * (W_px / nW);
        localPts.push_back(cv::Point2f(x, -halfH));
        localNormals.push_back(cv::Point2f(0.0f, -1.0f));

        localPts.push_back(cv::Point2f(x, halfH));
        localNormals.push_back(cv::Point2f(0.0f, 1.0f));
    }

    int nH = std::max(4, std::min(24, static_cast<int>(std::round(H_px * 0.5f))));
    for (int j = 0; j < nH; ++j) {
        float y = -halfH + (static_cast<float>(j) + 0.5f) * (H_px / nH);
        localPts.push_back(cv::Point2f(-halfW, y));
        localNormals.push_back(cv::Point2f(-1.0f, 0.0f));

        localPts.push_back(cv::Point2f(halfW, y));
        localNormals.push_back(cv::Point2f(1.0f, 0.0f));
    }

    size_t numPts = localPts.size();
    if (numPts == 0) return res;

    // Seleziona un sottoinsieme rappresentativo di esposizioni per velocità (3 esposizioni distanziate)
    std::vector<int> activeExpIndices;
    int numExp = static_cast<int>(expGradX.size());
    if (numExp <= 3) {
        for (int m = 0; m < numExp; ++m) activeExpIndices.push_back(m);
    } else {
        activeExpIndices.push_back(numExp / 6);              // Bassa esposizione (pad puliti)
        activeExpIndices.push_back(numExp / 2);              // Media esposizione (contrasto corpo)
        activeExpIndices.push_back(numExp - 1 - numExp / 6); // Alta esposizione (corpi scuri)
    }

    // Finestra di ricerca guidata dal Caliper
    float centerTx = caliperRes.dx;
    float centerTy = caliperRes.dy;
    float centerRot = caliperRes.dtheta;

    float searchRangeTx = caliperRes.has_reliable_angle ? 1.5f : config.max_translation_px;
    float searchRangeRot = caliperRes.has_reliable_angle ? 0.75f : 1.75f;
    float stepTx = std::max(0.5f, config.translation_step_px);
    float stepRot = std::max(0.25f, config.rotation_step_deg);

    float minRot = std::max(-config.max_rotation_deg, centerRot - searchRangeRot);
    float maxRot = std::min(config.max_rotation_deg, centerRot + searchRangeRot);
    float minTx = std::max(-config.max_translation_px, centerTx - searchRangeTx);
    float maxTx = std::min(config.max_translation_px, centerTx + searchRangeTx);
    float minTy = std::max(-config.max_translation_px, centerTy - searchRangeTx);
    float maxTy = std::min(config.max_translation_px, centerTy + searchRangeTx);

    float bestScore = -1e9f;
    float bestDx = centerTx;
    float bestDy = centerTy;
    float bestDtheta = centerRot;

    for (float dtheta = minRot; dtheta <= maxRot + 1e-4f; dtheta += stepRot) {
        double currentAngleRad = (baseAngleDeg + dtheta) * CV_PI / 180.0;
        float cosA = static_cast<float>(std::cos(currentAngleRad));
        float sinA = static_cast<float>(std::sin(currentAngleRad));

        std::vector<cv::Point2f> rotPts(numPts);
        std::vector<cv::Point2f> rotNormals(numPts);
        for (size_t k = 0; k < numPts; ++k) {
            rotPts[k].x = cosA * localPts[k].x - sinA * localPts[k].y;
            rotPts[k].y = sinA * localPts[k].x + cosA * localPts[k].y;
            rotNormals[k].x = cosA * localNormals[k].x - sinA * localNormals[k].y;
            rotNormals[k].y = sinA * localNormals[k].x + cosA * localNormals[k].y;
        }

        for (float dy = minTy; dy <= maxTy + 1e-4f; dy += stepTx) {
            for (float dx = minTx; dx <= maxTx + 1e-4f; dx += stepTx) {
                float multiScoreSum = 0.0f;
                int validExposures = 0;

                for (int m : activeExpIndices) {
                    const cv::Mat& gx = expGradX[m];
                    const cv::Mat& gy = expGradY[m];
                    const cv::Mat& gmag = expGradMag[m];
                    float expAlignSum = 0.0f;
                    int validCount = 0;

                    for (size_t k = 0; k < numPts; ++k) {
                        float px = center0.x + dx + rotPts[k].x;
                        float py = center0.y + dy + rotPts[k].y;

                        if (px >= 1.0f && px < gmag.cols - 1.0f && py >= 1.0f && py < gmag.rows - 1.0f) {
                            int x0 = static_cast<int>(px);
                            int y0 = static_cast<int>(py);
                            float fx = px - x0;
                            float fy = py - y0;

                            float gxv = (1.0f-fx)*(1.0f-fy)*gx.at<float>(y0, x0) + fx*(1.0f-fy)*gx.at<float>(y0, x0+1) +
                                        (1.0f-fx)*fy*gx.at<float>(y0+1, x0) + fx*fy*gx.at<float>(y0+1, x0+1);
                            float gyv = (1.0f-fx)*(1.0f-fy)*gy.at<float>(y0, x0) + fx*(1.0f-fy)*gy.at<float>(y0, x0+1) +
                                        (1.0f-fx)*fy*gy.at<float>(y0+1, x0) + fx*fy*gy.at<float>(y0+1, x0+1);
                            float magv = (1.0f-fx)*(1.0f-fy)*gmag.at<float>(y0, x0) + fx*(1.0f-fy)*gmag.at<float>(y0, x0+1) +
                                         (1.0f-fx)*fy*gmag.at<float>(y0+1, x0) + fx*fy*gmag.at<float>(y0+1, x0+1);

                            float dotNorm = std::abs(gxv * rotNormals[k].x + gyv * rotNormals[k].y);
                            float cosAlign = (magv > 1e-3f) ? (dotNorm / magv) : 0.0f;
                            // Normalizzazione: intensità saturata a 35.0f, cosAlign^2 punisce le deviazioni angolari
                            float edgeWeight = std::min(1.0f, magv / 35.0f);
                            expAlignSum += edgeWeight * (cosAlign * cosAlign);
                            validCount++;
                        }
                    }

                    if (validCount >= static_cast<int>(numPts * 0.5f)) {
                        multiScoreSum += (expAlignSum / validCount);
                        validExposures++;
                    }
                }

                if (validExposures == 0) continue;
                float normalizedTemplateScore = multiScoreSum / validExposures;

                // Penalità Caliper se affidabile
                float calPenalty = 0.0f;
                if (caliperRes.has_reliable_angle) {
                    float diffX = (dx - caliperRes.dx);
                    float diffY = (dy - caliperRes.dy);
                    float diffRot = (dtheta - caliperRes.dtheta);
                    calPenalty = caliperRes.confidence * (diffX * diffX + diffY * diffY + 1.5f * diffRot * diffRot) * 0.4f;
                }

                // Penalità quadratica verso orientamento nominale (blocca rotazioni fittizie)
                float nomPenalty = 0.04f * (dx * dx + dy * dy) + 0.08f * (dtheta * dtheta);
                float fusedScore = normalizedTemplateScore - calPenalty - nomPenalty;

                if (fusedScore > bestScore) {
                    bestScore = fusedScore;
                    bestDx = dx;
                    bestDy = dy;
                    bestDtheta = dtheta;
                }
            }
        }
    }

    res.dx = bestDx;
    res.dy = bestDy;
    res.dtheta = bestDtheta;
    res.score = bestScore;
    return res;
}

// =========================================================================
// PIPELINE PRINCIPALE DI FUSIONE CALIPER + EDGE-TEMPLATE
// =========================================================================

ComponentAlignmentResult ComponentAlignmentProcessor::process(
    const cv::Mat& pcbImage,
    const std::vector<VerifiedPcbComponent>& inputComponents,
    const AoiAlignmentTransform& transform,
    const ComponentAlignmentConfig& config,
    const std::vector<cv::Mat>& multiExposureImages) {

    ComponentAlignmentResult result;
    result.success = false;
    result.summary.total_input_components = static_cast<int>(inputComponents.size());

    if (pcbImage.empty() || inputComponents.empty()) {
        std::cerr << "[ALIGNMENT] Immagine PCB vuota o nessun componente in ingresso." << std::endl;
        return result;
    }

    std::vector<cv::Mat> expImgs = multiExposureImages;
    if (expImgs.empty()) {
        expImgs.push_back(pcbImage);
    }

    const double s = (transform.scale_px_per_mm > 1e-5) ? transform.scale_px_per_mm : 1.0;
    const int imgW = pcbImage.cols;
    const int imgH = pcbImage.rows;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "=== AVVIO MODELLAZIONE PRECISA COMPONENTI (CALIPER + TEMPLATE) ===" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << "[ALIGNMENT] Totale componenti in ingresso: " << inputComponents.size()
              << " | Immagini multi-esposizione: " << expImgs.size() << std::endl;

    // 1. Inizializzazione immagini gradiente Sobel multi-esposizione
    cv::Mat maxGradX = cv::Mat::zeros(pcbImage.size(), CV_32F);
    cv::Mat maxGradY = cv::Mat::zeros(pcbImage.size(), CV_32F);
    cv::Mat maxGradMag = cv::Mat::zeros(pcbImage.size(), CV_32F);

    std::vector<cv::Mat> expGradX, expGradY, expGradMag;
    expGradX.reserve(expImgs.size());
    expGradY.reserve(expImgs.size());
    expGradMag.reserve(expImgs.size());

    for (const auto& img : expImgs) {
        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = img.clone();
        }
        cv::Mat gx, gy, mag;
        cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
        cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
        cv::magnitude(gx, gy, mag);

        expGradX.push_back(gx);
        expGradY.push_back(gy);
        expGradMag.push_back(mag);

        for (int r = 0; r < imgH; ++r) {
            const float* mPtr = mag.ptr<float>(r);
            const float* gxPtr = gx.ptr<float>(r);
            const float* gyPtr = gy.ptr<float>(r);
            float* maxMPtr = maxGradMag.ptr<float>(r);
            float* maxGXPtr = maxGradX.ptr<float>(r);
            float* maxGYPtr = maxGradY.ptr<float>(r);

            for (int c = 0; c < imgW; ++c) {
                if (mPtr[c] > maxMPtr[c]) {
                    maxMPtr[c] = mPtr[c];
                    maxGXPtr[c] = gxPtr[c];
                    maxGYPtr[c] = gyPtr[c];
                }
            }
        }
    }

    // Salvataggio maschere diagnostiche dei gradienti
    double minV, maxV;
    cv::minMaxLoc(maxGradMag, &minV, &maxV);
    cv::Mat gradMagVis;
    maxGradMag.convertTo(gradMagVis, CV_8UC1, 255.0 / (maxV > 0.0 ? maxV : 1.0));
    result.intermediate_filter_masks["01_multi_exposure_grad_mag"] = gradMagVis;

    cv::Mat gxVis, gyVis;
    cv::convertScaleAbs(maxGradX, gxVis, 0.5);
    cv::convertScaleAbs(maxGradY, gyVis, 0.5);
    result.intermediate_filter_masks["02_sobel_gradient_x"] = gxVis;
    result.intermediate_filter_masks["03_sobel_gradient_y"] = gyVis;

    // 2. Filtra componenti validi (escludendo eventuali UNMATCHED_DETECTION non presenti in distinta AOI)
    std::vector<VerifiedPcbComponent> validInputs;
    for (const auto& comp : inputComponents) {
        if (comp.status == AoiMatchStatus::UNMATCHED_DETECTION) continue;
        validInputs.push_back(comp);
    }

    // =========================================================================
    // REQUISITO: "ricorda che componenti con stesso codice produttore hanno
    //             lo stesso package e quindi devono avere lo stesso ingombro in mm"
    // =========================================================================
    std::map<std::string, std::vector<std::pair<double, double>>> partMmSizesMap;
    std::map<std::string, std::pair<double, double>> datasheetSizeMmMap;

    for (const auto& comp : validInputs) {
        std::string key = !comp.aoi_part_number.empty() ? comp.aoi_part_number : comp.label;

        double w_mm = std::max(comp.box.width, comp.box.height) / s;
        double h_mm = std::min(comp.box.width, comp.box.height) / s;

        // Se è una detection validata da processed_output (non sintetizzata ex novo da AOI), aggiungila al campione statistico
        if (comp.status != AoiMatchStatus::RECOVERED_BY_AOI && w_mm > 0.2 && h_mm > 0.2) {
            partMmSizesMap[key].push_back({w_mm, h_mm});
        }

        double dsW = comp.datasheet_width_mm > 0.05 ? comp.datasheet_width_mm : comp.nominal_width_mm;
        double dsH = comp.datasheet_height_mm > 0.05 ? comp.datasheet_height_mm : comp.nominal_height_mm;
        if (dsW > 0.05 && dsH > 0.05) {
            datasheetSizeMmMap[key] = { std::max(dsW, dsH), std::min(dsW, dsH) };
        }
    }

    for (const auto& entry : partMmSizesMap) {
        const std::string& key = entry.first;
        const auto& samples = entry.second;
        if (samples.empty()) continue;

        std::vector<double> wVec, hVec;
        for (const auto& pr : samples) {
            wVec.push_back(pr.first);
            hVec.push_back(pr.second);
        }
        std::sort(wVec.begin(), wVec.end());
        std::sort(hVec.begin(), hVec.end());
        double medW = wVec[wVec.size() / 2];
        double medH = hVec[hVec.size() / 2];

        auto itDs = datasheetSizeMmMap.find(key);
        if (itDs != datasheetSizeMmMap.end()) {
            // Rispetta le dimensioni minime garantite da datasheet (es. condensatori elettrolitici)
            medW = std::max(medW, itDs->second.first * 0.92);
            medH = std::max(medH, itDs->second.second * 0.92);
        }

        result.summary.canonical_part_sizes_mm[key] = cv::Size2d(medW, medH);
        result.summary.canonical_part_sizes_px[key] = cv::Size2f(static_cast<float>(medW * s), static_cast<float>(medH * s));
    }

    // Se per qualche part_number non avevamo campioni da processed_output, usa il datasheet/nominale
    for (const auto& entry : datasheetSizeMmMap) {
        if (result.summary.canonical_part_sizes_mm.find(entry.first) == result.summary.canonical_part_sizes_mm.end()) {
            double dw = entry.second.first;
            double dh = entry.second.second;
            result.summary.canonical_part_sizes_mm[entry.first] = cv::Size2d(dw, dh);
            result.summary.canonical_part_sizes_px[entry.first] = cv::Size2f(static_cast<float>(dw * s), static_cast<float>(dh * s));
        }
    }

    // =========================================================================
    // REQUISITO: "parti dai componenti più grandi ai componenti più piccoli"
    // =========================================================================
    std::sort(validInputs.begin(), validInputs.end(), [&](const VerifiedPcbComponent& a, const VerifiedPcbComponent& b) {
        float areaA = (a.rotated_box.size.width > 0.0f) ? (a.rotated_box.size.width * a.rotated_box.size.height) : (a.box.width * a.box.height);
        float areaB = (b.rotated_box.size.width > 0.0f) ? (b.rotated_box.size.width * b.rotated_box.size.height) : (b.box.width * b.box.height);
        return areaA > areaB;
    });

    std::cout << "[ALIGNMENT] Ordinamento completato: componenti ordinati per area decrescente (da max a min)." << std::endl;

    // Maschera per sovrimpressione raggi caliper (per visualizzazione intermedia)
    cv::Mat caliperVis = pcbImage.clone();

    // Elaborazione fine di ciascun componente
    std::vector<AlignedComponent> processedComps;
    double sumErrDistMm = 0.0;
    double sumErrDistPx = 0.0;
    double sumErrRotDeg = 0.0;
    int validErrorCount = 0;

    for (const auto& inComp : validInputs) {
        AlignedComponent seg;
        seg.id = inComp.id;
        seg.ref_des = inComp.ref_des;
        seg.label = inComp.label;
        seg.part_number = inComp.aoi_part_number;
        seg.outline = inComp.aoi_outline;

        // =========================================================================
        // REQUISITO UTENTE: "come boundery box devi usare le box trovate in
        // processed output e aoi verification, sono corrette (non usare le dimensioni in mm)"
        // =========================================================================
        float boxW_px = inComp.box.width;
        float boxH_px = inComp.box.height;
        float baseAngle = 0.0f;

        if (inComp.rotated_box.size.width > 1.0f && inComp.rotated_box.size.height > 1.0f) {
            boxW_px = inComp.rotated_box.size.width;
            boxH_px = inComp.rotated_box.size.height;
            baseAngle = inComp.rotated_box.angle;
        }

        seg.box_width_px = boxW_px;
        seg.box_height_px = boxH_px;
        seg.box_width_mm = boxW_px / s;
        seg.box_height_mm = boxH_px / s;

        // Quote datasheet reali
        seg.datasheet_width_mm = inComp.datasheet_width_mm > 0.05 ? inComp.datasheet_width_mm : inComp.nominal_width_mm;
        seg.datasheet_height_mm = inComp.datasheet_height_mm > 0.05 ? inComp.datasheet_height_mm : inComp.nominal_height_mm;
        seg.datasheet_thickness_mm = inComp.datasheet_thickness_mm;
        seg.datasheet_source = inComp.datasheet_source;

        // Centro nominale e iniziale
        cv::Point2f center0 = inComp.center;
        if (center0.x <= 1.0f || center0.y <= 1.0f) {
            center0 = inComp.nominal_center_px;
        }
        seg.nominal_center_px = inComp.nominal_center_px;
        seg.nominal_x_mm = inComp.nominal_x_mm;
        seg.nominal_y_mm = inComp.nominal_y_mm;
        seg.nominal_rot_deg = inComp.nominal_rot_deg;

        // =========================================================================
        // FUSIONE OPZIONE 2 (CALIPER 1D) + OPZIONE 1 (EDGE-TEMPLATE MATCHING) MULTI-ESPOSIZIONE
        // =========================================================================
        // Fase 1: Caliper Edge Profiler 1D Multi-Esposizione (Consenso Mediano dei Picchi)
        CaliperEdgeResult calRes = runCaliperProfilingMultiExposure(
            center0, seg.box_width_px, seg.box_height_px, baseAngle,
            expGradX, expGradY, config);

        // Disegna raggi caliper sulla maschera intermedia
        for (const auto& ray : calRes.rays) {
            cv::line(caliperVis, ray.first, ray.second, cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
        }
        for (const auto& ep : calRes.edgePoints) {
            cv::circle(caliperVis, ep, 1, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
        }

        // Fase 2: Edge-Template MAP Multi-Esposizione con Gradiente Normalizzato
        FusedPoseResult fusedPose = runFusedPoseEstimationMultiExposure(
            center0, seg.box_width_px, seg.box_height_px, baseAngle,
            expGradX, expGradY, expGradMag, config, calRes);

        // Applica micro-traslazione e micro-rotazione ottimali
        cv::Point2f refinedCenter = center0 + cv::Point2f(fusedPose.dx, fusedPose.dy);
        float refinedAngle = baseAngle + fusedPose.dtheta;

        cv::RotatedRect tightBox(refinedCenter, cv::Size2f(seg.box_width_px, seg.box_height_px), refinedAngle);
        seg.rotated_box = tightBox;
        seg.bounding_box = tightBox.boundingRect2f();
        seg.pixel_center = tightBox.center;
        seg.pixel_angle_deg = tightBox.angle;

        // Estrazione contorno esatto rettangolare aderente
        cv::Point2f pts[4];
        tightBox.points(pts);
        seg.contour.clear();
        for (int j = 0; j < 4; ++j) {
            seg.contour.push_back(cv::Point(static_cast<int>(std::round(pts[j].x)),
                                            static_cast<int>(std::round(pts[j].y))));
        }
        seg.area_px = seg.box_width_px * seg.box_height_px;
        seg.found = true;
        seg.search_expansion_level = 0;

        // 4. Calcolo reale posizione in mm (con distorsione ottica omografica) ed errori
        cv::Point2d realMm = pixelToAoi(seg.pixel_center, transform);
        seg.real_x_mm = realMm.x;
        seg.real_y_mm = realMm.y;

        seg.error_x_mm = seg.real_x_mm - seg.nominal_x_mm;
        seg.error_y_mm = seg.real_y_mm - seg.nominal_y_mm;
        seg.error_dist_mm = std::hypot(seg.error_x_mm, seg.error_y_mm);
        seg.error_dist_px = seg.error_dist_mm * s;

        // Errore di rotazione: micro-rotazione residua stimata dall'algoritmo (in gradi)
        seg.error_rot_deg = fusedPose.dtheta;
        seg.real_rot_deg = seg.nominal_rot_deg + seg.error_rot_deg;
        while (seg.real_rot_deg < 0.0) seg.real_rot_deg += 360.0;
        while (seg.real_rot_deg >= 360.0) seg.real_rot_deg -= 360.0;

        if (inComp.status == AoiMatchStatus::RECOVERED_BY_AOI) {
            result.summary.recovered_by_aoi++;
            seg.notes = "Recovered by AOI; Fused Caliper+Template Refined";
        } else {
            result.summary.from_processed_output++;
            seg.notes = "Processed Output verified; Fused Caliper+Template Refined";
        }
        result.summary.successfully_aligned++;
        result.summary.preserved_original_box++;

        sumErrDistMm += seg.error_dist_mm;
        sumErrDistPx += seg.error_dist_px;
        sumErrRotDeg += std::abs(seg.error_rot_deg);
        validErrorCount++;

        result.summary.class_distribution[seg.label]++;
        processedComps.push_back(seg);
    }

    result.intermediate_filter_masks["04_caliper_rays_overlay"] = caliperVis;

    if (validErrorCount > 0) {
        result.summary.mean_error_dist_mm = sumErrDistMm / validErrorCount;
        result.summary.mean_error_dist_px = sumErrDistPx / validErrorCount;
        result.summary.mean_error_rot_deg = sumErrRotDeg / validErrorCount;
    }

    result.aligned_components = processedComps;

    std::cout << "\n========================================================" << std::endl;
    std::cout << "=== RIEPILOGO FINALE FUSIONE CALIPER + EDGE-TEMPLATE ===" << std::endl;
    std::cout << "  - Totale componenti elaborati:       " << result.summary.total_input_components << std::endl;
    std::cout << "  - Componenti da Processed Output:    " << result.summary.from_processed_output << std::endl;
    std::cout << "  - Componenti recuperati da AOI:      " << result.summary.recovered_by_aoi << std::endl;
    std::cout << "  - Micro-orientamento rifinito:       " << result.summary.successfully_aligned << " / " << result.summary.total_input_components << std::endl;
    std::cout << "  - Scostamento medio reale/nominale:  "
              << std::fixed << std::setprecision(3) << result.summary.mean_error_dist_mm << " mm ("
              << std::setprecision(2) << result.summary.mean_error_dist_px << " px)" << std::endl;
    std::cout << "  - Errore medio micro-rotazione:      "
              << std::fixed << std::setprecision(2) << result.summary.mean_error_rot_deg << "°" << std::endl;
    std::cout << "========================================================" << std::endl;

    // =========================================================================
    // RENDERING GRAFICO
    // =========================================================================
    auto palette = common::getPalette(common::getDefaultClasses());

    // 1. Immagine Contorni Reali (bordi sottili e colorati)
    cv::Mat contoursImg = pcbImage.clone();
    for (const auto& comp : processedComps) {
        cv::Scalar color(0, 255, 0);
        auto it = palette.find(comp.label);
        if (it != palette.end()) color = it->second;

        std::vector<std::vector<cv::Point>> cList = { comp.contour };
        cv::drawContours(contoursImg, cList, -1, color, config.contour_thickness, cv::LINE_AA);
        cv::circle(contoursImg, comp.pixel_center, 2, color, -1, cv::LINE_AA);

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(comp.ref_des, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        cv::Point textOrg(static_cast<int>(comp.pixel_center.x - textSize.width * 0.5f),
                          static_cast<int>(comp.pixel_center.y - 4));
        textOrg.x = std::max(2, std::min(imgW - textSize.width - 2, textOrg.x));
        textOrg.y = std::max(textSize.height + 2, std::min(imgH - 2, textOrg.y));

        cv::putText(contoursImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(contoursImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> extraStats = {
        {"Totale Componenti", static_cast<int>(processedComps.size())},
        {"Da Processed Out", result.summary.from_processed_output},
        {"Recuperati AOI", result.summary.recovered_by_aoi}
    };
    cv::Mat sideLegend1 = common::renderSideLegend(
        contoursImg.rows,
        result.summary.class_distribution,
        static_cast<int>(processedComps.size()),
        palette,
        "Contorni Allineati",
        extraStats,
        340);
    cv::Mat combinedContoursVis;
    cv::hconcat(contoursImg, sideLegend1, combinedContoursVis);
    result.contours_vis = combinedContoursVis;

    // 2. Immagine Rettangoli Minimi Aderenti Senza Spazi
    cv::Mat boxesImg = pcbImage.clone();
    for (const auto& comp : processedComps) {
        cv::Scalar color(0, 255, 0);
        auto it = palette.find(comp.label);
        if (it != palette.end()) color = it->second;

        cv::Point2f pts[4];
        comp.rotated_box.points(pts);
        for (int j = 0; j < 4; ++j) {
            cv::line(boxesImg, pts[j], pts[(j + 1) % 4], color, 1, cv::LINE_AA);
        }
        cv::circle(boxesImg, comp.pixel_center, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(comp.ref_des, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        cv::Point textOrg(static_cast<int>(comp.pixel_center.x - textSize.width * 0.5f),
                          static_cast<int>(comp.pixel_center.y - 4));
        textOrg.x = std::max(2, std::min(imgW - textSize.width - 2, textOrg.x));
        textOrg.y = std::max(textSize.height + 2, std::min(imgH - 2, textOrg.y));

        cv::putText(boxesImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(boxesImg, comp.ref_des, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    cv::Mat sideLegend2 = common::renderSideLegend(
        boxesImg.rows,
        result.summary.class_distribution,
        static_cast<int>(processedComps.size()),
        palette,
        "Rettangoli Aderenti",
        extraStats,
        340);
    cv::Mat combinedBoxesVis;
    cv::hconcat(boxesImg, sideLegend2, combinedBoxesVis);
    result.tight_boxes_vis = combinedBoxesVis;

    result.success = true;
    return result;
}

// =========================================================================
// SALVATAGGIO DEI RISULTATI SU DISCO
// =========================================================================

bool ComponentAlignmentProcessor::saveResults(
    const std::string& outputDir,
    const std::string& fileStem,
    const ComponentAlignmentResult& result) {

    fs::create_directories(outputDir);

    std::string csvPath = (fs::path(outputDir) / (fileStem + "_precise_aligned_components.csv")).string();
    std::ofstream csv(csvPath);
    if (!csv.is_open()) {
        std::cerr << "[ALIGNMENT] Impossibile creare il file CSV: " << csvPath << std::endl;
        return false;
    }

    csv << "id,componente_codice,tipologia_classe,codice_produttore,package_outline,"
        << "box_width_px,box_height_px,box_width_mm,box_height_mm,"
        << "datasheet_width_mm,datasheet_height_mm,datasheet_thickness_mm,datasheet_source,"
        << "pixel_center_x,pixel_center_y,pixel_angle_deg,area_px,"
        << "reale_pos_x_mm,reale_pos_y_mm,reale_rot_deg,"
        << "nominale_x_mm,nominale_y_mm,nominale_rot_deg,"
        << "errore_pos_x_mm,errore_pos_y_mm,errore_dist_mm,errore_dist_px,errore_rot_deg,"
        << "search_expansion_level,notes\n";

    for (const auto& c : result.aligned_components) {
        csv << c.id << ","
            << "\"" << c.ref_des << "\","
            << "\"" << c.label << "\","
            << "\"" << c.part_number << "\","
            << "\"" << c.outline << "\","
            << std::fixed << std::setprecision(2)
            << c.box_width_px << ","
            << c.box_height_px << ","
            << std::setprecision(3)
            << c.box_width_mm << ","
            << c.box_height_mm << ","
            << c.datasheet_width_mm << ","
            << c.datasheet_height_mm << ","
            << c.datasheet_thickness_mm << ",\""
            << c.datasheet_source << "\","
            << std::setprecision(2)
            << c.pixel_center.x << ","
            << c.pixel_center.y << ","
            << c.pixel_angle_deg << ","
            << c.area_px << ","
            << std::setprecision(4)
            << c.real_x_mm << ","
            << c.real_y_mm << ","
            << std::setprecision(2)
            << c.real_rot_deg << ","
            << std::setprecision(4)
            << c.nominal_x_mm << ","
            << c.nominal_y_mm << ","
            << std::setprecision(2)
            << c.nominal_rot_deg << ","
            << std::setprecision(4)
            << c.error_x_mm << ","
            << c.error_y_mm << ","
            << c.error_dist_mm << ","
            << std::setprecision(2)
            << c.error_dist_px << ","
            << c.error_rot_deg << ","
            << c.search_expansion_level << ","
            << "\"" << c.notes << "\"\n";
    }
    csv.close();
    std::cout << "[ALIGNMENT] Salvato CSV componenti allineati: " << csvPath << std::endl;

    std::string jsonPath = (fs::path(outputDir) / (fileStem + "_precise_aligned_summary.json")).string();
    std::ofstream json(jsonPath);
    if (json.is_open()) {
        json << "{\n"
             << "  \"board_name\": \"" << result.board_name << "\",\n"
             << "  \"side\": \"" << result.side << "\",\n"
             << "  \"total_input_components\": " << result.summary.total_input_components << ",\n"
             << "  \"successfully_aligned\": " << result.summary.successfully_aligned << ",\n"
             << "  \"from_processed_output\": " << result.summary.from_processed_output << ",\n"
             << "  \"recovered_by_aoi\": " << result.summary.recovered_by_aoi << ",\n"
             << "  \"preserved_original_box\": " << result.summary.preserved_original_box << ",\n"
             << "  \"mean_error_dist_mm\": " << std::fixed << std::setprecision(4) << result.summary.mean_error_dist_mm << ",\n"
             << "  \"mean_error_dist_px\": " << std::fixed << std::setprecision(2) << result.summary.mean_error_dist_px << ",\n"
             << "  \"mean_error_rot_deg\": " << std::fixed << std::setprecision(2) << result.summary.mean_error_rot_deg << ",\n"
             << "  \"canonical_part_number_sizes_mm\": {\n";

        size_t partIdx = 0;
        for (const auto& p : result.summary.canonical_part_sizes_mm) {
            json << "    \"" << p.first << "\": {\"width_mm\": "
                 << std::fixed << std::setprecision(3) << p.second.width
                 << ", \"height_mm\": " << p.second.height << "}";
            if (++partIdx < result.summary.canonical_part_sizes_mm.size()) json << ",";
            json << "\n";
        }
        json << "  },\n";

        json << "  \"class_distribution\": {\n";
        size_t cIdx = 0;
        for (const auto& c : result.summary.class_distribution) {
            json << "    \"" << c.first << "\": " << c.second;
            if (++cIdx < result.summary.class_distribution.size()) json << ",";
            json << "\n";
        }
        json << "  }\n";
        json << "}\n";
        json.close();
        std::cout << "[ALIGNMENT] Salvato JSON riepilogo: " << jsonPath << std::endl;
    }

    if (!result.contours_vis.empty()) {
        std::string contoursPath = (fs::path(outputDir) / (fileStem + "_aligned_contours.jpg")).string();
        cv::imwrite(contoursPath, result.contours_vis);
        std::cout << "[ALIGNMENT] Salvata immagine contorni sottili e colorati: " << contoursPath << std::endl;
    }

    if (!result.tight_boxes_vis.empty()) {
        std::string boxesPath = (fs::path(outputDir) / (fileStem + "_precise_bounding_boxes.jpg")).string();
        cv::imwrite(boxesPath, result.tight_boxes_vis);
        std::cout << "[ALIGNMENT] Salvata immagine rettangoli aderenti: " << boxesPath << std::endl;
    }

    // Salvataggio maschere diagnostiche
    if (!result.intermediate_filter_masks.empty()) {
        fs::path filtersDir = fs::path(outputDir) / "intermediate_filters";
        fs::create_directories(filtersDir);
        for (const auto& entry : result.intermediate_filter_masks) {
            std::string filterPath = (filtersDir / (entry.first + ".png")).string();
            cv::imwrite(filterPath, entry.second);
        }
        std::cout << "[ALIGNMENT] Salvate " << result.intermediate_filter_masks.size()
                  << " maschere dei filtri intermedi in: " << filtersDir << std::endl;
    }

    return true;
}

// =========================================================================
// ELABORAZIONE AD ALTO LIVELLO CON FUSIONE MULTI-ESPOSIZIONE
// =========================================================================

ComponentAlignmentResult ComponentAlignmentProcessor::processBoardSide(
    const std::string& boardName,
    const std::string& side,
    const std::string& undistortedDir,
    const std::string& aoiVerificationDir,
    const std::string& paramsDir,
    const std::string& outputBaseDir,
    const ComponentAlignmentConfig& config) {

    ComponentAlignmentResult res;
    res.board_name = boardName;
    res.side = side;

    fs::path aoiSideDir = fs::path(aoiVerificationDir) / boardName / side;
    if (!fs::exists(aoiSideDir)) {
        std::cerr << "[ALIGNMENT] Cartella AOI verification non trovata: " << aoiSideDir << std::endl;
        return res;
    }

    // 1. Cerca il file CSV verificato prodotto da AOI verification
    std::string verifiedCsvPath;
    for (const auto& entry : fs::directory_iterator(aoiSideDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            std::string name = entry.path().filename().string();
            if (name.find("_aoi_verified_components.csv") != std::string::npos) {
                verifiedCsvPath = entry.path().string();
                break;
            }
        }
    }

    if (verifiedCsvPath.empty()) {
        std::cerr << "[ALIGNMENT] File CSV AOI verified non trovato in: " << aoiSideDir << std::endl;
        return res;
    }

    std::string fileStem = fs::path(verifiedCsvPath).stem().string();
    size_t pos = fileStem.find("_aoi_verified_components");
    if (pos != std::string::npos) {
        fileStem = fileStem.substr(0, pos);
    }
    res.reference_image_name = fileStem;

    // 2. Carica l'immagine PCB di riferimento non distorta
    fs::path imgPath = fs::path(undistortedDir) / boardName / side / (fileStem + ".bmp");
    if (!fs::exists(imgPath)) {
        imgPath = fs::path(undistortedDir) / boardName / side / (fileStem + ".jpg");
    }
    if (!fs::exists(imgPath)) {
        fs::path sideDir = fs::path(undistortedDir) / boardName / side;
        if (fs::exists(sideDir)) {
            for (const auto& entry : fs::directory_iterator(sideDir)) {
                if (entry.is_regular_file() && (entry.path().extension() == ".bmp" || entry.path().extension() == ".jpg")) {
                    imgPath = entry.path();
                    break;
                }
            }
        }
    }

    cv::Mat pcbImage = cv::imread(imgPath.string());
    if (pcbImage.empty()) {
        std::cerr << "[ALIGNMENT] Impossibile caricare immagine PCB: " << imgPath << std::endl;
        return res;
    }

    // Caricamento immagini multi-esposizione
    fs::path sideDir = fs::path(undistortedDir) / boardName / side;
    std::vector<fs::path> exposureFiles;
    if (fs::exists(sideDir)) {
        for (const auto& entry : fs::directory_iterator(sideDir)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".bmp" || ext == ".jpg") {
                    exposureFiles.push_back(entry.path());
                }
            }
        }
    }
    std::sort(exposureFiles.begin(), exposureFiles.end());

    std::vector<cv::Mat> multiExposureImages;
    if (!exposureFiles.empty()) {
        std::cout << "[ALIGNMENT] Caricamento di " << exposureFiles.size()
                  << " immagini a esposizione diversa per estrazione multi-modale..." << std::endl;
        for (const auto& expPath : exposureFiles) {
            cv::Mat expImg = cv::imread(expPath.string());
            if (!expImg.empty() && expImg.size() == pcbImage.size()) {
                multiExposureImages.push_back(expImg);
            }
        }
        std::cout << "[ALIGNMENT] Caricate con successo " << multiExposureImages.size()
                  << " immagini multi-esposizione." << std::endl;
    }
    if (multiExposureImages.empty()) {
        multiExposureImages.push_back(pcbImage);
    }

    // 3. Carica i parametri di trasformazione e modello di distorsione YAML
    fs::path yamlPath = fs::path(paramsDir) / ("aoi_alignment_" + boardName + "_" + side + ".yml");
    AoiAlignmentTransform transform;
    if (fs::exists(yamlPath)) {
        cv::FileStorage fs(yamlPath.string(), cv::FileStorage::READ);
        if (fs.isOpened()) {
            transform.valid = true;
            fs["orientation_name"] >> transform.orientation_name;
            int hasDist = 0;
            fs["has_distortion_model"] >> hasDist;
            transform.has_distortion_model = (hasDist != 0);
            fs["homography_3x3"] >> transform.homography_3x3;
            if (!transform.homography_3x3.empty()) {
                cv::invert(transform.homography_3x3, transform.homography_inv_3x3);
            }
            fs["tx"] >> transform.tx;
            fs["ty"] >> transform.ty;
            fs["scale_px_per_mm"] >> transform.scale_px_per_mm;
            fs["inlier_count"] >> transform.inlier_count;
            fs["inlier_ratio"] >> transform.inlier_ratio;
            fs["avg_error_px"] >> transform.avg_error_px;
            fs["avg_error_mm"] >> transform.avg_error_mm;
            fs.release();
            std::cout << "[ALIGNMENT] Caricati parametri allineamento e omografia da: " << yamlPath << std::endl;
        }
    }

    // 4. Parser del CSV dei componenti verificati AOI
    std::vector<VerifiedPcbComponent> inputComps;
    std::ifstream csvFile(verifiedCsvPath);
    if (!csvFile.is_open()) {
        std::cerr << "[ALIGNMENT] Impossibile aprire CSV: " << verifiedCsvPath << std::endl;
        return res;
    }

    std::string headerLine;
    std::getline(csvFile, headerLine);

    std::string line;
    while (std::getline(csvFile, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> cols;

        bool inQuotes = false;
        std::string cur;
        for (char ch : line) {
            if (ch == '\"') inQuotes = !inQuotes;
            else if (ch == ',' && !inQuotes) {
                cols.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(ch);
            }
        }
        cols.push_back(cur);

        if (cols.size() < 24) continue;

        VerifiedPcbComponent vc;
        vc.id = std::stoi(cols[0]);
        vc.ref_des = cols[1];
        std::string statusStr = cols[2];
        if (statusStr == "MATCH_EXACT") vc.status = AoiMatchStatus::MATCH_EXACT_CLASS;
        else if (statusStr == "CLASS_CORRECTED") vc.status = AoiMatchStatus::CLASS_CORRECTED;
        else if (statusStr == "RECOVERED_BY_AOI") vc.status = AoiMatchStatus::RECOVERED_BY_AOI;
        else vc.status = AoiMatchStatus::UNMATCHED_DETECTION;

        vc.label = cols[3];
        vc.original_label = cols[4];
        vc.confidence = std::stof(cols[5]);
        vc.source_name = cols[6];
        vc.center.x = std::stof(cols[7]);
        vc.center.y = std::stof(cols[8]);
        float w = std::stof(cols[9]);
        float h = std::stof(cols[10]);
        float ang = std::stof(cols[11]);
        vc.box = cv::Rect2f(vc.center.x - w * 0.5f, vc.center.y - h * 0.5f, w, h);
        vc.rotated_box = cv::RotatedRect(vc.center, cv::Size2f(w, h), ang);
        vc.radius = std::stof(cols[12]);
        vc.area = std::stof(cols[13]);
        if (cols.size() > 14 && !cols[14].empty()) {
            try { vc.nominal_x_mm = std::stod(cols[14]); } catch (...) {}
        }
        if (cols.size() > 15 && !cols[15].empty()) {
            try { vc.nominal_y_mm = std::stod(cols[15]); } catch (...) {}
        }
        vc.nominal_width_mm = std::stod(cols[16]);
        vc.nominal_height_mm = std::stod(cols[17]);
        vc.nominal_center_px.x = std::stof(cols[18]);
        vc.nominal_center_px.y = std::stof(cols[19]);
        vc.offset_error_px = std::stod(cols[20]);
        vc.offset_error_mm = std::stod(cols[21]);
        vc.aoi_part_number = cols[22];
        vc.aoi_outline = cols[23];
        if (cols.size() > 24) vc.notes = cols[24];
        if (cols.size() > 25 && !cols[25].empty()) {
            try { vc.datasheet_width_mm = std::stod(cols[25]); } catch (...) {}
        }
        if (cols.size() > 26 && !cols[26].empty()) {
            try { vc.datasheet_height_mm = std::stod(cols[26]); } catch (...) {}
        }
        if (cols.size() > 27 && !cols[27].empty()) {
            try { vc.datasheet_thickness_mm = std::stod(cols[27]); } catch (...) {}
        }
        if (cols.size() > 28) vc.datasheet_source = cols[28];
        if (cols.size() > 29 && !cols[29].empty()) {
            try { vc.nominal_rot_deg = std::stod(cols[29]); } catch (...) {}
        }

        inputComps.push_back(vc);
    }
    csvFile.close();

    // Riferimento prioritario: carica le dimensioni reali da component_cv_mapping.txt
    std::string mappingFilePath = (fs::path(paramsDir) / "component_cv_mapping.txt").string();
    auto dsMap = loadDatasheetMapping(mappingFilePath);
    for (auto& vc : inputComps) {
        if (vc.datasheet_width_mm <= 0.05) {
            auto itRef = dsMap.find("REF:" + vc.ref_des);
            if (itRef != dsMap.end() && itRef->second.w_mm > 0.05) {
                vc.datasheet_width_mm = itRef->second.w_mm;
                vc.datasheet_height_mm = itRef->second.h_mm;
                vc.datasheet_thickness_mm = itRef->second.thick_mm;
                vc.datasheet_source = itRef->second.source;
            } else {
                auto itCode = dsMap.find("CODE:" + vc.aoi_part_number);
                if (itCode != dsMap.end() && itCode->second.w_mm > 0.05) {
                    vc.datasheet_width_mm = itCode->second.w_mm;
                    vc.datasheet_height_mm = itCode->second.h_mm;
                    vc.datasheet_thickness_mm = itCode->second.thick_mm;
                    vc.datasheet_source = itCode->second.source;
                }
            }
        }
        if (vc.nominal_width_mm <= 0.05 && vc.datasheet_width_mm > 0.05) {
            vc.nominal_width_mm = vc.datasheet_width_mm;
            vc.nominal_height_mm = vc.datasheet_height_mm;
        }
    }

    std::cout << "[ALIGNMENT] Caricati " << inputComps.size() << " componenti verificati da: " << verifiedCsvPath << std::endl;

    // 5. Esegui la fusione Caliper + Template usando la sequenza multi-esposizione
    res = process(pcbImage, inputComps, transform, config, multiExposureImages);
    res.board_name = boardName;
    res.side = side;
    res.reference_image_name = fileStem;

    // 6. Salva tutti i risultati su disco
    fs::path outSideDir = fs::path(outputBaseDir) / boardName / side;
    saveResults(outSideDir.string(), fileStem, res);

    if (config.show_windows && !res.contours_vis.empty()) {
        std::string winName = "Contorni Allineati: " + boardName + " [" + side + "]";
        cv::namedWindow(winName, cv::WINDOW_NORMAL);
        cv::resizeWindow(winName, 1400, 950);
        cv::imshow(winName, res.contours_vis);
        std::cout << "\nPremi un tasto sulla finestra OpenCV dei contorni per continuare..." << std::endl;
        cv::waitKey(0);
        cv::destroyWindow(winName);
    }

    return res;
}

} // namespace pcb
