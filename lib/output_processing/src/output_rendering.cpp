#include "output_internal.hpp"
#include <algorithm>

namespace pcb {

cv::Mat OutputProcessor::createSideLegendPanel(
    const std::vector<std::pair<std::string, int>>& label_counts,
    int total_count,
    const std::map<std::string, cv::Scalar>& palette,
    int target_height,
    const std::string& title,
    const std::vector<std::pair<std::string, int>>& extra_stats) {

    return pcb::common::renderSideLegend(
        target_height, label_counts, total_count, palette, title, extra_stats, 440);
}

// =========================================================================
// Rendering Grafico Passaggi Intermedi e Finali
// =========================================================================
cv::Mat OutputProcessor::renderStep1Elements(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        cv::Rect r(static_cast<int>(std::round(c.box.x)),
                   static_cast<int>(std::round(c.box.y)),
                   static_cast<int>(std::round(c.box.width)),
                   static_cast<int>(std::round(c.box.height)));
        r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (r.area() <= 0) continue;

        cv::Mat roi = canvas(r);
        cv::Mat overlay = roi.clone();
        overlay.setTo(COLOR_ELEMENT);
        cv::addWeighted(overlay, 0.28, roi, 0.72, 0.0, roi);

        cv::rectangle(canvas, r, COLOR_ELEMENT, 2);
    }

    // Creiamo il pannello legenda laterale
    std::vector<std::pair<std::string, int>> dummy;
    std::map<std::string, cv::Scalar> dummy_pal;
    cv::Mat legend = createSideLegendPanel(dummy, static_cast<int>(comps.size()), dummy_pal, canvas.rows, "Step 1: WHERE (DL)");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep1Types(const cv::Mat& base_img, const std::vector<FusedDlComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::map<std::string, int> counts;
    for (const auto& c : comps) {
        counts[c.label]++;
    }

    std::vector<std::string> all_labels;
    for (const auto& [lbl, _] : counts) all_labels.push_back(lbl);
    auto palette = pcb::common::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Rect r(static_cast<int>(std::round(c.box.x)),
                   static_cast<int>(std::round(c.box.y)),
                   static_cast<int>(std::round(c.box.width)),
                   static_cast<int>(std::round(c.box.height)));
        r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (r.area() <= 0) continue;

        cv::Scalar col = cv::Scalar(180, 180, 180);
        auto it = palette.find(c.label);
        if (it != palette.end()) col = it->second;

        cv::rectangle(canvas, r, col, 2);

        // Disegna etichetta (codice breve)
        std::string short_lbl = pcb::common::getShortLabel(c.label);
        int baseLine = 0;
        double font_scale = 0.52;
        cv::Size sz = cv::getTextSize(short_lbl, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseLine);
        cv::Point txt_org(r.x, std::max(18, r.y - 4));
        cv::rectangle(canvas, cv::Rect(txt_org.x, txt_org.y - sz.height - 2, sz.width + 4, sz.height + 4), cv::Scalar(15, 15, 15), cv::FILLED);
        cv::putText(canvas, short_lbl, cv::Point(txt_org.x + 2, txt_org.y - 1), cv::FONT_HERSHEY_SIMPLEX, font_scale, col, 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows, "Classi DL Fuse");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep2CvElements(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        if (c.cv_geom.shape == ShapeType::CIRCLE) {
            cv::circle(canvas, c.cv_geom.center, static_cast<int>(std::round(c.cv_geom.radius)), COLOR_CV_ONLY, 2, cv::LINE_AA);
            cv::circle(canvas, c.cv_geom.center, 2, COLOR_CV_ONLY, -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.cv_geom.rotated_box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(canvas, pts[i], pts[(i + 1) % 4], COLOR_CV_ONLY, 2, cv::LINE_AA);
            }
        }
    }

    std::vector<std::pair<std::string, int>> dummy;
    std::map<std::string, cv::Scalar> dummy_pal;
    cv::Mat legend = createSideLegendPanel(dummy, static_cast<int>(comps.size()), dummy_pal, canvas.rows, "Step 2: CV Geometria");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep2CvTypes(const cv::Mat& base_img, const std::vector<ClassifiedCvComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::map<std::string, int> counts;
    for (const auto& c : comps) {
        counts[c.label]++;
    }

    std::vector<std::string> all_labels;
    for (const auto& [lbl, _] : counts) all_labels.push_back(lbl);
    auto palette = pcb::common::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Scalar col = cv::Scalar(180, 180, 180);
        auto it = palette.find(c.label);
        if (it != palette.end()) col = it->second;

        if (c.cv_geom.shape == ShapeType::CIRCLE) {
            cv::circle(canvas, c.cv_geom.center, static_cast<int>(std::round(c.cv_geom.radius)), col, 2, cv::LINE_AA);
            cv::circle(canvas, c.cv_geom.center, 2, col, -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.cv_geom.rotated_box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(canvas, pts[i], pts[(i + 1) % 4], col, 2, cv::LINE_AA);
            }
        }

        std::string short_lbl = pcb::common::getShortLabel(c.label);
        cv::Point txt_org(static_cast<int>(c.cv_geom.center.x), std::max(16, static_cast<int>(c.cv_geom.center.y - 6)));
        int baseLine = 0;
        double font_scale = 0.50;
        cv::Size sz = cv::getTextSize(short_lbl, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseLine);
        cv::rectangle(canvas, cv::Rect(txt_org.x, txt_org.y - sz.height - 2, sz.width + 4, sz.height + 4), cv::Scalar(15, 15, 15), cv::FILLED);
        cv::putText(canvas, short_lbl, cv::Point(txt_org.x + 2, txt_org.y - 1), cv::FONT_HERSHEY_SIMPLEX, font_scale, col, 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows, "Classi CV (DL)");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep3FinalElements(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    for (const auto& c : comps) {
        cv::Rect r(static_cast<int>(std::round(c.box.x)),
                   static_cast<int>(std::round(c.box.y)),
                   static_cast<int>(std::round(c.box.width)),
                   static_cast<int>(std::round(c.box.height)));
        r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
        if (r.area() <= 0) continue;

        cv::Mat roi = canvas(r);
        cv::Mat overlay = roi.clone();
        overlay.setTo(COLOR_ELEMENT);
        cv::addWeighted(overlay, 0.28, roi, 0.72, 0.0, roi);

        cv::rectangle(canvas, r, COLOR_ELEMENT, 2);
    }

    std::vector<std::pair<std::string, int>> dummy;
    std::map<std::string, cv::Scalar> dummy_pal;
    cv::Mat legend = createSideLegendPanel(dummy, static_cast<int>(comps.size()), dummy_pal, canvas.rows, "Step 3: Componenti Totali");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep3FinalTypes(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::map<std::string, int> counts;
    std::map<std::string, int> source_counts;
    for (const auto& c : comps) {
        counts[c.label]++;
        source_counts[sourceToString(c.source)]++;
    }

    std::vector<std::string> all_labels;
    for (const auto& [lbl, _] : counts) all_labels.push_back(lbl);
    auto palette = pcb::common::getPalette(all_labels);

    for (const auto& c : comps) {
        cv::Scalar col = cv::Scalar(180, 180, 180);
        auto it = palette.find(c.label);
        if (it != palette.end()) col = it->second;

        if (c.shape == ShapeType::CIRCLE) {
            cv::circle(canvas, c.center, static_cast<int>(std::round(c.radius)), col, 2, cv::LINE_AA);
            cv::circle(canvas, c.center, 2, col, -1, cv::LINE_AA);
        } else if (std::abs(c.rotated_box.angle) > 1.0f && c.rotated_box.size.width > 2.0f) {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(canvas, pts[i], pts[(i + 1) % 4], col, 2, cv::LINE_AA);
            }
        } else {
            cv::Rect r(static_cast<int>(std::round(c.box.x)),
                       static_cast<int>(std::round(c.box.y)),
                       static_cast<int>(std::round(c.box.width)),
                       static_cast<int>(std::round(c.box.height)));
            r &= cv::Rect(0, 0, canvas.cols, canvas.rows);
            if (r.area() > 0) {
                cv::rectangle(canvas, r, col, 2);
            }
        }

        std::string short_lbl = pcb::common::getShortLabel(c.label);
        cv::Point txt_org(static_cast<int>(c.center.x), std::max(16, static_cast<int>(c.center.y - 6)));
        int baseLine = 0;
        double font_scale = 0.52;
        cv::Size sz = cv::getTextSize(short_lbl, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseLine);
        cv::rectangle(canvas, cv::Rect(txt_org.x, txt_org.y - sz.height - 2, sz.width + 4, sz.height + 4), cv::Scalar(15, 15, 15), cv::FILLED);
        cv::putText(canvas, short_lbl, cv::Point(txt_org.x + 2, txt_org.y - 1), cv::FONT_HERSHEY_SIMPLEX, font_scale, col, 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> sorted_counts(counts.begin(), counts.end());
    std::sort(sorted_counts.begin(), sorted_counts.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });

    std::vector<std::pair<std::string, int>> stats;
    for (const auto& [src, cnt] : source_counts) {
        stats.emplace_back(src, cnt);
    }

    cv::Mat legend = createSideLegendPanel(sorted_counts, static_cast<int>(comps.size()), palette, canvas.rows,
                                           "Risultato Finale Unificato", stats);

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

cv::Mat OutputProcessor::renderStep3Geometric(const cv::Mat& base_img, const std::vector<FinalComponent>& comps) {
    if (base_img.empty()) return {};
    cv::Mat canvas = base_img.clone();

    std::map<std::string, int> source_counts;

    for (const auto& c : comps) {
        cv::Scalar col = COLOR_DL_ONLY;
        if (c.source == ComponentSource::FUSED_DL_CV) col = COLOR_FUSED;
        else if (c.source == ComponentSource::CV_ONLY) col = COLOR_CV_ONLY;
        else if (c.source == ComponentSource::SIMILARITY_DISCOVERED) col = COLOR_SIMILARITY;

        source_counts[sourceToString(c.source)]++;

        if (c.shape == ShapeType::CIRCLE) {
            cv::circle(canvas, c.center, static_cast<int>(std::round(c.radius)), col, 2, cv::LINE_AA);
            cv::circle(canvas, c.center, 3, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        } else {
            cv::Point2f pts[4];
            c.rotated_box.points(pts);
            for (int i = 0; i < 4; ++i) {
                cv::line(canvas, pts[i], pts[(i + 1) % 4], col, 2, cv::LINE_AA);
            }
            cv::circle(canvas, c.center, 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        }

        std::string tag = "#" + std::to_string(c.id);
        cv::putText(canvas, tag, cv::Point(static_cast<int>(c.center.x + 4), static_cast<int>(c.center.y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        cv::putText(canvas, tag, cv::Point(static_cast<int>(c.center.x + 4), static_cast<int>(c.center.y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    std::vector<std::pair<std::string, int>> stats;
    for (const auto& [src, cnt] : source_counts) {
        stats.emplace_back(src, cnt);
    }

    std::map<std::string, cv::Scalar> pal = {
        {"FUSED_DL_CV", COLOR_FUSED},
        {"DL_ONLY", COLOR_DL_ONLY},
        {"CV_ONLY", COLOR_CV_ONLY},
        {"SIMILARITY_DISCOVERED", COLOR_SIMILARITY}
    };

    cv::Mat legend = createSideLegendPanel(stats, static_cast<int>(comps.size()), pal, canvas.rows, "Fonti Rilevamento");

    cv::Mat combined;
    cv::hconcat(canvas, legend, combined);
    return combined;
}

} // namespace pcb

