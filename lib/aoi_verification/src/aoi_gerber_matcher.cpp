#include "aoi_internal.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace pcb {

void AoiVerificationProcessor::getNominalOutlineDimensions(const std::string& outline, double& width_mm, double& height_mm) {
    std::string uOut = toUpper(outline);

    if (uOut.find("0603") != std::string::npos) {
        width_mm = 1.6; height_mm = 0.8;
    } else if (uOut.find("0805") != std::string::npos) {
        width_mm = 2.0; height_mm = 1.25;
    } else if (uOut.find("1206") != std::string::npos) {
        width_mm = 3.2; height_mm = 1.6;
    } else if (uOut.find("1210") != std::string::npos) {
        width_mm = 3.2; height_mm = 2.5;
    } else if (uOut.find("1812") != std::string::npos) {
        width_mm = 4.5; height_mm = 3.2;
    } else if (uOut.find("2512") != std::string::npos) {
        width_mm = 6.3; height_mm = 3.2;
    } else if (uOut.find("SOT23") != std::string::npos || uOut.find("TO236") != std::string::npos) {
        width_mm = 2.9; height_mm = 2.4;
    } else if (uOut.find("SOT553") != std::string::npos) {
        width_mm = 1.6; height_mm = 1.6;
    } else if (uOut.find("SOT89") != std::string::npos) {
        width_mm = 4.5; height_mm = 2.5;
    } else if (uOut.find("SOT685") != std::string::npos || uOut.find("SOT1212") != std::string::npos) {
        width_mm = 5.0; height_mm = 6.0;
    } else if (uOut.find("MSOP8") != std::string::npos || uOut.find("MSOP16") != std::string::npos) {
        width_mm = 3.0; height_mm = 4.9;
    } else if (uOut.find("D2PAK") != std::string::npos || uOut.find("TO-263") != std::string::npos) {
        width_mm = 10.0; height_mm = 15.0;
    } else if (uOut.find("SC79") != std::string::npos) {
        width_mm = 1.6; height_mm = 0.8;
    } else if (uOut.find("DO-214") != std::string::npos || uOut.find("DO214") != std::string::npos) {
        width_mm = 5.4; height_mm = 3.6;
    } else if (uOut.find("CHUUT") != std::string::npos) {
        width_mm = 10.0; height_mm = 10.0;
    } else if (uOut.find("RHMS1") != std::string::npos) {
        width_mm = 3.6; height_mm = 1.5;
    } else if (uOut.find("SON6") != std::string::npos) {
        width_mm = 2.0; height_mm = 2.0;
    } else if (uOut.find("SO4") != std::string::npos) {
        width_mm = 4.6; height_mm = 4.6;
    } else if (uOut.find("LH4") != std::string::npos || uOut.find("LH6") != std::string::npos) {
        width_mm = 7.0; height_mm = 7.0;
    } else {
        width_mm = 2.0; height_mm = 1.25;
    }
}

} // namespace pcb

