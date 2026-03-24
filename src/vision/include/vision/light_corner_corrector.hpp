//
// Created by yuanshangkun on 2026/1/1.
//

#ifndef POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP
#define POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP


#include <opencv2/opencv.hpp>
// project
#include "armor.hpp"

namespace auto_aim {

    struct SymmetryAxis {
        cv::Point2f centroid;
        cv::Point2f direction;
        float mean_val; // Mean brightness
    };

    // This class is used to improve the precision of the corner points of the light bar.
    // First, the PCA algorithm is used to find the symmetry axis of the light bar,
    // and then along the symmetry axis to find the corner points of the light bar based on the gradient of brightness.
    class LightCornerCorrector {
    public:
        explicit LightCornerCorrector() noexcept {}

        // Correct the corners of the armor's lights
        void correctCorners(Armor &armor, const cv::Mat &gray_img);

    private:
        // Find the symmetry axis of the light
        SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const Lightbar &light);

        // Find the corner of the light
        cv::Point2f findCorner(const cv::Mat &gray_img,
                               const Lightbar &light,
                               const SymmetryAxis &axis,
                               std::string order);
    };

}  // namespace fyt::auto_aim
#endif //POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP
