#ifndef POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP
#define POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP


#include <opencv2/opencv.hpp>
#include "armor.hpp"

namespace auto_aim {

    struct SymmetryAxis {
        cv::Point2f centroid;
        cv::Point2f direction;
        float mean_val; // 平均亮度
    };

    // 使用 PCA 和亮度梯度修正灯条端点，提高装甲板角点精度。
    class LightCornerCorrector {
    public:
        explicit LightCornerCorrector() noexcept {}

        // 修正装甲板两侧灯条的上下端点。
        void correctCorners(Armor &armor, const cv::Mat &gray_img);

    private:
        // 寻找灯条亮度分布的对称轴。
        SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const Lightbar &light);

        // 沿对称轴寻找灯条端点。
        cv::Point2f findCorner(const cv::Mat &gray_img,
                               const Lightbar &light,
                               const SymmetryAxis &axis,
                               std::string order);
    };

}  // namespace auto_aim
#endif //POTENTIAL_VISION_LIGHT_CORNER_CORRECTOR_HPP
