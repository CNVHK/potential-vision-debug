#ifndef POTENTIAL_VISION_DETECTOR_HPP
#define POTENTIAL_VISION_DETECTOR_HPP
#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <execution>
#include <algorithm>
#include "classifier.hpp"
#include "light_corner_corrector.hpp"
#include "armor.hpp"

namespace auto_aim {
    class Detector {
    public:
        Detector(const std::string & config_path, bool debug = true);
        std::vector<Armor> detect_armors(const cv::Mat &input_img, const Color &color);
        cv::Mat preprocess_image(const cv::Mat &input_img);
        // std::list<Armor> detector();
        std::unique_ptr<NumberClassifier> classifier;
        std::unique_ptr<LightCornerCorrector> corner_corrector;
        Color detect_color;
        std::string config_param_path;
    private:
        double min_lightbar_ratio_;
        double max_lightbar_ratio_;
        double max_angle_error_;
        double min_lightbar_length_;
        double img_threshold_;
        double threshold_;
        double max_armor_ratio_;
        double min_armor_ratio_;
        double max_side_ratio_;
        double max_rectangular_error_;
        double max_rectangular_error_diff_;
        double max_top_angle_;
        double max_light_angle_;
        double max_top_bottom_angle_;
        double min_top_bottom_angle_;
        std::string model_path_;
        std::string label_path_;
        std::vector<std::string> ignore_classes_;
        int color_diff_thresh_;
        bool debug_;
        cv::Mat gray_img_;
        std::vector<Lightbar> find_lights(const cv::Mat &bgr_img, const cv::Mat &binary_img);
        std::vector<Armor> match_armors(const std::vector<Lightbar> &lightbars, const Color &color);
        std::vector<Armor> entire_armor(const cv::Mat &input_img, std::vector<Armor> &armors);
        Color get_color(const cv::Mat &bgr_img, const std::vector<cv::Point> &contour);
        bool contain_ligthbar(int i, int j, const std::vector<Lightbar> &lightbars);
        bool check_geometry(const Lightbar &lightbar);
        bool check_geometry(const Armor &armor);
        ArmorType get_type(const Armor &armor);
        void show_result(const cv::Mat & binary_img, const cv::Mat & input_img, std::vector<Armor> &armors);
        void draw_armor(const cv::Mat &input_img,  std::vector<Armor> &armors);
        void lightbar_points_corrector(Lightbar &lightbar, const cv::Mat &gray_img);
    };
}
#endif //POTENTIAL_VISION_DETECTOR_HPP