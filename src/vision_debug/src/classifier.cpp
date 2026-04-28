#include <algorithm>
#include <cstddef>
#include <execution>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <vector>
#include "classifier.hpp"
namespace auto_aim {
    NumberClassifier::NumberClassifier(const std::string &model_path, const std::string &label_path,
        double thre, const std::vector<std::string> &ignore_classes): threshold(thre), ignore_classes_(ignore_classes) {
        net_ = cv::dnn::readNetFromONNX(model_path);
        std::ifstream ifs(label_path);
        std::string line;
        while (std::getline(ifs, line)) {
            class_names_.push_back(line);
        }
    }

    cv::Mat NumberClassifier::extract_number(const cv::Mat &src, const ::auto_aim::Armor &armor) {
        static const int light_length = 12;
        // 透视变换后的图像尺寸。
        static const int warp_height = 28;
        static const int small_armor_width = 32;
        static const int large_armor_width = 54;
        // 数字区域和模型输入尺寸。
        static const cv::Size roi_size(20, 28);
        static const cv::Size input_size(28, 28);
        cv::Point2f lights_vertices[4] = {
            armor.left.bottom_middle, armor.left.top_middle, armor.right.top_middle, armor.right.bottom_middle};
        const int top_light_y = (warp_height - light_length) / 2 - 1;
        const int bottom_light_y = top_light_y + light_length;
        const int warp_width = armor.type == ArmorType::small ? small_armor_width : large_armor_width;
        cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(0, top_light_y),
            cv::Point(warp_width - 1, top_light_y),
            cv::Point(warp_width - 1, bottom_light_y),
          };
        cv::Mat number_img;
        auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
        cv::warpPerspective(src, number_img, rotation_matrix, cv::Size(warp_width, warp_height));
        number_img = number_img(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
        cv::cvtColor(number_img, number_img, cv::COLOR_RGB2GRAY);
        cv::threshold(number_img, number_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::resize(number_img, number_img, input_size);
        return number_img;
    }

    void NumberClassifier::classify(const cv::Mat &src, Armor &armor) {
        // 归一化后送入 ONNX 数字分类模型。
        cv::Mat input = armor.number_img / 255.0;

        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);

        mutex_.lock();
        net_.setInput(blob);

        cv::Mat outputs = net_.forward().clone();
        mutex_.unlock();

        double confidence;
        cv::Point class_id_point;
        minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.name = static_cast<ArmorName>(label_id);
    }

    void NumberClassifier::erase_ignore_classes(std::vector<Armor> &armors) {
        armors.erase(
          std::remove_if(armors.begin(),
                         armors.end(),
                         [this](const Armor &armor) {
                               if (armor.confidence < threshold) {
                                 return true;
                               }

                           for (const auto &ignore_class : ignore_classes_) {
                               auto it = std::find(class_names_.begin(),class_names_.end(),ignore_class);
                               if (it == class_names_.end()) continue;
                               if (armor.name == std::distance(class_names_.begin(), it)) {
                                   return true;
                               }
                            }

                               bool mismatch_armor_type = false;
                               if (armor.type == ArmorType::big) {
                                 mismatch_armor_type = armor.name == ArmorName::outpost || armor.name == ArmorName::two ||
                                                       armor.name == ArmorName::sentry;
                               } else if (armor.type == ArmorType::small) {
                                 mismatch_armor_type = armor.name == ArmorName::one || armor.name == ArmorName::base;
                               }
                               return mismatch_armor_type;
                            } ),
          armors.end());
    }


}
