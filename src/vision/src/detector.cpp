#include <yaml-cpp/yaml.h>
#include <numeric>
#include "detector.hpp"
#include <cmath>
namespace auto_aim {
    Detector::Detector(const std::string & config_path, bool debug) : config_param_path(config_path), debug_(debug) {
        YAML::Node config = YAML::LoadFile(config_path);
        min_lightbar_ratio_ = config["min_lightbar_ratio"].as<double>();
        max_lightbar_ratio_ = config["max_lightbar_ratio"].as<double>();
        max_angle_error_ = config["max_angle_error"].as<double>();
        min_lightbar_length_ = config["min_lightbar_length"].as<double>();
        color_diff_thresh_ = config["color_diff_thresh"].as<int>();
        max_armor_ratio_ = config["max_armor_ratio"].as<double>();
        min_armor_ratio_ = config["min_armor_ratio"].as<double>();
        max_rectangular_error_ = config["max_rectangular_error"].as<double>();
        img_threshold_ = config["img_threshold"].as<double>();
        threshold_ = config["threshold"].as<double>();
        model_path_ = config["model_path"].as<std::string>();
        label_path_ = config["label_path"].as<std::string>();
        ignore_classes_ = config["ignore_classes"].as<std::vector<std::string>>();
        max_side_ratio_ = config["max_side_ratio"].as<double>();
        max_rectangular_error_diff_ = config["max_rectangular_error_diff"].as<double>();
        max_top_angle_ = config["max_top_angle"].as<double>();
        max_light_angle_ = config["max_light_angle"].as<double>();
        max_top_bottom_angle_ = config["max_top_bottom_angle"].as<double>();
        min_top_bottom_angle_ = config["min_top_bottom_angle"].as<double>();
    }

    std::vector<Armor> Detector::detect_armors(const cv::Mat &input_img, const Color &color) {
        std::vector<Armor> armors;
        cv::Mat binary_img = preprocess_image(input_img);
        std::vector<Lightbar> lightbars = find_lights(input_img, binary_img);
        if (lightbars.size() >= 2) {
            // for (auto &lightbar : lightbars) {
            //     cv::line(
            //             input_img, lightbar.top_middle, lightbar.bottom_middle, cv::Scalar(0, 255, 0), 10, cv::LINE_AA);
            // }
            armors = match_armors(lightbars, color);
            armors = entire_armor(input_img, armors);
        }
        if (debug_ == true) {

            show_result(binary_img, input_img, armors);
        }
        return armors;
    }


    cv::Mat Detector::preprocess_image(const cv::Mat &input_img) {
        cv::Mat binary_img;
        cv::cvtColor(input_img, gray_img_, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img_, binary_img, img_threshold_, 255, cv::THRESH_BINARY);
        return binary_img;
    }

    std::vector<Lightbar> Detector::find_lights(const cv::Mat &bgr_img, const cv::Mat &binary_img) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::vector<Lightbar> lightbars;
        for (const auto &contour : contours) {
            auto rotated_rect = cv::minAreaRect(contour);
            auto lightbar = Lightbar(rotated_rect);
            // std::cout << lightbar.ratio << " " << lightbar.length << " " << lightbar.angle_error << std::endl;
            if (!check_geometry(lightbar)) continue;
            lightbar.color = get_color(bgr_img, contour);
            // std::cout << lightbar.color << std::endl;
            if (lightbar.color == Color::extinguish) continue;
                lightbars.emplace_back(lightbar);
        }

        std::sort(lightbars.begin(), lightbars.end(),[](const Lightbar &l1, const Lightbar &l2) {
            return l1.center.x < l2.center.x;
        });
        // std::cout <<  lightbars.size() << "\n";
        return lightbars;
    }

    std::vector<Armor> Detector::match_armors(const std::vector<Lightbar> &lightbars, const Color &color) {
        std::vector<Armor> armors;
        for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
            if (left->color != color) continue;
            // double max_iter_width = left->length * armor_params.max_large_center_distance;
            for (auto right = std::next(left); right != lightbars.end(); right++) {
                if (right->color != color) continue;
                if (contain_ligthbar(left - lightbars.begin(), right - lightbars.begin(), lightbars)) continue;
                auto armor = Armor(*left, *right);
                if (!check_geometry(armor)) continue;
                 //std::cout << "YES" << "armor.top_bottom_angle_error " << armor.top_bottom_angle_error <<
                  //   " armor.top_angle_error " << armor.top_angle_error << " armor.light_angle_error " <<armor.light_angle_error << std::endl;
                armor.type = get_type(armor);
                // std::cout << "ratio:" << armor.ratio << std::endl;
                armors.emplace_back(armor);
            }
        }
        return armors;
    }

    std::vector<Armor> Detector::entire_armor(const cv::Mat &input_img, std::vector<Armor> &armors) {
        classifier = std::make_unique<NumberClassifier>(model_path_, label_path_, threshold_, ignore_classes_);
        corner_corrector = std::make_unique<LightCornerCorrector>();
        // std::cout << "armors_size:" << armors.size() << std::endl;
        if (!armors.empty() && classifier != nullptr) {
            std::for_each(
              std::execution::par, armors.begin(), armors.end(), [this, &input_img](Armor &armor) {
                  // std::cout << "classifier?" << std::endl;
                  armor.number_img = classifier->extract_number(input_img, armor);
                  classifier->classify(input_img, armor);
                  if (corner_corrector != nullptr) {
                      // std::cout << "corner_corrector?" << std::endl;
                      // corner_corrector->correctCorners(armor, gray_img_);
                  }
                  // lightbar_points_corrector(armor.left, gray_img_);
                  // lightbar_points_corrector(armor.right, gray_img_);

              });
            classifier->erase_ignore_classes(armors);
        }
        return armors;
    }

    bool Detector::contain_ligthbar(int i, int j, const std::vector<Lightbar> &lightbars) {
        const Lightbar &left = lightbars.at(i), right = lightbars.at(j);
        auto points = std::vector<cv::Point2f>{left.top_left, left.bottom_left, right.top_right, right.bottom_right};
        auto bounding_rect = cv::boundingRect(points);
        double avg_length = (left.length + right.length) / 2;
        double avg_width = (left.width + right.width) / 2;
        for (int k = i + 1; k < j; k++) {
            const Lightbar &test_lightbar = lightbars.at(k);
            if (test_lightbar.length > 2 * avg_length) continue;
            if (test_lightbar.width < 0.5 * avg_width) continue;
            if (bounding_rect.contains(test_lightbar.top_left) or bounding_rect.contains(test_lightbar.bottom_right) or bounding_rect.contains(test_lightbar.center)) {
                return true;
            }
        }

        return false;
    }

    bool Detector::check_geometry(const Lightbar &lightbar) {
        auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
        auto angle_ok = lightbar.angle_error < max_angle_error_;
        auto length_ok = lightbar.length > min_lightbar_length_;
        // std::cout << ratio_ok << " " << angle_ok <<  " " << length_ok << std::endl;
        return ratio_ok && angle_ok && length_ok;
    }

    bool Detector::check_geometry(const Armor &armor) {
        auto ratio_ok = armor.ratio < max_armor_ratio_ && armor.ratio > min_armor_ratio_;
        auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
        auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
        auto top_angle_ok = armor.top_angle_error < max_top_angle_;
        auto light_angle_error_ok = armor.light_angle_error < max_light_angle_;
        auto top_bottom_err_ok =  armor.top_bottom_angle_error < max_top_bottom_angle_ && armor.top_bottom_angle_error > min_top_bottom_angle_;
        if (std::isinf(armor.top_bottom_angle_error)) {
            top_bottom_err_ok = 1;
        }
        else if (std::isnan(armor.top_bottom_angle_error)) {
            top_bottom_err_ok = 1;
        }
        // std::cout <<"NOT" << "armor.ratio " << armor.ratio << " armor.side_ratio " << armor.side_ratio
        // << " armor.top_bottom_angle_error " << armor.top_bottom_angle_error <<
        //             " armor.top_angle_error " << armor.top_angle_error << " armor.light_angle_error " <<armor.light_angle_error << std::endl;
        // std::cout << max_light_angle_ << std::endl;
        // auto rectangular_error_diff_ok = armor.rectangular_error_diff < max_rectangular_error_diff_;
        // std::cout << "ratio_ok:" << ratio_ok << side_ratio_ok << rectangular_error_ok << std::endl;
        // std::cout << "ratio_ok:" << top_bottom_err_ok << light_angle_error_ok << top_angle_ok << std::endl;
        return ratio_ok && side_ratio_ok && rectangular_error_ok;
        // return  ratio_ok && side_ratio_ok && rectangular_error_ok && top_angle_ok && light_angle_error_ok && top_bottom_err_ok;
    }

    ArmorType Detector::get_type(const Armor &armor) {
        if (armor.ratio > 3.0) {
            return ArmorType::big;
        }
        if (armor.ratio < 2.5) {
            return ArmorType::small;
        }

        return ArmorType::small;

    }
    Color Detector::get_color(const cv::Mat &bgr_img, const std::vector<cv::Point> &contour) {
        int sum_red = 0, sum_blue = 0;
        for (const cv::Point &point : contour) {
            sum_blue += bgr_img.at<cv::Vec3b>(point)[0];
            sum_red += bgr_img.at<cv::Vec3b>(point)[2];
        }

        // if (sum_red < and sum_blue <) return Color::extinguish;
        if (std::abs(sum_red - sum_blue) / static_cast<int>(contour.size()) > color_diff_thresh_) {
            return sum_blue > sum_red ? Color::blue : Color::red;
        }
        return Color::extinguish;
    }

    void Detector::show_result(const cv::Mat & binary_img, const cv::Mat & input_img, std::vector<Armor> &armors) {
        draw_armor(input_img, armors);
        // cv::namedWindow("input_img", cv::WINDOW_NORMAL);

        // 2. 强制把窗口设为 640x480
        // cv::resizeWindow("input_img", 640, 480);
        cv::imshow("HikCamera", input_img);
        cv::imshow("binary_img", binary_img);
        cv::imshow("gray_img_", gray_img_);
    }

    void Detector::lightbar_points_corrector(Lightbar &lightbar, const cv::Mat &gray_img) {
        constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值
        constexpr float ROI_SCALE = 0.07;     // ROI扩展比例
        constexpr float SEARCH_START = 0.4;   // 搜索起始位置比例（原0.8/2）
        constexpr float SEARCH_END = 0.6;     // 搜索结束位置比例（原1.2/2）

        cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
        roi_box.x -= roi_box.width * ROI_SCALE;
        roi_box.y -= roi_box.height * ROI_SCALE;
        roi_box.width += 2 * roi_box.width * ROI_SCALE;
        roi_box.height += 2 * roi_box.height * ROI_SCALE;

        roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);
        cv::Mat roi = gray_img(roi_box);
        cv::imshow("roi", roi);
        const float mean_val = cv::mean(roi)[0];
        roi.convertTo(roi, CV_32F);
        cv::normalize(roi, roi, 0.0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

        const cv::Moments moments = cv::moments(roi);
        const cv::Point2f centroid(
          moments.m10 / moments.m00 + roi_box.x, moments.m01 / moments.m00 + roi_box.y);

        std::vector<cv::Point2f> points;
        for (int i = 0; i < roi.rows; ++i) {
            for (int j = 0; j < roi.cols; ++j) {
                const float weight = roi.at<float>(i, j);
                if (weight > 1e-3) {          // 忽略极小值提升性能
                    points.emplace_back(j, i);  // 坐标相对于ROI区域
                }
            }
        }

        cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        axis /= cv::norm(axis);
        if (axis.y > 0) axis = -axis;  // 统一方向
        const auto find_corner = [&](int direction) -> cv::Point2f {
            const float dx = axis.x * direction;
            const float dy = axis.y * direction;
            const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

            std::vector<cv::Point2f> candidates;

            // 横向采样多个候选线
            const int half_width = (lightbar.width - 2) / 2;
            for (int i_offset = -half_width; i_offset <= half_width; ++i_offset) {
                // 计算搜索起点
                cv::Point2f start_point(
                  centroid.x + lightbar.length * SEARCH_START * dx + i_offset,
                  centroid.y + lightbar.length * SEARCH_START * dy);

                // 沿轴搜索亮度跳变点
                cv::Point2f corner = start_point;
                float max_diff = 0;
                bool found = false;

                for (float step = 0; step < search_length; ++step) {
                    const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

                    // 边界检查
                    if (
                      cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 ||
                      cur_point.y >= gray_img.rows) {
                        break;
                      }
                    cv::Point2f prev_point = cur_point - cv::Point2f(dx, dy);

                    if (
                        prev_point.x < 0 || prev_point.x >= gray_img.cols ||
                        prev_point.y < 0 || prev_point.y >= gray_img.rows
                    ) {
                        break;
                    }
                    // 计算亮度差（使用双线性插值提升精度）
                    const auto prev_val = gray_img.at<uchar>(cv::Point2i(prev_point));
                    const auto cur_val = gray_img.at<uchar>(cv::Point2i(cur_point));
                    const float diff = prev_val - cur_val;

                    if (diff > max_diff && prev_val > mean_val) {
                        max_diff = diff;
                        corner = cur_point - cv::Point2f(dx, dy);  // 跳变发生在上一位置
                        found = true;
                    }
                }

                if (found) {
                    candidates.push_back(corner);
                }
            }

            // 返回候选点均值
            return candidates.empty()
                     ? cv::Point2f(-1, -1)
                     : std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) /
                         static_cast<float>(candidates.size());
        };
        lightbar.top_middle = find_corner(1);
        lightbar.bottom_middle = find_corner(-1);

    }
    void Detector::draw_armor(const cv::Mat &input_img,  std::vector<Armor> &armors) {
        if (!armors.empty()) {
            for (const auto&armor : armors) {
                cv::line(
                    input_img, armor.left.top_middle, armor.left.bottom_middle, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(
                    input_img, armor.right.top_middle, armor.right.bottom_middle, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(
                    input_img, armor.left.top_middle, armor.right.top_middle, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::line(
                    input_img, armor.left.bottom_middle, armor.right.bottom_middle, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::Mat resized;
                cv::resize(armor.number_img, resized, cv::Size (128, 128));
                cv::imshow("number_img", resized);
            }
        }

    }

}