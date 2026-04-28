
#ifndef POTENTIAL_VISION_CLASSIFIER_HPP
#define POTENTIAL_VISION_CLASSIFIER_HPP

#endif //POTENTIAL_VISION_CLASSIFIER_HPP
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "armor.hpp"

namespace auto_aim {
    class NumberClassifier {
    public:
        NumberClassifier(const std::string & model_path, const std::string & label_path, double threshold,
                   const std::vector<std::string> &ignore_classes = {} );
        cv::Mat extract_number(const cv::Mat &src, const Armor &armor);
        void classify(const cv::Mat &src, Armor &armor);
        void erase_ignore_classes(std::vector<Armor> &armors);

        double threshold;


    private:
        std::mutex mutex_;
        cv::dnn::Net net_;
        std::vector<std::string> class_names_;
        std::vector<std::string> ignore_classes_;

    };
}
