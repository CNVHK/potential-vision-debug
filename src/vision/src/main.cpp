#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fmt/chrono.h>
#include "detector.hpp"
#include "armor.hpp"
#include <sstream>
#include <memory>

#include "tracker.hpp"
#include "solver.hpp"
#include "aimer.hpp"
#include "shooter.hpp"
#include <stdio.h>
#include <iostream>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
void draw_points(
  cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
    std::vector<std::vector<cv::Point>> contours = {points};
    cv::drawContours(img, contours, -1, color, thickness);
}

void draw_points(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
{
    std::vector<cv::Point> int_points(points.begin(), points.end());
    draw_points(img, int_points, color, thickness);
}

int main() {
    int nRet = MV_OK;
    void* handle = NULL;

    // 1. 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return -1;
    }

    if (stDeviceList.nDeviceNum > 0) {
        // 默认选择第一个设备
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];
        printf("Found Device: [%s]\n", pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);

        // 2. 创建句柄
        nRet = MV_CC_CreateHandle(&handle, pDeviceInfo);
        if (MV_OK != nRet) return -1;

        // 3. 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet) return -1;

    // 可选：GigE 相机设置网络包大小，降低丢包概率。
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0) {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
            }
        }

        // 4. 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet) return -1;
        cv::namedWindow("HikCamera", cv::WINDOW_NORMAL);

        // 2. 强制把窗口设为 640x480
        cv::resizeWindow("HikCamera", 640, 480);
        // 5. 获取图像循环 vv
        MV_FRAME_OUT stOutFrame = {0};
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
        std::string config_path = "D:\\potential_vision\\vision_params.yaml";
        auto_aim::Detector detector(config_path, false);
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path);
        auto_aim::Aimer aimer(config_path);
        std::chrono::steady_clock::time_point t;
        while (true) {
            // 超时时间 1000ms
            double vyaw;
            double vx;
            double vy;
            nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
            if (nRet == MV_OK) {
                MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
                stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth;
                stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight;
                stConvertParam.pSrcData = stOutFrame.pBufAddr;
                stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;
                stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; // 转为 BGR 供 OpenCV 使用
                cv::Mat srcImage;
                srcImage.create(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC3);
                stConvertParam.pDstBuffer = srcImage.data;
                stConvertParam.nDstBufferSize = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3;

                // 执行转换
                nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
                if (MV_OK != nRet) {
                    printf("Convert Pixel Fail! nRet [0x%x]\n", nRet);
                    continue;
                }
                auto t = std::chrono::steady_clock::now();
                auto armors = detector.detect_armors(srcImage, auto_aim::Color::blue);
                armors.erase(
                    std::remove_if(
                        armors.begin(), armors.end(),
                        [](const auto_aim::Armor & armor) { return armor.color != auto_aim::Color::blue; }),
                    armors.end());
                std::sort(armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
                    cv::Point2f img_center(1440 / 2, 1080 / 2);
                    auto distance_1 = cv::norm(a.center - img_center);
                    auto distance_2 = cv::norm(b.center - img_center);
                    return distance_1 < distance_2;
                });
                std::stable_sort(armors.begin(), armors.end(), [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
                    return a.priority < b.priority;
                });
                if (!armors.empty()) {
                    if (tracker.needs_target_initialization(t)) {
                        solver.solve(armors.front());
                    } else {
                        auto identity = tracker.tracked_armor_identity();
                        if (identity.has_value()) {
                            for (auto & armor : armors) {
                                if (armor.name == identity->name && armor.type == identity->type) {
                                    solver.solve(armor);
                                }
                            }
                        }
                    }
                }

                auto targets = tracker.track(armors, t, auto_aim::Color::blue, false);
                auto command = aimer.aim(targets, t, 25, solver.R_gimbal2world());
                if (!targets.empty()) {
                    if (!armors.empty()) {
                        auto armor = armors.front();
                        draw_points(srcImage, armor.points, {0, 0, 255}, 3);
                        auto target = targets.front();
                        auto ekf_x = target.ekf_x();
                        vx = ekf_x[1];
                        vy = ekf_x[3];
                        vyaw = ekf_x[7];
                        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
                        auto min_error = 1e10;
                        Eigen::Vector4d detector_xyza;
                        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                            auto yaw_error = std::abs(armor.ypr_in_world[0] - xyza[3]);
                            if (yaw_error < min_error) {
                                min_error = yaw_error;
                                detector_xyza = xyza;
                            }
                        }
                        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                            if (xyza == detector_xyza) continue;
                            auto image_points =
                              solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                            draw_points(srcImage, image_points, {255, 255, 0}, 3);
                        }
                    }
                }
                auto img_gimbal_info = fmt::format("gimbal_yaw: {:.1f}, gimbal_pitch: {:.1f}", command.yaw * 180 / CV_2PI, command.pitch * 180 / CV_2PI);
                auto img_robot_info = fmt::format("rpm: {:.1f}, vx: {:.2f}, vy: {:.2f}", vyaw, vx, vy);
                cv::putText(srcImage, img_gimbal_info, cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 2.0,cv::Scalar(0, 255, 255), 3);
                cv::putText(srcImage, img_robot_info, cv::Point(20, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 2.0,cv::Scalar(0, 255, 255), 3);
                cv::imshow("HikCamera", srcImage);
                if (cv::waitKey(1) == 27) break; // 按 ESC 退出
                MV_CC_FreeImageBuffer(handle, &stOutFrame);
            }
        }
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
    } else {
        printf("No device found.\n");
    }
    return 0;
}
