#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include "detector.hpp"
#include "number_classifier.hpp"

using namespace std;

namespace rm_auto_aim{
    Detector::Detector(
    const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
    : binary_thres(bin_thres), detect_color(color), l(l), a(a)
    {
    }

    std::vector<Armor> Detector::detect(const cv::Mat & input)
    {
        binary_img = preprocessImage(input);
        lights_ = findLights(input, binary_img);
        armors_ = matchLights(lights_);
        // if (armors_.size() == 0) {
        // std::cout << "Vector is empty" << std::endl;
        // } else {
        //     std::cout << "Vector is not empty" << std::endl;
        // }
        cv::namedWindow("binary_img", cv::WINDOW_NORMAL);
        cv::imshow("binary_img", binary_img);
        // 创建数字识别对象
        std::string model_path = "D:/Progamming/cv/model/mlp.onnx";
        std::string label_path = "D:/Progamming/cv/model/label.txt";
        double threshold = 0.7;
        std::vector<std::string> ignore_classes = {"negative"};
        rm_auto_aim::NumberClassifier numberClassifier(model_path, label_path, threshold,ignore_classes);
        if (!armors_.empty()) {
            numberClassifier.extractNumbers(input, armors_);
            numberClassifier.classify(armors_);
        }
        return armors_;
    }

    cv::Mat Detector::preprocessImage(const cv::Mat& rgb_img) {
        // 创建一个与输入图像相同大小和类型的输出图像
        cv::Mat gray_img;
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
        // cv::namedWindow("gray_img", cv::WINDOW_NORMAL);
        // cv::imshow("gray_img", gray_img);
        cv::Mat binary_img;
        cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);
        // cv::namedWindow("binary_img", cv::WINDOW_NORMAL);
        // cv::imshow("binary_img", binary_img);
        return binary_img;
    }

    std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
    {
        /*函数功能：寻找符合条件的灯条
            input:
            rbg_img 判断颜色
            binary_img 寻找符合灯条
            output:
            light 符合条件的灯条及灯条颜色
        */
        using std::vector;
        vector<vector<cv::Point>> contours; // 存储轮廓

        vector<cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        vector<Light> lights;
        for (const auto & contour : contours) {
            if (contour.size() < 5) continue;
            auto r_rect = cv::minAreaRect(contour);
            auto light = Light(r_rect);
            
            // if (light.top.x != 0.0f || light.top.y != 0.0f) {
            //     // light 对象有值
            //     cout << "0" << endl;
            // } else {
            //     // light 对象为空
            //     cout << "1" << endl;
            // }
            //-----------判断灯条颜色--------------
            if (isLight(light)) {
                // cout << "in" << endl;
                auto rect = light.boundingRect();// 光源外接矩阵
                if (  // Avoid assertion failed
                    0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
                    0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
                    int sum_r = 0, sum_b = 0;
                    auto roi = rbg_img(rect);
                    // Iterate through the ROI
                    for (int i = 0; i < roi.rows; i++) {
                    for (int j = 0; j < roi.cols; j++) {
                        if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
                        // if point is inside contour
                        sum_b += roi.at<cv::Vec3b>(i, j)[0];
                        sum_r += roi.at<cv::Vec3b>(i, j)[2];
                        }
                    }
                    }
                    // Sum of red pixels > sum of blue pixels ?
                    light.color = sum_r > sum_b ? RED : BLUE;
                    lights.emplace_back(light);
                }
            }
        }
    return lights;
    }//find_lights

    bool Detector::isLight(const Light & light)
    {
        // The ratio of light (short side / long side)
        float ratio = light.width / light.length;
        bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;
        bool angle_ok = light.tilt_angle < l.max_angle;
        bool is_light = ratio_ok && angle_ok;

        //--------仿真界面debug------------
        // Fill in debug information
        // auto_aim_interfaces::msg::DebugLight light_data;
        // light_data.center_x = light.center.x;
        // light_data.ratio = ratio;
        // light_data.angle = light.tilt_angle;
        // light_data.is_light = is_light;
        // this->debug_lights.data.emplace_back(light_data);

        return is_light;
    }

    std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
    {
        std::vector<Armor> armors;
        // this->debug_armors.data.clear();

        // Loop all the pairing of lights
        for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
            for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_1->color != detect_color || light_2->color != detect_color) continue;

            if (containLight(*light_1, *light_2, lights)) {
                continue;
            }

            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
            }
        }
        return armors;
    }
    
    // Check if there is another light in the boundingRect formed by the 2 lights
    bool Detector::containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
    {
        auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
        auto bounding_rect = cv::boundingRect(points);

        for (const auto & test_light : lights) {
            if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

            if (
            bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            // cout << "***" << endl;
            return true;
            }
        }
        return false;
    }

    ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
    {
        // Ratio of the length of 2 lights (short side / long side)
        float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                                    : light_2.length / light_1.length;
        bool light_ratio_ok = light_length_ratio > a.min_light_ratio;
        
        // Distance between the center of 2 lights (unit : light length)
        float avg_light_length = (light_1.length + light_2.length) / 2;
        float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
        bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                                    center_distance < a.max_small_center_distance) ||
                                    (a.min_large_center_distance <= center_distance &&
                                    center_distance < a.max_large_center_distance);

        // Angle of light center connection
        cv::Point2f diff = light_1.center - light_2.center;
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angle_ok = angle < a.max_angle;
        // cout << light_ratio_ok << " " << center_distance_ok <<" "<< angle_ok << endl;
        bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

        // Judge armor type
        ArmorType type;
        if (is_armor) {
            // cout << "is_armor" << endl;
            type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
        } else {
            // cout << "is not armor" << endl;
            type = ArmorType::INVALID;
        }

        // 仿真参数设置
        // Fill in debug information
        // auto_aim_interfaces::msg::DebugArmor armor_data;
        // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
        // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
        // armor_data.light_ratio = light_length_ratio;
        // armor_data.center_distance = center_distance;
        // armor_data.angle = angle;
        // this->debug_armors.data.emplace_back(armor_data);

        return type;
    }

    cv::Mat Detector::getAllNumbersImage()
    {
        if (armors_.empty()) {
            return cv::Mat(cv::Size(20, 28), CV_8UC1);
        } else {
            std::vector<cv::Mat> number_imgs;
            number_imgs.reserve(armors_.size());
            for (auto & armor : armors_) {
            number_imgs.emplace_back(armor.number_img);
            }
            cv::Mat all_num_img;
            cv::vconcat(number_imgs, all_num_img);
            return all_num_img;
        }
    }

    void Detector::drawResults(cv::Mat & img)
    {
        // Draw Lights
        for (const auto & light : lights_) {
            cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
            cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
            auto line_color = light.color == RED ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 255, 0);
            cv::line(img, light.top, light.bottom, line_color, 2);
        }


        // Draw armors
        for (const auto & armor : armors_) {
            cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
            cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
        }

        // Show numbers and confidence
        for (const auto & armor : armors_) {
            cv::putText(
            img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
            cv::Scalar(0, 255, 255), 2);
        }
        cv::namedWindow("lights", cv::WINDOW_NORMAL);
        cv::imshow("lights", img);
    }
}//auto_aim

int main() {
    //---------读取图像文件---------
    cv::Mat rgb_img = cv::imread("D:/Moon/RoborMaster/cvPhoto/ImageDataSet1/test/image/479.jpg", cv::IMREAD_COLOR);

    if (rgb_img.empty()) {
        std::cout << "Failed to read image" << std::endl;
        return -1;
    }
    //-----------创建对象-------------------
    // 创建 rm_auto_aim::Detector 对象
    rm_auto_aim::Detector::LightParams lightParams;
    // 设置 lightParams 的值
    rm_auto_aim::Detector::ArmorParams armorParams;
    // 设置 armorParams 的值
    int binary_thres = 170;
    int detect_color = 1;
    rm_auto_aim::Detector detector(binary_thres, detect_color, lightParams, armorParams);


    //------调用-----------------
    detector.detect(rgb_img);
    detector.drawResults(rgb_img);
    //等待按下任意按键后关闭窗口
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}