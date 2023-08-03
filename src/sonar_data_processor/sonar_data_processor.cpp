#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "sonar_data_processor.h"

static const glm::fvec3 UP_AXIS = glm::fvec3(0.0, 0.0, 1.0);   // z in UE4 and RViz
static const int POINT_LENGTH = sizeof(float) * 3;

SonarDataProcessor::SonarDataProcessor(
    int minimum_range,
    int maximum_range,
    int azimuth,
    float surface_detection_threshold,
    std::function<void(std::vector<uint8_t>)> &result_callback
) {
    minimum_range_ = minimum_range;
    maximum_range_ = maximum_range;
    azimuth_ = azimuth;
    surface_detection_threshold_ = surface_detection_threshold;
    result_callback_ = result_callback;

    processing_thread_ = std::thread(&SonarDataProcessor::convert_sonar_data_to_pointcloud, this);
}

void SonarDataProcessor::process_data(const std::vector<std::vector<double>> &data) {
    std::unique_lock<std::mutex> lock(mutex_);
    processing_queue_.push(data);
}

glm::fvec3 rotate(const glm::fvec3& vector, const glm::fvec3& rotation_axis, float theta) {
    const float cos_theta = cos(theta);
    const float sin_theta = sin(theta);

    const glm::fvec3 rotated = (vector * cos_theta) + (glm::cross(rotation_axis, vector) * sin_theta) + (rotation_axis * glm::dot(rotation_axis, vector)) * (1 - cos_theta);

    return rotated;
}

void SonarDataProcessor::convert_sonar_data_to_pointcloud() {
    while (true) {

        std::vector<std::vector<double>> imaging_sonar_data;
        {
            std::unique_lock<std::mutex> lock(mutex_);

            if (processing_queue_.empty()) {
                continue;
            }

            imaging_sonar_data = processing_queue_.front();
            processing_queue_.pop();
        }

        const int number_of_range_bins = imaging_sonar_data.size();
        const int number_of_azimuth_bins = imaging_sonar_data[0].size();

        std::vector<uint8_t> point_cloud_data;
        point_cloud_data.reserve(number_of_range_bins * number_of_azimuth_bins);

        for (int rangebin_index = 0; rangebin_index < number_of_range_bins; rangebin_index++) {
            for (int azimuthbin_index = 0; azimuthbin_index < number_of_azimuth_bins; azimuthbin_index++) {
                const double intensity = imaging_sonar_data[rangebin_index][azimuthbin_index];
                if (intensity < surface_detection_threshold_) {
                    continue;
                }

                // Calculate detected surface point
                const double current_distance = (rangebin_index * (maximum_range_ - minimum_range_)) / (double)number_of_range_bins + minimum_range_;
                const double current_angle = (azimuth_ / 2) - ((azimuth_ * azimuthbin_index) / (double)number_of_azimuth_bins);
                const float theta = current_angle * (M_PI / 180);

                const glm::fvec3 point_vector(current_distance, 0.0, 0.0);          // forward vector multiplied with the point's distance
                glm::fvec3 current_point = rotate(point_vector, UP_AXIS, theta);    // rotate to point to the right direction
                current_point.y *= -1;                                              // flip UE4's Y axis for RViz

                // Convert point to byte arrays
                uint8_t* currrent_point_in_bytes = reinterpret_cast<uint8_t*>(&current_point);
                for (unsigned int i = 0; i < POINT_LENGTH; i++) {
                    point_cloud_data.push_back(currrent_point_in_bytes[i]);
                }
            }
        }

        result_callback_(point_cloud_data);
    }
}