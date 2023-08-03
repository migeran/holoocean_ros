#include <iostream>
#include <mutex>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <queue>
#include <thread>

namespace py = pybind11;

class SonarDataProcessor{
private:
    int minimum_range_;
    int maximum_range_;
    int azimuth_;
    float surface_detection_threshold_;
    std::function<void(std::vector<uint8_t>)> result_callback_;

    std::thread processing_thread_;
    std::mutex mutex_;
    std::queue<std::vector<std::vector<double>>> processing_queue_;

public:
    SonarDataProcessor(
        int minimum_range,
        int maximum_range,
        int azimuth,
        float surface_detection_threshold,
        std::function<void(std::vector<uint8_t>)> &result_callback);
    void process_data(const std::vector<std::vector<double>> &data);

private:
    void convert_sonar_data_to_pointcloud();
};

PYBIND11_MODULE(sonar_data_processor, module) {
    py::class_<SonarDataProcessor>(module, "SonarDataProcessor")
        .def(py::init<int, int, int, float, std::function<void(std::vector<uint8_t>)>&>())
        .def("process_data", &SonarDataProcessor::process_data);
}