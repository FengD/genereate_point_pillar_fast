#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "preprocess_points.h"

void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points) {
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++) {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points[i * 4 + 0] = point.x;
    out_points[i * 4 + 1] = point.y;
    out_points[i * 4 + 2] = point.z;
    out_points[i * 4 + 3] = point.intensity;
  }
}

void makePointsForTest(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr) {
  pcl::PointXYZI point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  point.intensity = 4;
  in_pcl_pc_ptr->push_back(point);
  point.x = 5;
  point.y = 5;
  point.z = 0;
  point.intensity = 4;
  in_pcl_pc_ptr->push_back(point);
  point.x = -5;
  point.y = -5;
  point.z = 0;
  point.intensity = 4;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.9892;
  point.y = -9.98058;
  point.z = 0;
  point.intensity = 4;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.5673;
  point.y = -12.9834;
  point.z = 0.21862;
  point.intensity = 1;
  in_pcl_pc_ptr->push_back(point);
  point.x = 11.8697;
  point.y = -11.123;
  point.z = -0.189377;
  point.intensity = 35;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.489;
  point.y = -9.59703;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.9084;
  point.y = -10.9626;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.5673;
  point.y = -12.9834;
  point.z = 0.21862;
  point.intensity = 1;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.9184;
  point.y = -10.9626;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 12.9384;
  point.y = -10.9626;
  point.z = -2.15565;
  point.intensity = 11;
  in_pcl_pc_ptr->push_back(point);
  point.x = 73.8676;
  point.y = -9.61668;
  point.z = 0.0980819;
  point.intensity = 14;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.5673;
  point.y = -12.9834;
  point.z = 0.21862;
  point.intensity = 1;
  in_pcl_pc_ptr->push_back(point);
  point.x = 13.8213;
  point.y = -10.8529;
  point.z = -1.22883;
  point.intensity = 19;
  in_pcl_pc_ptr->push_back(point);
  point.x = 11.8957;
  point.y = -10.3189;
  point.z = -1.28556;
  point.intensity = 13;
  in_pcl_pc_ptr->push_back(point);
  point.x = 11.897;
  point.y = -10.3189;
  point.z = -1.28556;
  point.intensity = 20;
  in_pcl_pc_ptr->push_back(point);
  for (int i = 0; i < 1000; i++) {
    point.x = -50 + 0.04 * i;
    point.y = 50 - 0.03 * i;
    point.z = 0;
    point.intensity = 20;
    in_pcl_pc_ptr->push_back(point);
  }
}


int main(int argc, char** argv) {
    const int32_t MAX_NUM_PILLARS = 10000;
    const int32_t MAX_NUM_POINTS_PER_PILLAR = 32;
    const int32_t MAX_NUM_POINTS = 100000;
    const int32_t GRID_X_SIZE = 500;
    const int32_t GRID_Y_SIZE = 500;
    const int32_t GRID_Z_SIZE = 1;
    const float PILLAR_X_SIZE = 0.2f;
    const float PILLAR_Y_SIZE = 0.2f;
    const float PILLAR_Z_SIZE = 4.0f;
    const float MIN_X_RANGE = -50.0f;
    const float MIN_Y_RANGE = -50.0f;
    const float MIN_Z_RANGE = -3.0f;
    const int32_t IN_POINT_DIM = 4;
    const int32_t OUT_POINT_DIM = 10;
    const int32_t PILLAR_DIM = 4;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    makePointsForTest(pcl_pc_ptr);
    float* in_points = (float*)malloc(IN_POINT_DIM * sizeof(float) * pcl_pc_ptr->size());
    float* out_points = (float*)malloc(OUT_POINT_DIM * sizeof(float) * MAX_NUM_PILLARS * MAX_NUM_PILLARS);
    int32_t* out_pillars = (int32_t*)malloc(MAX_NUM_PILLARS * sizeof(int32_t) * PILLAR_DIM);
    memset(in_points, 0, IN_POINT_DIM * sizeof(float) * pcl_pc_ptr->size());
    memset(out_points, 0, OUT_POINT_DIM * sizeof(float) * MAX_NUM_PILLARS * MAX_NUM_PILLARS);
    memset(out_pillars, 0, MAX_NUM_PILLARS * sizeof(int32_t) * PILLAR_DIM);
    pclToArray(pcl_pc_ptr, in_points);
    std::shared_ptr<PreprocessPoints<PillarBaseLidar>> preprocess_points_ptr_;
    preprocess_points_ptr_.reset(new PreprocessPoints<PillarBaseLidar>(
        MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, MAX_NUM_POINTS, GRID_X_SIZE,
        GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE, PILLAR_Y_SIZE,
        PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE,
        IN_POINT_DIM, OUT_POINT_DIM, PILLAR_DIM));
    auto b = std::chrono::high_resolution_clock::now();
    auto b_us = std::chrono::time_point_cast<std::chrono::microseconds>(b).time_since_epoch().count();

    // for(int i = 0; i < 10000; i++) {
    preprocess_points_ptr_->generate_pillars_and_points(in_points, pcl_pc_ptr->size(), out_points, out_pillars);
    // }
    // auto l = std::chrono::high_resolution_clock::now();
    // auto l_us = std::chrono::time_point_cast<std::chrono::microseconds>(l).time_since_epoch().count();

    // std::cout << (l_us - b_us) / 1000 << std::endl;
    uint8_t mm[GRID_X_SIZE * GRID_Y_SIZE] = {0};
    for (int i = 0; i < MAX_NUM_PILLARS; ++i) {
        if (out_pillars[i * PILLAR_DIM + 2] != 0 || out_pillars[i * PILLAR_DIM + 3] != 0) {
            mm[out_pillars[i * PILLAR_DIM + 3] * GRID_X_SIZE + out_pillars[i * PILLAR_DIM + 2]] = 255;
        }
    }
    cv::Mat pillar_img(GRID_X_SIZE, GRID_Y_SIZE, CV_8UC1, mm);
    cv::imwrite("test.png", pillar_img);
    return 0;
}