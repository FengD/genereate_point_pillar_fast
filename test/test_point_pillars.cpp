/*
 * Copyright 2018-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
* @file test_point_pillars.cpp
* @brief unit test file
* @author Kosuke Murakami
* @date 2019/02/26
*/

// headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

// headers in gtest
#include <gtest/gtest.h>

//headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//headers in local files
#include "lidar_point_pillars/point_pillars_ros.h"
#include "lidar_point_pillars/preprocess_points.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }
};

class TestClass
{
public:
  TestClass(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR, const int GRID_X_SIZE,
            const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
            const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE, const float MIN_Z_RANGE,
            const int NUM_INDS_FOR_SCAN, const int NUM_POINT_DIM);
  const int MAX_NUM_PILLARS;
  const int MAX_NUM_POINTS_PER_PILLAR;
  const int GRID_X_SIZE;
  const int GRID_Y_SIZE;
  const int GRID_Z_SIZE;
  const float PILLAR_X_SIZE;
  const float PILLAR_Y_SIZE;
  const float PILLAR_Z_SIZE;
  const float MIN_X_RANGE;
  const float MIN_Y_RANGE;
  const float MIN_Z_RANGE;
  const int NUM_INDS_FOR_SCAN;
  const int NUM_POINT_DIM;

  // Make pointcloud for test
  void makePointsForTest(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr);
  void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array);
  void preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
                  float* num_points_per_pillar, float* pillar_x, float* pillar_y, float* pillar_z, float* pillar_i,
                  float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped, float* pillar_feature_mask,
                  float* sparse_pillar_map, int* host_pillar_count);
  void generateAnchors(float* anchors_px, float* anchors_py, float* anchors_pz, float* anchors_dx,
                       float* anchors_dy, float* anchors_dz, float* anchors_ro);
  void convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py, float* anchors_dx, float* anchors_dy,
                                 float* box_anchors_min_x, float* box_anchors_min_y,
                                 float* box_anchors_max_x, float* box_anchors_max_y);

private:
  std::unique_ptr<PreprocessPoints> preprocess_points_ptr_;
  std::unique_ptr<PointPillars> point_pillars_ptr_;
};

TestClass::TestClass(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR, const int GRID_X_SIZE,
                     const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
                     const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE,
                     const float MIN_Z_RANGE, const int NUM_INDS_FOR_SCAN, const int NUM_POINT_DIM)
  : MAX_NUM_PILLARS(MAX_NUM_PILLARS)
  , MAX_NUM_POINTS_PER_PILLAR(MAX_NUM_POINTS_PER_PILLAR)
  , GRID_X_SIZE(GRID_X_SIZE)
  , GRID_Y_SIZE(GRID_Y_SIZE)
  , GRID_Z_SIZE(GRID_Z_SIZE)
  , PILLAR_X_SIZE(PILLAR_X_SIZE)
  , PILLAR_Y_SIZE(PILLAR_Y_SIZE)
  , PILLAR_Z_SIZE(PILLAR_Z_SIZE)
  , MIN_X_RANGE(MIN_X_RANGE)
  , MIN_Y_RANGE(MIN_Y_RANGE)
  , MIN_Z_RANGE(MIN_Z_RANGE)
  , NUM_INDS_FOR_SCAN(NUM_INDS_FOR_SCAN)
  , NUM_POINT_DIM(NUM_POINT_DIM)
{
  preprocess_points_ptr_.reset(new PreprocessPoints(
      MAX_NUM_PILLARS, MAX_NUM_POINTS_PER_PILLAR, GRID_X_SIZE, GRID_Y_SIZE, GRID_Z_SIZE, PILLAR_X_SIZE,
      PILLAR_Y_SIZE, PILLAR_Z_SIZE, MIN_X_RANGE, MIN_Y_RANGE, MIN_Z_RANGE, NUM_INDS_FOR_SCAN, NUM_POINT_DIM));

  bool baselink_support=true;
  bool reproduce_result_mode=false;
  float score_threshold = 0.5;
  float nms_overlap_threshold = 0.5;
  std::string package_path = ros::package::getPath("lidar_point_pillars");
#ifdef TVM_IMPLEMENTATION
  std::string path = package_path + "/test/data/data_tvm/";
#else
  std::string path = package_path + "/test/data/data_onnx/dummy.onnx";
#endif
  std::string pfe_path = path;
  std::string rpn_path = path;

  point_pillars_ptr_.reset(new PointPillars(reproduce_result_mode, score_threshold, nms_overlap_threshold,
                                            pfe_path, rpn_path));
};

void TestClass::preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
                           float* num_points_per_pillar, float* pillar_x, float* pillar_y, float* pillar_z,
                           float* pillar_i, float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped,
                           float* pillar_feature_mask, float* sparse_pillar_map, int* host_pillar_count) {
  preprocess_points_ptr_->preprocess(in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar, pillar_x,
                                     pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped,
                                     pillar_feature_mask, sparse_pillar_map, host_pillar_count);
}

void TestClass::pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array) {
  for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
  {
    pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
    out_points_array[i * 4 + 0] = point.x;
    out_points_array[i * 4 + 1] = point.y;
    out_points_array[i * 4 + 2] = point.z;
    out_points_array[i * 4 + 3] = point.intensity;
  }
}

void TestClass::makePointsForTest(pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl_pc_ptr) {
  pcl::PointXYZI point;
  point.x = 12.9892;
  point.y = -9.98058;
  point.z = 0;
  point.intensity = 4;
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
  point.x = 13.8676;
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
  point.y = -10.2189;
  point.z = -1.28556;
  point.intensity = 20;
  in_pcl_pc_ptr->push_back(point);
}


TEST(TestSuite, CheckPreprocessPointsCPU) {
  const int MAX_NUM_PILLARS = 12000;
  const int MAX_NUM_POINTS_PER_PILLAR = 100;
  const int GRID_X_SIZE = 432;
  const int GRID_Y_SIZE = 496;
  const int GRID_Z_SIZE = 1;
  const float PILLAR_X_SIZE = 0.16;
  const float PILLAR_Y_SIZE = 0.16;
  const float PILLAR_Z_SIZE = 4.0;
  const float MIN_X_RANGE = 0;
  const float MIN_Y_RANGE = -39.68;
  const float MIN_Z_RANGE = -3.0;
  const int NUM_INDS_FOR_SCAN = 512;
  const int NUM_POINT_DIM = 4;
  TestClass test_obj(MAX_NUM_PILLARS,
                     MAX_NUM_POINTS_PER_PILLAR,
                     GRID_X_SIZE,
                     GRID_Y_SIZE,
                     GRID_Z_SIZE,
                     PILLAR_X_SIZE,
                     PILLAR_Y_SIZE,
                     PILLAR_Z_SIZE,
                     MIN_X_RANGE,
                     MIN_Y_RANGE,
                     MIN_Z_RANGE,
                     NUM_INDS_FOR_SCAN,
                     NUM_POINT_DIM);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  test_obj.makePointsForTest(pcl_pc_ptr);

  float* points_array = new float[pcl_pc_ptr->size() * 4];
  test_obj.pclToArray(pcl_pc_ptr, points_array);

  int x_coors[test_obj.MAX_NUM_PILLARS] = { 0 };
  int y_coors[test_obj.MAX_NUM_PILLARS] = { 0 };
  float num_points_per_pillar[test_obj.MAX_NUM_PILLARS] = { 0 };
  float* pillar_x = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];
  float* pillar_y = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];
  float* pillar_z = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];
  float* pillar_i = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];

  float* x_coors_for_sub_shaped = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];
  float* y_coors_for_sub_shaped = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];
  float* pillar_feature_mask = new float[test_obj.MAX_NUM_PILLARS * test_obj.MAX_NUM_POINTS_PER_PILLAR];

  float* sparse_pillar_map = new float[512 * 512];

  int host_pillar_count[1] = { 0 };
  test_obj.preprocess(points_array, pcl_pc_ptr->size(), x_coors, y_coors, num_points_per_pillar, pillar_x, pillar_y,
                      pillar_z, pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped, pillar_feature_mask,
                      sparse_pillar_map, host_pillar_count);
  EXPECT_EQ(1, num_points_per_pillar[0]);
  EXPECT_FLOAT_EQ(12.9892, pillar_x[0]);
  EXPECT_EQ(74, x_coors[1]);
  EXPECT_EQ(178, y_coors[1]);
  EXPECT_EQ(1, sparse_pillar_map[178 * 512 + 74]);
  EXPECT_EQ(8, host_pillar_count[0]);
  delete[] points_array;
  delete[] pillar_x;
  delete[] pillar_y;
  delete[] pillar_z;
  delete[] pillar_i;
  delete[] x_coors_for_sub_shaped;
  delete[] y_coors_for_sub_shaped;
  delete[] pillar_feature_mask;
  delete[] sparse_pillar_map;
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
