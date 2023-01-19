#pragma once

#include "pillar_base.h"

template <typename T>
class PreprocessPoints {
 private:
  const int MAX_NUM_PILLARS;
  const int MAX_NUM_POINTS_PER_PILLAR;
  const int MAX_NUM_POINTS;
  const int GRID_X_SIZE;
  const int GRID_Y_SIZE;
  const int GRID_Z_SIZE;
  const float PILLAR_X_SIZE;
  const float PILLAR_Y_SIZE;
  const float PILLAR_Z_SIZE;
  const float MIN_X_RANGE;
  const float MIN_Y_RANGE;
  const float MIN_Z_RANGE;
  const int IN_POINT_DIM;
  const int OUT_POINT_DIM;
  const int PILLAR_DIM;
  int32_t num_valid_pillar_;
  int32_t *coor_to_pillaridx_;
  int32_t *valid_points_pillar_map_;
  int32_t *pillar_points_counter_;
  T *pillar_bases_;

  /**
  * @brief Initializing variables for preprocessing
  * @details Initializeing input arguments with certain values
  */
  void clear();

  /**
  * @brief Generate the pillars feature and the points feature
  * @param[in] in_points_array Pointcloud array
  * @param[in] in_num_points The number of points
  * @param[out] out_pillars pillar array
  */
  void generate_pillars(const float* in_points_array, const int& in_num_points, int32_t* out_pillars);

  /**
  * @brief Generate the pillars feature and the points feature
  * @param[in] in_points_array Pointcloud array
  * @param[in] in_num_points The number of points
  * @param[out] out_points Pointcloud array
  */
  void generate_points(const float* in_points_array, const int& in_num_points, float* out_points);

 public:
  /**
  * @brief Constructor
  * @param[in] MAX_NUM_PILLARS Maximum number of pillars
  * @param[in] MAX_NUM_POINTS_PER_PILLAR Maximum number of points per pillar
  * @param[in] MAX_NUM_POINTS Maximum number of points per pillar
  * @param[in] GRID_X_SIZE Number of pillars in x-coordinate
  * @param[in] GRID_Y_SIZE Number of pillars in y-coordinate
  * @param[in] GRID_Z_SIZE Number of pillars in z-coordinate
  * @param[in] PILLAR_X_SIZE Size of x-dimension for a pillar
  * @param[in] PILLAR_Y_SIZE Size of y-dimension for a pillar
  * @param[in] PILLAR_Z_SIZE Size of z-dimension for a pillar
  * @param[in] MIN_X_RANGE Minimum x value for pointcloud
  * @param[in] MIN_Y_RANGE Minimum y value for pointcloud
  * @param[in] MIN_Z_RANGE Minimum z value for pointcloud
  * @param[in] IN_POINT_DIM Number of point dimension
  * @param[in] OUT_POINT_DIM Number of point dimension
  * @param[in] PILLAR_DIM Pillar dimension idx, z, x, y
  * @details Captital variables never change after the compile
  */
  PreprocessPoints(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR, const int MAX_NUM_POINTS, const int GRID_X_SIZE,
                   const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE, const float PILLAR_Y_SIZE,
                   const float PILLAR_Z_SIZE, const float MIN_X_RANGE, const float MIN_Y_RANGE, const float MIN_Z_RANGE,
                   const int IN_POINT_DIM, const int OUT_POINT_DIM, const int PILLAR_DIM);

  /**
  * @brief Generate the pillars feature and the points feature
  * @param[in] in_points_array Pointcloud array
  * @param[in] in_num_points The number of points
  * @param[out] out_points Pointcloud array
  * @param[out] out_pillars pillar array
  */
  void generate_pillars_and_points(const float* in_points_array, const int& in_num_points,
                                   float* out_points, int32_t* out_pillars);

  
};
