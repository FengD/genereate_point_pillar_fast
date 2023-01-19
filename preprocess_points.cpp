#include <string.h>
#include <cmath>
#include <iostream>
#include "preprocess_points.h"


template class PreprocessPoints<PillarBaseLidar>;
template class PreprocessPoints<PillarBaseRadar>;

template <typename T>
PreprocessPoints<T>::PreprocessPoints(const int MAX_NUM_PILLARS, const int MAX_NUM_POINTS_PER_PILLAR,
                                      const int MAX_NUM_POINTS, const int GRID_X_SIZE,
                                   const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE,
                                   const float PILLAR_Y_SIZE, const float PILLAR_Z_SIZE, const float MIN_X_RANGE,
                                   const float MIN_Y_RANGE, const float MIN_Z_RANGE,
                                   const int IN_POINT_DIM, const int OUT_POINT_DIM, const int PILLAR_DIM)
  : MAX_NUM_PILLARS(MAX_NUM_PILLARS)
  , MAX_NUM_POINTS_PER_PILLAR(MAX_NUM_POINTS_PER_PILLAR)
  , MAX_NUM_POINTS(MAX_NUM_POINTS)
  , GRID_X_SIZE(GRID_X_SIZE)
  , GRID_Y_SIZE(GRID_Y_SIZE)
  , GRID_Z_SIZE(GRID_Z_SIZE)
  , PILLAR_X_SIZE(PILLAR_X_SIZE)
  , PILLAR_Y_SIZE(PILLAR_Y_SIZE)
  , PILLAR_Z_SIZE(PILLAR_Z_SIZE)
  , MIN_X_RANGE(MIN_X_RANGE)
  , MIN_Y_RANGE(MIN_Y_RANGE)
  , MIN_Z_RANGE(MIN_Z_RANGE)
  , IN_POINT_DIM(IN_POINT_DIM)
  , OUT_POINT_DIM(OUT_POINT_DIM)
  , PILLAR_DIM(PILLAR_DIM) {
    num_valid_pillar_ = 0;
    coor_to_pillaridx_ = (int32_t*)malloc(GRID_Y_SIZE * GRID_X_SIZE * sizeof(int32_t));
    valid_points_pillar_map_ = (int32_t*)malloc(MAX_NUM_POINTS * sizeof(int32_t));
    pillar_bases_ = (T*)malloc(MAX_NUM_PILLARS * sizeof(T));
    pillar_points_counter_ = (int32_t*)malloc(MAX_NUM_PILLARS * sizeof(int32_t));
    clear();
}

template <typename T>
void PreprocessPoints<T>::clear() {
  memset(coor_to_pillaridx_, -1, GRID_Y_SIZE * GRID_X_SIZE * sizeof(int32_t));
  memset(valid_points_pillar_map_, -1, MAX_NUM_POINTS * sizeof(int32_t));
  memset(pillar_bases_, 0, MAX_NUM_PILLARS * sizeof(T));
  memset(pillar_points_counter_, 0, MAX_NUM_PILLARS * sizeof(int32_t));
}

template <typename T>
void PreprocessPoints<T>::generate_pillars(const float* in_points_array, const int& in_num_points, int32_t* out_pillars) {
  clear();
  int pillar_count = 0;
  int index = 0;
  for (auto i = 0; i < in_num_points; ++i) {
    if (i > MAX_NUM_POINTS) {
      break;
    }
    int x_coor = std::floor((in_points_array[i * IN_POINT_DIM] - MIN_X_RANGE) / PILLAR_X_SIZE);
    int y_coor = std::floor((in_points_array[i * IN_POINT_DIM + 1] - MIN_Y_RANGE) / PILLAR_Y_SIZE);
    int z_coor = std::floor((in_points_array[i * IN_POINT_DIM + 2] - MIN_Z_RANGE) / PILLAR_Z_SIZE);
    if (x_coor < 0 || x_coor >= GRID_X_SIZE || y_coor < 0 || y_coor >= GRID_Y_SIZE || z_coor < 0 ||
        z_coor >= GRID_Z_SIZE) {
      continue;
    }
    index = y_coor * GRID_X_SIZE + x_coor;
    int pillar_index = coor_to_pillaridx_[index];
    if (-1 == pillar_index) {
      pillar_index = pillar_count;
      if (pillar_count >= MAX_NUM_PILLARS) {
        break;
      }
      pillar_count += 1;
      coor_to_pillaridx_[index] = pillar_index;
      pillar_bases_[pillar_index].has_points = true;
      pillar_bases_[pillar_index].pillar_coordinate_x = x_coor;
      pillar_bases_[pillar_index].pillar_coordinate_y = y_coor;
      pillar_bases_[pillar_index].geo_x = x_coor * PILLAR_X_SIZE + PILLAR_X_SIZE / 2 + MIN_X_RANGE;
      pillar_bases_[pillar_index].geo_y = y_coor * PILLAR_Y_SIZE + PILLAR_Y_SIZE / 2 + MIN_Y_RANGE;
      pillar_bases_[pillar_index].geo_z = z_coor * PILLAR_Z_SIZE + PILLAR_Z_SIZE / 2 + MIN_Z_RANGE;
    }

    if (pillar_bases_[pillar_index].num_points_in_pillar < MAX_NUM_POINTS_PER_PILLAR) {
      pillar_bases_[pillar_index].points_sum_x += in_points_array[i * IN_POINT_DIM];
      pillar_bases_[pillar_index].points_sum_y += in_points_array[i * IN_POINT_DIM + 1];
      pillar_bases_[pillar_index].points_sum_z += in_points_array[i * IN_POINT_DIM + 2];
      pillar_bases_[pillar_index].num_points_in_pillar += 1;
      valid_points_pillar_map_[i] = pillar_index;
    }
  }
  for (auto i = 0; i < pillar_count; i++) {
    pillar_bases_[i].points_mean_x = pillar_bases_[i].points_sum_x / pillar_bases_[i].num_points_in_pillar;
    pillar_bases_[i].points_mean_y = pillar_bases_[i].points_sum_y / pillar_bases_[i].num_points_in_pillar;
    pillar_bases_[i].points_mean_z = pillar_bases_[i].points_sum_z / pillar_bases_[i].num_points_in_pillar;
    out_pillars[i * PILLAR_DIM + 2] = pillar_bases_[i].pillar_coordinate_x;
    out_pillars[i * PILLAR_DIM + 3] = pillar_bases_[i].pillar_coordinate_y;
  }
}

template <typename T>
void PreprocessPoints<T>::generate_points(const float* in_points_array, const int& in_num_points, float* out_points) {
  for (auto i = 0; i < in_num_points; i++) {
    if (i > MAX_NUM_POINTS) {
      break;
    }
    if (-1 != valid_points_pillar_map_[i]) {
      int index = valid_points_pillar_map_[i] * MAX_NUM_POINTS_PER_PILLAR + pillar_points_counter_[valid_points_pillar_map_[i]];
      int dim = i * IN_POINT_DIM;
      T p = pillar_bases_[valid_points_pillar_map_[i]];
      out_points[index] = in_points_array[dim];
      out_points[index + 1] = in_points_array[dim + 1];
      out_points[index + 2] = in_points_array[dim + 2];
      out_points[index + 3] = in_points_array[dim + 3];
      out_points[index + 4] = in_points_array[dim] - p.points_mean_x;
      out_points[index + 5] = in_points_array[dim + 1] - p.points_mean_y;
      out_points[index + 6] = in_points_array[dim + 2] - p.points_mean_z;
      out_points[index + 7] = in_points_array[dim] - p.geo_x;
      out_points[index + 8] = in_points_array[dim + 1] - p.geo_y;
      out_points[index + 9] = in_points_array[dim + 2] - p.geo_z;
      pillar_points_counter_[valid_points_pillar_map_[i]]++;
    }
  }
}


template <typename T>
void PreprocessPoints<T>::generate_pillars_and_points(const float* in_points_array,
  const int& in_num_points, float* out_points, int32_t* out_pillars) {
  generate_pillars(in_points_array, in_num_points, out_pillars);
  generate_points(in_points_array, in_num_points, out_points);
}
