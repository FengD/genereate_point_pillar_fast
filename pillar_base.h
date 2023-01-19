#pragma once

#include <stdlib.h>

/**
 * @class PillarBaseLidar
 * @brief The base of the lidar pillar struct. Cloud extend.
 */
struct PillarBaseLidar {
  bool has_points;
  int32_t pillar_coordinate_x;
  int32_t pillar_coordinate_y;
  int32_t num_points_in_pillar;
  union {
    float pillar_info1[10];
    struct {
      float points_mean_x;
      float points_mean_y;
      float points_mean_z;
      float points_sum_x;
      float points_sum_y;
      float points_sum_z;
    };
  };
  union {
    float pillar_info2[3];
    struct {
      float geo_x;
      float geo_y;
      float geo_z;
    };
  };
  union {
    float pillar_info3[2];
    struct {
      float max_intensity;
    };
  };
};

/**
 * @class PillarBaseRadar
 * @brief The base of the radar pillar struct. Cloud extend.
 */
struct PillarBaseRadar {
  bool has_points;
  int32_t pillar_coordinate_x;
  int32_t pillar_coordinate_y;
  int32_t num_points_in_pillar;
  union {
    float pillar_info1[10];
    struct {
      float points_mean_x;
      float points_mean_y;
      float points_mean_z;
      float points_sum_x;
      float points_sum_y;
      float points_sum_z;
      float points_mean_vx;
      float points_mean_vy;
      float points_sum_vx;
      float points_sum_vy;
    };
  };
  union {
    float pillar_info2[3];
    struct {
      float geo_x;
      float geo_y;
      float geo_z;
    };
  };
  union {
    float pillar_info3[2];
    struct {
      float max_rcs;
    };
  };
};