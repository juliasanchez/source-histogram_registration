#ifndef GET_PROJ_AXIS
#define GET_PROJ_AXIS

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_proj_axis(float axis1_min, float axis2_min, float axis1_max, float axis2_max,  pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<std::vector<float>>& hist_axis);

#include "get_proj_axis.inl"

#endif // GET_PROJ_AXIS
