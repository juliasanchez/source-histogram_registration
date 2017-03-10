#ifndef GET_ANGLES_HIST
#define GET_ANGLES_HIST

#include <iostream>
#include <string>

typedef pcl::PointXYZ pcl_point;

void get_angles_hist(pcl::PointCloud<pcl::PointNormal>::Ptr normals, std::vector<std::vector<float>>& hist);

#include "get_angles_hist.inl"

#endif // GET_ANGLES_HIST
