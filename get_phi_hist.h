#ifndef GET_PHI_HIST
#define GET_PHI_HIST

#include <iostream>
#include <string>
#include <pcl/features/fpfh_omp.h>

#include "cloud.h"
#include "display_normals.h"

typedef pcl::PointXYZ pcl_point;

void get_phi_hist(pcl::PointCloud<pcl::PointNormal>::Ptr normals, std::vector<std::vector<float>>& hist);

#include "get_phi_hist.inl"

#endif // GET_PHI_HIST