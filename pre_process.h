#ifndef PRE_PROCESS
#define PRE_PROCESS

#include <iostream>
#include <string>
#include <pcl/features/fpfh_omp.h>

#include "cloud.h"
#include "display_normals.h"

typedef pcl::PointXYZ pcl_point;

void pre_process(std::string pcd_file1,float sample,float normal_radius,int display, pcl::PointCloud<pcl_point>::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr, float* max_dist);

#include "pre_process.inl"

#endif // PRE_PROCESS
