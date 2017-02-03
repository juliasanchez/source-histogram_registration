#include <iostream>
#include <string>

#include "cloud.h"
#include "display_normals.h"
#include "get_phi_hist.h"
#include "get_proj_axis.h"
#include "pre_process.h"
#include "get_corr_axis.h"
#include "get_error_phi.h"
#include "get_error_theta.h"
#include "save_vector.h"
#include "get_walls.h"
#include "norm_hist.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    ///preprocess cloud--------------------------------------------------------------------------------------------------------------

    float normal_radius =atof(argv[4]);
    float sample =atof(argv[3]);
    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    pre_process(argv[1],sample,normal_radius, 0, cloud_src, normals_src);
    pre_process(argv[2],sample,normal_radius, 0, cloud_tgt, normals_tgt);

    pcl::io::savePCDFileASCII ("sampled_src.pcd", *cloud_src);
    pcl::io::savePCDFileASCII ("sampled_tgt.pcd", *cloud_tgt);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);

    ///compute 2D histograms (phi/theta)--------------------------------------------------------------------------------------------------------------

    //parameters for normal histograms...........................................................................................................

    int N_hist=atoi(argv[5]);

    std::vector<std::vector<float>> hist1(N_hist, std::vector<float>(N_hist*2,0.0));
    std::vector<float> hist1_phi(N_hist*2, 0.0);
    std::vector<float> hist11_phi(N_hist*2, 0.0);

    std::vector<std::vector<float>> hist2(N_hist,std::vector<float>(N_hist*2, 0.0));
    std::vector<float> hist2_phi(N_hist*2, 0.0);

    std::vector<float> error_phi(N_hist*2,0.0);
    std::vector<float>  error_theta(N_hist*2, 0.0);

    int rotation_phi;
    int rotation_theta;

    get_phi_hist(pointNormals_src,hist1);
    get_phi_hist(pointNormals_tgt,hist2);

    ///compute projection histogram on x,y plane--------------------------------------------------------------------------------------------------------------

    for (int n=0; n<N_hist; n++)
    {
        for (int m=0; m<N_hist*2; m++)
        {
            hist1_phi[m]=hist1_phi[m]+hist1[n][m];
            hist2_phi[m]=hist2_phi[m]+hist2[n][m];
        }
    }

    ///compute error function for phi--------------------------------------------------------------------------------------------------------------

    get_error_phi(hist1_phi, hist2_phi, error_phi, &rotation_phi);

    ///TURN CLOUD TO ALIGN PROJECTIONS AND TURN THE TWO CLOUDS OF 90°--------------------------------------------------------------------------------------------------------------

    //turn around x axis y becomes z and z becomes y. theta is the angle of rotation around y
    Eigen::Matrix4f transform_90 = Eigen::Matrix4f::Identity();
    transform_90 (1,2) = -1;
    transform_90 (2,1) = 1;
    transform_90 (1,1) = 0;
    transform_90 (2,2) = 0;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    //float phi = (float)(rotation_phi)*M_PI/(float)(N_hist);
    float phi=atof(argv[6]);
    if (phi==0)
    {
        phi=(float)(rotation_phi)*M_PI/(float)(N_hist);
    }
    //float phi=5.59596191421;
    //float phi=4.63875353924;
    transform (0,0) = cos (phi);
    transform (0,1) = -sin(phi);
    transform (1,0) = sin (phi);
    transform (1,1) = cos (phi);

    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, transform_90*transform);
    pcl::transformPointCloudWithNormals (*pointNormals_tgt, *pointNormals_tgt, transform_90);

    ///compute error function for theta--------------------------------------------------------------------------------------------------------------

    std::vector<std::vector<float>> hist1_theta(N_hist, std::vector<float>(N_hist*2,0.0));
    std::vector<std::vector<float>> hist2_theta(N_hist,std::vector<float>(N_hist*2, 0.0));

    get_phi_hist(pointNormals_src,hist1_theta);
    get_phi_hist(pointNormals_tgt,hist2_theta);

    get_error_theta(hist1_theta, hist2_theta, error_theta, &rotation_theta);

    ///save error functions--------------------------------------------------------------------------------------------------------------

    save_vector(error_phi, "error_phi.csv");
    save_vector(error_theta, "error_theta.csv");

    std::vector<float> res_phi(4,0.0);

    std::cout<<"  phi rotation  : "<<std::endl;

    for (int k =0; k<4; k++)
    {
        if((float)(rotation_phi)*180.0/(float)(N_hist)+k*90<360)
        {
          res_phi[k]=(float)(rotation_phi)*180.0/(float)(N_hist)+k*90;
          std::cout<<"  -->"<<(float)(rotation_phi)*180.0/(float)(N_hist)+k*90<<"  degrés"<<std::endl;
        }
        else
        {
          res_phi[k]=(float)(rotation_phi)*180.0/(float)(N_hist)-(4-k)*90;
          std::cout<<"  -->"<<(float)(rotation_phi)*180.0/(float)(N_hist)-(4-k)*90<<"  degrés"<<std::endl;
        }

    }

    std::cout<<std::endl<<"  theta rotation: "<<std::endl;

    for (int k =0; k<4; k++)
    {
        if((float)(rotation_theta)*180.0/(float)(N_hist)+k*90<360)
        {
        std::cout<<"  -->"<<(float)(rotation_theta)*180.0/(float)(N_hist)+k*90<<"  degrés"<<std::endl;
        }
        else
        {
          std::cout<<"  -->"<<(float)(rotation_theta)*180.0/(float)(N_hist)-(4-k)*90<<"  degrés"<<std::endl;
        }

    }

 ///rotate cloud in the accurate way with rotation information-----------------------------------------------------------------------------------

    Eigen::Matrix4f transform_phi = Eigen::Matrix4f::Identity();
    transform_phi (0,0) = cos (phi);
    transform_phi (0,1) = -sin(phi);
    transform_phi (1,0) = sin (phi);
    transform_phi (1,1) = cos (phi);

    Eigen::Matrix4f transform_theta = Eigen::Matrix4f::Identity();
    float theta=0;
    transform_theta (0,0) = cos (theta);
    transform_theta (0,2) = sin(theta);
    transform_theta (2,0) = -sin (theta);
    transform_theta (2,2) = cos (theta);

    std::cout<<transform_theta*transform_phi<<std::endl<<std::endl;

    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);

//    pcl::transformPointCloud (*cloud_src, *cloud_src, transform_theta*transform_phi);
    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, transform_theta*transform_phi);
//    pcl::io::savePCDFileASCII ("transformed_source.pcd", *cloud_src);

    ///compute phi and theta of the principal normal of the principal wall

    //get main axis from the target normals histogram
    float temp=0;
    for (int n=10; n<N_hist-10; n++)  //Je commence à 10 car je ne veux pas plafonds et sols (theta=0)
    {
        for (int m=0; m<N_hist*2; m++)
        {
            if(temp<hist2[n][m])
            {   temp=hist2[n][m];
                phi=(float)(m)*M_PI/(float)(N_hist);
                theta=(float)(n)*M_PI/(float)(N_hist);
            }
        }
    }

    //compute axis of these normals

    float angles1[]={phi, theta};
    float angles2[]={phi+M_PI/2, theta};
    std::vector<float> axis1={cos(angles1[0]), sin(angles1[0]), 0};
    std::vector<float> axis2={cos(angles2[0]), sin(angles2[0]), 0};

    save_vector(axis1, "axis1.csv");
    save_vector(axis2, "axis2.csv");

    ///transform clouds to be aligned in x and y axis


    Eigen::Matrix4f align_xy = Eigen::Matrix4f::Identity();
    align_xy (0,0) = cos (-phi);
    align_xy(0,1) = -sin(-phi);
    align_xy (1,0) = sin (-phi);
    align_xy (1,1) = cos (-phi);

    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, align_xy);
    pcl::transformPointCloudWithNormals (*pointNormals_tgt, *pointNormals_tgt, align_xy);
//    pcl::io::savePCDFileASCII ("align_xy_src.pcd", *pointNormals_src);
//    pcl::io::savePCDFileASCII ("align_xy_tgt.pcd", *pointNormals_tgt);

    ///filter clouds to keep walls--------------------------------------------------------------------------------------------------------------

    pcl::PointCloud<pcl_point>::Ptr cloud_src_filtered(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt_filtered(new pcl::PointCloud<pcl_point>);

    float lim=atof(argv[10]);

    get_walls(pointNormals_src, lim, cloud_src_filtered);
    get_walls(pointNormals_tgt, lim, cloud_tgt_filtered);

//    pcl::io::savePCDFileASCII ("source_filtered.pcd", *cloud_src_filtered);
//    pcl::io::savePCDFileASCII ("target_filtered.pcd", *cloud_tgt_filtered);

    ///get histograms on normals axes--------------------------------------------------------------------------------------------------------------

    int N_hist_axis=atoi(argv[7]);
    std::vector<std::vector<float>> hist1_axis(N_hist_axis,std::vector<float>(N_hist_axis,0.0));
    std::vector<std::vector<float>> hist2_axis(N_hist_axis,std::vector<float>(N_hist_axis,0.0));

    pcl::transformPointCloud(*cloud_src, *cloud_src, align_xy);
    pcl::transformPointCloud (*cloud_tgt, *cloud_tgt, align_xy);

    pcl::PointXYZ min_src, max_src, min_tgt, max_tgt;
    pcl::getMinMax3D (*cloud_src, min_src, max_src);
    pcl::getMinMax3D (*cloud_tgt, min_tgt, max_tgt);

    float axis1_min=std::min(min_src.x, min_tgt.x)-0.3;
    float axis2_min=std::min(min_src.y, min_tgt.y)-0.3;
    float axis1_max=std::max(max_src.x, max_tgt.x)+0.3;
    float axis2_max=std::max(max_src.y, max_tgt.y)+0.3;

    get_proj_axis(axis1_min, axis2_min, axis1_max, axis2_max,  cloud_src_filtered, hist1_axis);
    get_proj_axis(axis1_min, axis2_min, axis1_max, axis2_max,  cloud_tgt_filtered, hist2_axis);

    ///compute axis independent histograms--------------------------------------------------------------------------------------------------------------

    std::vector<float> hist1_axis1(N_hist_axis, 0.0);
    std::vector<float> hist2_axis1(N_hist_axis, 0.0);
    std::vector<float> hist1_axis2(N_hist_axis, 0.0);
    std::vector<float> hist2_axis2(N_hist_axis, 0.0);

    std::vector<float> corr_axis1(2*N_hist_axis-1, 0.0);
    std::vector<float> corr_axis2(2*N_hist_axis-1,0.0);

    int translation_axis1;
    int translation_axis2;

    for (int n=0; n<N_hist_axis; n++)
    {
        for (int m=0; m<N_hist_axis; m++)
        {
            hist1_axis1[n]=hist1_axis1[n]+hist1_axis[n][m];
            hist2_axis1[n]=hist2_axis1[n]+hist2_axis[n][m];
            hist1_axis2[m]=hist1_axis2[m]+hist1_axis[n][m];
            hist2_axis2[m]=hist2_axis2[m]+hist2_axis[n][m];
        }
    }

    norm_hist(hist1_axis1);
    norm_hist(hist1_axis2);
    norm_hist(hist2_axis1);
    norm_hist(hist2_axis2);

    ///save histograms--------------------------------------------------------------------------------------------------------------
    save_vector (hist1_axis1, "hist1_axis1.csv");
    save_vector (hist1_axis2, "hist1_axis2.csv");
    save_vector (hist2_axis1,"hist2_axis1.csv");
    save_vector (hist2_axis2,"hist2_axis2.csv");

    ///compute corr function for axis1--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis1, hist2_axis1, corr_axis1, &translation_axis1);

    save_vector(corr_axis1, "corr_axis1.csv");

    if(atoi(argv[8])!=0)
    {
    translation_axis1=atoi(argv[8]);
    }

    float delta1=(float)(  (axis1_max-axis1_min)/(float)(N_hist)  );

    std::cout<<"movment for axis 1: "<<std::endl;
    std::cout<<std::endl<<"  x translation: "<<(translation_axis1-N_hist_axis+1)*delta1*cos(phi)<<"  y translation: "<<(translation_axis1-N_hist_axis+1)*delta1*sin(phi)<<"  z translation: "<<(translation_axis1-N_hist_axis+1)*delta1*cos(theta)<<std::endl<<std::endl;


    ///compute corr function for axis2--------------------------------------------------------------------------------------------------------------

    get_corr_axis(hist1_axis2, hist2_axis2, corr_axis2, &translation_axis2);

    save_vector(corr_axis2, "corr_axis2.csv");

    if(atoi(argv[9])!=0)
    {
    translation_axis2=atoi(argv[9]);
    }

    float delta2=(float)(  (axis2_max-axis1_min)/(float)(N_hist)  );
    std::cout<<"movment for axis 2: "<<std::endl;
    std::cout<<std::endl<<"  x translation: "<<(translation_axis2-N_hist_axis+1)*delta2*cos(phi+M_PI/2)<<"  y translation: "<<(translation_axis2-N_hist_axis+1)*delta2*sin(phi+M_PI/2)<<"  z translation: "<<(translation_axis2-N_hist_axis+1)*delta2*cos(theta)<<std::endl<<std::endl;

    Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
    translation_transform(0,3)=(translation_axis2-N_hist_axis+1)*delta2*cos(phi+M_PI/2)+(translation_axis1-N_hist_axis+1)*delta1*cos(phi);
    translation_transform(1,3)=(translation_axis2-N_hist_axis+1)*delta2*sin(phi+M_PI/2)+(translation_axis1-N_hist_axis+1)*delta1*sin(phi);
    translation_transform(2,3)=(translation_axis2-N_hist_axis+1)*delta2*cos(theta)+(translation_axis1-N_hist_axis+1)*delta1*cos(theta);
    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();
    total_transform=transform_theta*transform_phi+translation_transform;
    std::cout<<"total transformation : "<<std::endl<<total_transform<<std::endl<<std::endl;

    std::string file_name;
    std::string file_name1;
    std::string file_name2;
    file_name=argv[1];
    size_t lastindex_point = file_name.find_last_of(".");
    size_t lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
       lastindex_slash = 0;
    }

    file_name1 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    file_name=argv[2];
    lastindex_point = file_name.find_last_of(".");
    lastindex_slash = file_name.find_last_of("/");
    if (lastindex_slash==std::string::npos)
    {
       lastindex_slash = 0;
    }
    file_name2 = file_name.substr(lastindex_slash+1, lastindex_point-(lastindex_slash+1));
    std::stringstream sstm;
    sstm.str("");
    sstm<<"transformations/"<<file_name1<<"_"<<file_name2<<".txt";
    std::string file_name_tot = sstm.str();
    ofstream file (file_name_tot);
    file<<total_transform;
    file.close();
}

