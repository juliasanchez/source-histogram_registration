#include <iostream>
#include <string>
#include <math.h>
#include <time.h>

#include "cloud.h"
#include "display_normals.h"
#include "get_phi_hist.h"
#include "pre_process.h"
#include "get_error_phi.h"
#include "get_error_theta.h"
#include "save_vector.h"
#include "save_axis.h"
#include "get_translation.h"
#include "get_LCP.h"
#include "get_angles_hist.h"


typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    ///preprocess clouds--------------------------------------------------------------------------------------------------------------

    clock_t t;
    clock_t t_tot=clock();
    float f=CLOCKS_PER_SEC;

    float normal_radius =atof(argv[4]);
    float sample =atof(argv[3]);
    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);

    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    Eigen::Matrix4f transform_init = Eigen::Matrix4f::Identity();

    pre_process(argv[1],sample,normal_radius, 0, 10, cloud_src, transform_init, normals_src);

//    std::vector<int> indices;
//    pcl::removeNaNNormalsFromPointCloud(*normals_src, *normals_src, indices);
//    pcl::io::savePCDFileASCII ("normals.pcd", *normals_src);

    Eigen::Matrix4f transform_phi_init = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_theta_init = Eigen::Matrix4f::Identity();

    float theta_init =  atof(argv[8])*M_PI/180;
    float phi_init=  atof(argv[9])*M_PI/180;
    std::vector<float> rot_ax={cloud_src->points[0].y, -cloud_src->points[0].x,0};
    transform_theta_init  (0,0) = rot_ax[0]*rot_ax[0]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (0,1) = rot_ax[0]*rot_ax[1]*(1-cos (theta_init))-rot_ax[2]*sin (theta_init);
    transform_theta_init  (0,2) = rot_ax[0]*rot_ax[2]*(1-cos (theta_init))+rot_ax[1]*sin (theta_init);
    transform_theta_init  (1,0) = rot_ax[1]*rot_ax[0]*(1-cos (theta_init))+rot_ax[2]*sin (theta_init);
    transform_theta_init  (1,1) = rot_ax[1]*rot_ax[1]*(1-cos (theta_init))+cos (theta_init);
    transform_theta_init  (1,2) = rot_ax[1]*rot_ax[2]*(1-cos (theta_init))-rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,0) = rot_ax[2]*rot_ax[0]*(1-cos (theta_init))-rot_ax[1]*sin (theta_init);
    transform_theta_init  (2,1) = rot_ax[2]*rot_ax[1]*(1-cos (theta_init))+rot_ax[0]*sin (theta_init);
    transform_theta_init  (2,2) = rot_ax[2]*rot_ax[2]*(1-cos (theta_init))+cos (theta_init);

    transform_phi_init  (0,0) = cos (phi_init);
    transform_phi_init  (0,1) = -sin(phi_init);
    transform_phi_init  (1,0) = sin(phi_init);
    transform_phi_init  (1,1) = cos(phi_init);


    transform_init=transform_phi_init*transform_theta_init;

    // Print the transformation
    std::cout << "initial transform : "<<std::endl<<transform_init << std::endl;

    pre_process(argv[2],sample,normal_radius, 0, 10, cloud_tgt,  transform_init, normals_tgt);


    pcl::io::savePCDFileASCII ("preprocess_src.pcd", *cloud_src);
    pcl::io::savePCDFileASCII ("preprocess_tgt.pcd", *cloud_tgt);

    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal> pointNormals_src_init;
    pcl::PointCloud<pcl::PointNormal> pointNormals_tgt_init;
    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);

    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_src, *pointNormals_src, indices);
    pcl::removeNaNNormalsFromPointCloud(*pointNormals_tgt, *pointNormals_tgt, indices);

    pcl::io::savePCDFileASCII ("src_with_normals.pcd", *pointNormals_src);
    pcl::io::savePCDFileASCII ("tgt_with_normals.pcd", *pointNormals_tgt);

    pointNormals_src_init=*pointNormals_src;
    pointNormals_tgt_init=*pointNormals_tgt;

    ///compute 2D histograms (phi/theta)--------------------------------------------------------------------------------------------------------------

    //parameters for normal histograms...........................................................................................................

    int N_hist=atoi(argv[5]);

    std::vector<float> hist1_phi(N_hist, 0.0);
    std::vector<float> hist2_phi(N_hist, 0.0);

    std::vector<float> error_phi(N_hist, 0.0);

    int rotation_phi;

    get_phi_hist(pointNormals_src,hist1_phi);
    get_phi_hist(pointNormals_tgt,hist2_phi);

    save_vector(hist1_phi, "hist1_phi.csv");
    save_vector(hist2_phi, "hist2_phi.csv");

    ///compute error function for phi--------------------------------------------------------------------------------------------------------------

    t = clock();
    get_error_phi(hist1_phi, hist2_phi, error_phi, &rotation_phi);
    t = clock()-t;
    std::cout<<"get_error_phi : "<<((float)t)/CLOCKS_PER_SEC<<" seconds"<<std::endl<<std::endl;

    save_vector(error_phi, "error_phi.csv");

    float delta_phi= 2*M_PI/(float)(N_hist);

    float phi=(float)(rotation_phi)*delta_phi;
    std::cout<<"PHI = "<<phi*180/M_PI<<std::endl<<std::endl;

    std::cout<<"----------------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl<<std::endl;

    ///TURN CLOUD TO ALIGN PROJECTIONS AND TURN THE TWO CLOUDS OF PHI--------------------------------------------------------------------------------------------------------------

    int tmp=0;
    int idx=0;
    int LCP=0;

    Eigen::Matrix4f good_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();


    float phi1;
    float theta1;
    float phi0;
    float theta0;

//    std::vector<std::vector<float>> hist1_angles((int)(N_hist/2), std::vector<float>(N_hist, 0.0));
//    get_angles_hist(pointNormals_src,hist1_angles);
    std::vector<std::vector<float>> hist2_angles((int)(N_hist/2), std::vector<float>(N_hist, 0.0));
    get_angles_hist(pointNormals_tgt,hist2_angles);

    for (int q=0; q<4; q++)
    {
        ///initialize cloud_src cloud_tgt and phi--------------------------------------------------------------------------------------------------------------------------------------

        pcl::copyPointCloud(pointNormals_src_init,*pointNormals_src);
        pcl::copyPointCloud(pointNormals_tgt_init,*pointNormals_tgt);

         phi=(float)(rotation_phi)*delta_phi + q*M_PI/2;
         if(phi>2*M_PI)
         {
             phi=phi-2*M_PI;
         }

         std::cout<<"PHI = "<<phi*180/M_PI<<std::endl<<std::endl;

     ///rotate cloud in the accurate way with rotation information--------------------------------------------------------------------------------------------------------------------------------------

        Eigen::Matrix4f rotation_transform = Eigen::Matrix4f::Identity();
        transform(phi, 0, &rotation_transform);

        std::cout<<rotation_transform<<std::endl<<std::endl;

        pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, rotation_transform);

     /// get 2 main axis (2 main walls) with the average of normals from source and target------------------------------------------------------------------------------------------------------------------------------------------------

        // first axis

        std::vector<std::vector<float>> hist1_angles((int)(N_hist/2), std::vector<float>(N_hist, 0.0));
        get_angles_hist(pointNormals_src,hist1_angles);

        float temp=0;
        for (int n=10*(int)(N_hist/360); n<(int)(N_hist/2)-10*(int)(N_hist/360); n++) // hypothesis 1 : z axis deflected from local frame from at most 10*(int)(N_hist/360)=10°
        {
            for (int m=0; m<N_hist; m++)
            {
                if(temp<hist1_angles[n][m]+hist2_angles[n][m])
                {
                    temp=hist1_angles[n][m]+hist2_angles[n][m];
                    phi0=(float)(m)*delta_phi;
                    theta0=(float)(n)*delta_phi;
                }
            }
        }

        if(phi0>2*M_PI)
        {
            phi0=phi0-2*M_PI;
        }

        // second axis

        temp=0;
        float phi00;
        phi00=phi0+M_PI;

        if(phi00>2*M_PI)
        {
            phi00=phi00-2*M_PI;
        }

        for (int n=10*(int)(N_hist/360); n<(int)(N_hist/2)-10*(int)(N_hist/360); n++)
        {

            for (int m=0; m<N_hist; m++)
            {
                if(temp<hist1_angles[n][m]+hist2_angles[n][m] && ( abs((float)(m)*delta_phi-phi0)>0.4) &&  ( abs((float)(m)*delta_phi-phi00)>0.4) )
                {
                    temp=hist1_angles[n][m]+hist2_angles[n][m];
                    phi1=(float)(m)*delta_phi;
                    theta1=(float)(n)*delta_phi;
                }
            }
        }

        //compute angles of these normals

        std::vector<std::vector<float>> angles(3, std::vector<float>(2, 0.0));
        angles[0][0]=phi0+delta_phi/2;
        angles[0][1]=theta0+delta_phi/2;
        angles[1][0]=phi1+delta_phi/2;
        angles[1][1]=theta1+delta_phi/2;

        std::vector<std::vector<float>> axis(3, std::vector<float>(3, 0.0)); //columns = axis 1 axis 2 axis 3 __ rows = x, y ,z
        axis[0][0]=cos(angles[0][0])*sin(angles[0][1]);
        axis[0][1]=sin(angles[0][0])*sin(angles[0][1]);
        axis[0][2]=cos(angles[0][1]);
        axis[1][0]=cos(angles[1][0])*sin(angles[1][1]);
        axis[1][1]=sin(angles[1][0])*sin(angles[1][1]);
        axis[1][2]=cos(angles[1][1]);
        axis[2][0]=axis[0][1]*axis[1][2]-axis[0][2]*axis[1][1];
        axis[2][1]=-axis[0][0]*axis[1][2]+axis[0][2]*axis[1][0];
        axis[2][2]=axis[0][0]*axis[1][1]-axis[0][1]*axis[1][0];

        angles[2][0]=acos( axis[2][0]/(axis[2][0]*axis[2][0]+axis[2][1]*axis[2][1]) ); // hypothesis 2 : Walls perp to ground and roof : The third axis is the cross product of the two first ones.
        angles[2][1]=acos(axis[2][2]);

        save_axis(axis[0], "axis_x.csv");
        save_axis(axis[1], "axis_y.csv");
        save_axis(axis[2], "axis_z.csv");

        ///get translation with histograms correlation

        Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
        int N_hist_axis=atoi(argv[6]);
        float lim = atof(argv[7]);

        t = clock();
        get_translation(pointNormals_src, pointNormals_tgt, lim, axis, N_hist_axis, &translation_transform) ;
        t = clock()-t;
        std::cout<<"get_translation : "<<((float)t)/CLOCKS_PER_SEC<<" seconds"<<std::endl<<std::endl;


        total_transform=rotation_transform+translation_transform;
        std::cout<<"total transformation : "<<std::endl<<total_transform<<std::endl<<std::endl;

        std::cout<<"----------------------------------------------------------------------------------------------------------------------------------------------------"<<std::endl<<std::endl;

        ///gcompute LCP for this transformation

        get_LCP(*cloud_src, *cloud_tgt, &total_transform, &LCP);

        if(tmp<LCP)
        {
            tmp=LCP;
            idx=q;
            good_transform=total_transform;
        }
     }

        ///save best transformation in build/transformations
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
        file<<good_transform;
        file.close();

        t_tot=clock()-t_tot;
        std::cout<<"total time to get transform :" <<((float)t_tot)/CLOCKS_PER_SEC<<" seconds"<<std::endl<<std::endl;

        std::cout<<"best transformation : "<<std::endl<<good_transform<<std::endl<<std::endl;


}

