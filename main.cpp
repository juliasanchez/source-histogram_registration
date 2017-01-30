#include <iostream>
#include <string>

#include "cloud.h"
#include "display_normals.h"
#include "get_phi_hist.h"
#include "get_proj_axis.h"
#include "pre_process.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ pcl_point;

int main(int argc, char *argv[])
{
    int N_hist=atoi(argv[5]);

    std::vector<std::vector<float>> hist1(N_hist, std::vector<float>(N_hist*2));
    std::vector<float> hist1_phi(N_hist*2);
    std::vector<std::vector<float>> hist1_theta(N_hist, std::vector<float>(N_hist*2));
    std::vector<float> hist11_phi(N_hist*2);
    std::vector<std::vector<float>> hist11_theta(N_hist, std::vector<float>(N_hist*2));

    std::vector<std::vector<float>> hist2(N_hist,std::vector<float>(N_hist*2));
    std::vector<float> hist2_phi(N_hist*2);
    std::vector<std::vector<float>> hist2_theta(N_hist, std::vector<float>(N_hist*2));

    float normal_radius =atof(argv[4]);
    float sample =atof(argv[3]);
    float error_phi[N_hist*2];
    float error_theta[N_hist*2];
    float temp_phi=2000000;
    float temp_theta=20000000;
    int p=0;
    int rotation_phi;
    int rotation_theta;

    ///preprocess cloud--------------------------------------------------------------------------------------------------------------

    pcl::PointCloud<pcl_point>::Ptr cloud_src(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_src (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl_point>::Ptr cloud_tgt(new pcl::PointCloud<pcl_point>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_tgt (new pcl::PointCloud<pcl::Normal>);

    pre_process(argv[1],sample,normal_radius, 0, cloud_src, normals_src);
    pre_process(argv[2],sample,normal_radius, 0, cloud_tgt, normals_tgt);

//    pcl::io::savePCDFileASCII ("pre_processed_source.pcd", *cloud_src);
//    pcl::io::savePCDFileASCII ("pre_processed_tgt.pcd", *cloud_tgt);


    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals_tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_src, *normals_src, *pointNormals_src);
    pcl::concatenateFields (*cloud_tgt, *normals_tgt, *pointNormals_tgt);


    ///compute 2D histograms (phi/theta)--------------------------------------------------------------------------------------------------------------

    get_phi_hist(pointNormals_src,hist1);
    get_phi_hist(pointNormals_tgt,hist2);

    ///save 2D histograms--------------------------------------------------------------------------------------------------------------

    ofstream hist1_file ("hist1.csv");
    ofstream hist2_file ("hist2.csv");
    for (int n=0; n<N_hist; n++)
    {
        for(int m=0; m<N_hist*2; m++)
        {
            hist1_file <<hist1[n][m] << " " ;
            hist2_file <<hist2[n][m] << " " ;
        }
        hist1_file << "\n" ;
        hist2_file <<"\n" ;
    }
    hist1_file.close();
    hist2_file.close();

    ///compute projection histogram on x,y plane--------------------------------------------------------------------------------------------------------------

    for (int m=0; m<N_hist*2; m++)
    {
        hist1_phi[m]=0;
        hist2_phi[m]=0;
    }

    for (int n=0; n<N_hist; n++)
    {
        for (int m=0; m<N_hist*2; m++)
        {
            hist1_phi[m]=hist1_phi[m]+hist1[n][m];
            hist2_phi[m]=hist2_phi[m]+hist2[n][m];
        }
    }

    ///compute error function for phi--------------------------------------------------------------------------------------------------------------

    for (int k=0; k<N_hist*2; k++)
    {
        //turn object of delta_phi

        for (int m=0; m<N_hist*2; m++)
        {
            if(m+k<N_hist*2)
            {
                hist11_phi[m+k]=hist1_phi[m];
                p=0;
            }
            else
            {
                hist11_phi[p]=hist1_phi[m];
                p++;
            }
        }

        //compute error and get minimum

        error_phi[k]=0;

        for (int m=0; m<N_hist*2; m++)
        {
            error_phi[k]=error_phi[k]+(hist11_phi[m]-hist2_phi[m])*(hist11_phi[m]-hist2_phi[m]);
        }

        if (temp_phi>error_phi[k])
        {
            temp_phi=error_phi[k];
            rotation_phi=k;
        }
    }

    ///TURN CLOUD TO ALIGN PROJECTIONS AND TURN THE TWO CLOUDS OF 90°--------------------------------------------------------------------------------------------------------------

    //turn around x axis y becomes z and z becomesy. theta is the angle of rotation around y
    Eigen::Matrix4f transform_90 = Eigen::Matrix4f::Identity();
    transform_90 (1,2) = -1;
    transform_90 (2,1) = 1;
    transform_90 (1,1) = 0;
    transform_90 (2,2) = 0;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    float phi = (float)(rotation_phi)*M_PI/(float)(N_hist);
   // float phi=5.59596191421;
    transform (0,0) = cos (phi);
    transform (0,1) = -sin(phi);
    transform (1,0) = sin (phi);
    transform (1,1) = cos (phi);

    pcl::transformPointCloudWithNormals (*pointNormals_src, *pointNormals_src, transform_90*transform);
    pcl::transformPointCloudWithNormals (*pointNormals_tgt, *pointNormals_tgt, transform_90);
//    pcl::io::savePCDFileASCII ("source_transformed.pcd", *pointNormals_src);
//    pcl::io::savePCDFileASCII ("target_transformed.pcd", *pointNormals_tgt);

    ///compute 2D histograms (phi/theta)--------------------------------------------------------------------------------------------------------------

    get_phi_hist(pointNormals_src,hist1_theta);
    get_phi_hist(pointNormals_tgt,hist2_theta);

    ///compute error function for theta--------------------------------------------------------------------------------------------------------------

    for (int k=0; k<N_hist*2; k++)
    {

        //turn object of delta_theta

        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<N_hist*2; m++)
            {
                if(m+k<N_hist*2)
                {
                    hist11_theta[n][m+k]=hist1_theta[n][m];
                    p=0;
                }
                else
                {
                    hist11_theta[n][p]=hist1_theta[n][m];
                    p++;
                }
            }
        }

        //compute error and get minimum

        error_theta[k]=0;
        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<N_hist*2; m++)
            {
                error_theta[k]=error_theta[k]+(hist11_theta[n][m]-hist2_theta[n][m])*(hist11_theta[n][m]-hist2_theta[n][m]);
            }
        }
        if (temp_theta>error_theta[k])
        {
            temp_theta=error_theta[k];
            rotation_theta=k;
        }
    }

    ofstream error_phi_file ("error_phi.csv");
    for(int k = 0; k < N_hist*2; k ++)
    {
        error_phi_file <<error_phi[k] << "\n" ;
    }
    error_phi_file.close();

    ofstream error_theta_file ("error_theta.csv");
    for(int k = 0; k < N_hist; k ++)
    {
        error_theta_file <<error_theta[k] << "\n" ;
    }
    error_theta_file.close();

    std::cout<<"  phi rotation  : "<<std::endl;

    for (int k =0; k<4; k++)
    {
        if((float)(rotation_phi)*180.0/(float)(N_hist)+k*90<360)
        {
          std::cout<<"  -->"<<(float)(rotation_phi)*180.0/(float)(N_hist)+k*90<<"  degrés"<<std::endl;
        }
        else
        {
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

 ///rotate cloud in the accurate way

    Eigen::Matrix4f transform_phi = Eigen::Matrix4f::Identity();
    phi=5.59596191421;
    //phi=(float)(rotation_phi)*M_PI/(float)(N_hist);
    transform_phi (0,0) = cos (phi);
    transform_phi (0,1) = -sin(phi);
    transform_phi (1,0) = sin (phi);
    transform_phi (1,1) = cos (phi);
    std::cout<<transform_phi<<std::endl<<std::endl;
    Eigen::Matrix4f transform_theta = Eigen::Matrix4f::Identity();
    float theta=0;
    transform_theta (0,0) = cos (theta);
    transform_theta (0,2) = sin(theta);
    transform_theta (2,0) = -sin (theta);
    transform_theta (2,2) = cos (theta);
    std::cout<<transform_theta<<std::endl<<std::endl;

//    pcl::io::savePCDFileASCII ("cloud_src_before.pcd", *cloud_src);
    std::cout<<transform_theta*transform_phi<<std::endl<<std::endl;
    pcl::transformPointCloud (*cloud_src, *cloud_src, transform_theta*transform_phi);
//    pcl::io::savePCDFileASCII ("transformed_cloud_src.pcd", *cloud_src);

    ///compute axis histogram to get plans
    ///
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

    float angles1[]={phi, theta};
    float angles2[]={phi+M_PI/2, theta};
    float axis1_max=atof(argv[6]);
    float axis2_max=axis1_max;

    int N_hist_axis=atoi(argv[7]);
    std::vector<std::vector<float>> hist1_axis(N_hist_axis,std::vector<float>(N_hist_axis));
    std::vector<std::vector<float>> hist2_axis(N_hist_axis,std::vector<float>(N_hist_axis));
    std::vector<float> axis1={cos(angles1[0]), sin(angles1[0]), 0};
    std::vector<float> axis2={cos(angles2[0]), sin(angles2[0]), 0};

    ofstream axis1_file ("axis1.csv");
    for(int k = 0; k < 3; k ++)
    {
        axis1_file <<axis1[k] << "," ;
    }
    axis1_file.close();

    ofstream axis2_file ("axis2.csv");
    for(int k = 0; k < 3; k ++)
    {
        axis2_file <<axis2[k] << "," ;
    }
    axis2_file.close();

    get_proj_axis(axis1, axis2, axis1_max, axis2_max,  cloud_src, hist1_axis);
    get_proj_axis(axis1, axis2, axis1_max, axis2_max,  cloud_tgt, hist2_axis);

    ///save 2D histograms--------------------------------------------------------------------------------------------------------------

    ofstream hist1_axis_file ("hist1_axis.csv");
    ofstream hist2_axis_file ("hist2_axis.csv");
    for (int n=0; n<N_hist_axis; n++)
    {
        for(int m=0; m<N_hist_axis; m++)
        {
            hist1_axis_file <<hist1_axis[n][m] << " " ;
            hist2_axis_file <<hist2_axis[n][m] << " " ;
        }
        hist1_axis_file << "\n" ;
        hist2_axis_file <<"\n" ;
    }
    hist1_axis_file.close();
    hist2_axis_file.close();

    ///compute axis independent histograms--------------------------------------------------------------------------------------------------------------

    std::vector<float> hist1_axis1(N_hist_axis);
    std::vector<float> hist1_axis11(3*N_hist_axis-2);
    std::vector<float> hist2_axis1(N_hist_axis);
    std::vector<float> hist2_axis12(3*N_hist_axis-2);
    std::vector<float> hist1_axis2(N_hist_axis);
    std::vector<float> hist1_axis21(3*N_hist_axis-2);
    std::vector<float> hist2_axis2(N_hist_axis);
    std::vector<float> hist2_axis22(3*N_hist_axis-2);

    float error_axis1[2*N_hist_axis-1];
    float error_axis2[2*N_hist_axis-1];

    int translation_axis1;
    int translation_axis2;

    for (int m=0; m<N_hist_axis; m++)
    {
        hist1_axis1[m]=0;
        hist2_axis1[m]=0;
        hist1_axis2[m]=0;
        hist2_axis2[m]=0;
    }

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

    p=0;
    for (int m=0; m<3*N_hist_axis-2; m++)
    {
        hist1_axis11[m]=0;
        hist1_axis21[m]=0;
        hist2_axis12[m]=0;
        hist2_axis22[m]=0;
        if(m>=N_hist_axis && m<2*N_hist)
        {
            hist2_axis12[m]=hist2_axis1[p];
            hist2_axis22[m]=hist2_axis2[p];
            p++;
        }
    }

    ///compute error function for axis1--------------------------------------------------------------------------------------------------------------
    p=0;
    temp=0;
    error_axis1[0]=0;
    for (int k=1; k<2*N_hist_axis-1; k++)
    {
        //move object of delta_axis1

        hist1_axis11[k-1]=0;
        for (int n=0; n<N_hist_axis; n++)
        {
            if(k<=N_hist_axis)
            {hist1_axis11[n+k]=hist1_axis1[n]/k;}
            else
            {hist1_axis11[n+k]=hist1_axis1[n]/(2*N_hist_axis-k);}
        }

        //compute error and get minimum

        error_axis1[k]=0;

        for (int n=0; n<3*N_hist_axis-2; n++)
        {
            error_axis1[k]=error_axis1[k]+hist1_axis11[n]*hist2_axis12[n];
        }

        if (temp<error_axis1[k])
        {
            temp=error_axis1[k];
            translation_axis1=k;
        }
    }

    ofstream error_axis1_file ("error_axis1.csv");
    for(int k = 1; k < 2*N_hist_axis-1; k ++)
    {
        error_axis1_file <<error_axis1[k] << "\n" ;
    }
    error_axis1_file.close();

    if(atoi(argv[8])!=0)
    {
    translation_axis1=atoi(argv[8]);
    }

    float delta1=(float)(2*axis1_max/(float)(N_hist_axis));
    std::cout<<"movment for axis 1: "<<std::endl;
    std::cout<<std::endl<<"  x translation: "<<(translation_axis1-N_hist_axis)*delta1*cos(phi)<<"  y translation: "<<(translation_axis1-N_hist_axis)*delta1*sin(phi)<<"  z translation: "<<(translation_axis1-N_hist_axis)*delta1*cos(theta)<<std::endl;


    ///compute error function for axis2--------------------------------------------------------------------------------------------------------------
    p=0;
    temp=0;
    error_axis2[0]=0;
    for (int k=1; k<2*N_hist_axis-1; k++)
    {
        //move object of delta_axis1

        hist1_axis21[k-1]=0;
        for (int m=0; m<N_hist_axis; m++)
        {
            if(k<=N_hist_axis)
            {hist1_axis21[m+k]=hist1_axis2[m]/k;}
            else
            {hist1_axis21[m+k]=hist1_axis2[m]/(2*N_hist_axis-k);}
        }

        //compute error and get minimum

        error_axis2[k]=0;

        for (int m=0; m<3*N_hist_axis-2; m++)
        {
            error_axis2[k]=error_axis2[k]+hist1_axis21[m]*hist2_axis22[m];
        }

        if (temp<error_axis2[k])
        {
            temp=error_axis2[k];
            translation_axis2=k;
        }
    }

    ofstream error_axis2_file ("error_axis2.csv");
    for(int k = 1; k < 2*N_hist_axis-1; k ++)
    {
        error_axis2_file <<error_axis2[k] << "\n" ;
    }
    error_axis2_file.close();

    if(atoi(argv[9])!=0)
    {
    translation_axis2=atoi(argv[9]);
    }

    float delta2=(float)(2*axis2_max/(float)(N_hist_axis));
    std::cout<<"movment for axis 2: "<<std::endl;
    std::cout<<std::endl<<"  x translation: "<<(translation_axis2-N_hist_axis)*delta2*cos(phi+M_PI/2)<<"  y translation: "<<(translation_axis2-N_hist_axis)*delta2*sin(phi+M_PI/2)<<"  z translation: "<<(translation_axis2-N_hist_axis)*delta2*cos(theta)<<std::endl;

    Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Zero();
    translation_transform(0,3)=(translation_axis2-N_hist_axis)*delta2*cos(phi+M_PI/2)+(translation_axis1-N_hist_axis)*delta1*cos(phi);
    translation_transform(1,3)=(translation_axis2-N_hist_axis)*delta2*sin(phi+M_PI/2)+(translation_axis1-N_hist_axis)*delta1*sin(phi);
    translation_transform(2,3)=(translation_axis2-N_hist_axis)*delta2*cos(theta)+(translation_axis1-N_hist_axis)*delta1*cos(theta);
    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();
    total_transform=transform_theta*transform_phi+translation_transform;
    std::cout<<"total transformation : "<<std::endl<<total_transform<<std::endl<<std::endl;


}

