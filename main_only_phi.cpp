#include <iostream>
#include <string>

#include "cloud.h"
#include "display_normals.h"
#include "get_normal_hist.h"

typedef pcl::PointXYZI pcl_point;

int main(int argc, char *argv[])
{
    int N_hist=atoi(argv[5]);
    std::vector<std::vector<float>> hist1(N_hist, std::vector<float>(N_hist*2));
    std::vector<std::vector<float>> hist11(N_hist,std::vector<float>(N_hist*2));
    std::vector<std::vector<float>> hist2(N_hist,std::vector<float>(N_hist*2));
    float normal_radius =atof(argv[4]);
    float sample =atof(argv[3]);
    get_normal_hist(argv[1],sample,normal_radius, hist1,0);
    get_normal_hist(argv[2],sample,normal_radius, hist2,0);
    float error[N_hist*2];
    float temp=2000000;
    int p=0;
    int rotation;

    for (int k=0; k<N_hist*2; k++)
    {
        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<N_hist*2; m++)
            {
                if(m+k<N_hist*2)
                {
                    hist11[n][m+k]=hist1[n][m];
                    p=0;
                }
                else
                {
                    hist11[n][p]=hist1[n][m];
                    p++;
                }
            }
        }
        error[k]=0;
        for (int n=0; n<N_hist; n++)
        {
            for (int m=0; m<N_hist*2; m++)
            {
                error[k]=error[k]+(hist11[n][m]-hist2[n][m])*(hist11[n][m]-hist2[n][m]);
            }
        }
        if (temp>error[k])
        {
            temp=error[k];
            rotation=k;
        }
    }

    ofstream error_file ("error.csv");
    for(int k = 0; k < N_hist*2; k ++)
    {
        error_file <<error[k] << "\n" ;
    }
    error_file.close();

    ofstream hist1_file ("hist1.csv");
    ofstream hist2_file ("hist2.csv");
    for (int n=0; n<N_hist; n++)
    {
        for(int m=0; m<N_hist*2; m++)
        {
            hist1_file <<hist1[n][m] << "\n" ;
            hist2_file <<hist2[n][m] << "\n" ;
        }
    }
    hist1_file.close();
    hist2_file.close();

    std::cout<<"  rotation (degrés) : "<<(float)(rotation)*180.0/(float)(N_hist)<<"   error : "<<temp<<std::endl<<std::endl;

}
