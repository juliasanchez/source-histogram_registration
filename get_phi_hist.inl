void get_phi_hist(pcl::PointCloud<pcl::PointNormal>::Ptr normals, std::vector<std::vector<float>>& hist)
{
      int N_hist=hist.size();
      float theta;
      float phi;
      float z_dot=normals->points[0].normal_z;
      float x_dot=1;

      for (int n=0; n<N_hist; n++)
      {
          for(int m=0; m<N_hist*2; m++)
          {
              hist[n][m]=0;
          }
      }

      int done=0;
      float count=0;

      for (size_t i = 0; i < normals->points.size (); ++i)
      {
          float z_dot = normals->points[i].normal_z /sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y+normals->points[i].normal_z*normals->points[i].normal_z);
          theta=acos(z_dot);
          float x_dot = normals->points[i].normal_x /sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y);
          phi=acos(x_dot);
          if(normals->points[i].normal_y<0 && phi!=0)
          {
              phi=2*M_PI-phi;
          }
          done=0;
          for (int n = 0; n < N_hist; ++n)
          {
              if (  ((n+1)*M_PI/N_hist-theta)>0 && theta-n*M_PI/N_hist>=0 )
              {
                  for (int m = 0; m < N_hist*2; ++m)
                  {
                      if (   ((m+1)*M_PI/N_hist-phi)>0 && phi-m*M_PI/N_hist>=0    )
                      {
                      hist[n][m]=hist[n][m]+1.0/(float)(normals->points.size ());
                      done=1;
                      count=count +1.0/(float)(normals->points.size ());
                      break;
                      }
                      else if (m==N_hist*2-1)
                      {
//                          std::cout<<"can't order this normal :  x:"<<x_dot<<"   z: "<<z_dot<<std::endl<<std::endl;
                      }
                  }
              }
              else if (n==N_hist-1 && done==0)
              {
 //                 std::cout<<"can't order this normal :  indice:"<<i<<"   x: "<<x_dot<<"   z: "<<z_dot<<std::endl<<std::endl;
              }
              if(done)
              {break;}
           }
      }
}
