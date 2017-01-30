void get_proj_axis(std::vector<float>& axis1, std::vector<float>& axis2, float axis1_max, float axis2_max,  pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<std::vector<float>>& hist_axis)
{
      int N_hist=hist_axis.size();

      float v_axis1;
      float v_axis2;

      for (int n=0; n<N_hist; n++)
      {
          for(int m=0; m<N_hist; m++)
          {
              hist_axis[n][m]=0;
          }
      }

      int done=0;
      float delta1=(float)(2*axis1_max/(float)(N_hist));
      float delta2=(float)(2*axis2_max/(float)(N_hist));

      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
          v_axis1 = cloud->points[i].x*axis1[0]+cloud->points[i].y*axis1[1]+cloud->points[i].z*axis1[2];
	  v_axis2 = cloud->points[i].x*axis2[0]+cloud->points[i].y*axis2[1]+cloud->points[i].z*axis2[2];

          done=0;
          for (int n = 0; n < N_hist; ++n)
          {
              if (  v_axis1-( (n+1)*delta1-axis1_max) <0 && v_axis1-(n*delta1-axis1_max)>=0 )
              {
                  for (int m = 0; m < N_hist; ++m)
                  {
                      if (   v_axis2-( (m+1)*delta2-axis2_max) <0 && v_axis2-(m*delta2-axis2_max)>=0    )
                      {
                      hist_axis[n][m]=hist_axis[n][m]+1.0/(float)(cloud->points.size ());
                      done=1;
                      break;
                      }
                      else if (m==N_hist-1)
                      {
                          std::cout<<"can't order this point :  v_axis1:"<<v_axis1<<"   v_axis2: "<<v_axis2<<std::endl<<std::endl;
                      }
                  }
              }
              else if (n==N_hist-1 && done==0)
              {
                  std::cout<<"can't order this point :  indice:"<<i<<"   v_axis1: "<<v_axis1<<"   v_axis2: "<<v_axis2<<std::endl<<std::endl;
              }
              if(done)
              {break;}
           }
      }
}
