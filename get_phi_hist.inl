void get_phi_hist(pcl::PointCloud<pcl::PointNormal>::Ptr normals, std::vector<float>& hist)
{
          int N_hist=hist.size();
          float phi;

          for(int m=0; m<N_hist; m++)
          {
              hist[m]=0;
          }

          float delta=2*M_PI/N_hist;

          for (size_t i = 0; i < normals->points.size (); ++i)
          {
              float x_dot = normals->points[i].normal_x /sqrt(normals->points[i].normal_x*normals->points[i].normal_x+normals->points[i].normal_y*normals->points[i].normal_y);
              phi=acos(x_dot);

              if(phi==phi)
              {
                  if(normals->points[i].normal_y<0 && phi!=0)
                  {
                      phi=2*M_PI-phi;
                  }

                  int m=(int)(phi/delta);
                  hist[m]=hist[m]+1.0/(float)(normals->points.size ());
              }
          }
}
