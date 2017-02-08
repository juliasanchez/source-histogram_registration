void get_proj_axis(float axis1_min, float axis2_min, float axis1_max, float axis2_max,  pcl::PointCloud<pcl_point>::Ptr cloud, std::vector<std::vector<float>>& hist_axis)
{
      int N_hist=hist_axis.size();

      float v_axis1;
      float v_axis2;

      float delta1=(float)(  (axis1_max-axis1_min)/(float)(N_hist)  );
      float delta2=(float)(  (axis2_max-axis2_min)/(float)(N_hist)  );

      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
          v_axis1 = cloud->points[i].x;
          v_axis2 = cloud->points[i].y;
          int n=(int)(v_axis1/delta1+abs(axis1_min)/delta1);
          int m=(int)(v_axis2/delta2+abs(axis2_min)/delta2);
          hist_axis[n][m]=hist_axis[n][m]+1.0/(float)(cloud->points.size ());
      }
}
