void get_walls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, float lim, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    float dot1;
    float dot2;
    for (int i=0; i<cloud_in->points.size(); i++)
    {
        int p=0;
        dot1=cloud_in->points[i].normal_x; //*axis1[0]+cloud_in->points[i].normal_y*axis1[1]+cloud_in->points[i].normal_z*axis1[2];
        dot2=cloud_in->points[i].normal_y; //*axis2[0]+cloud_in->points[i].normal_y*axis2[1]+cloud_in->points[i].normal_z*axis2[2];

        if( (abs(dot1)<lim || abs(dot2)<lim) && ( (1-abs(dot1))<lim || ( 1-abs(dot2) )<lim ) )
        {
            pcl::PointXYZ point;
            point.x=cloud_in->points[i].x;
            point.y=cloud_in->points[i].y;
            point.z=cloud_in->points[i].z;
            cloud_filtered->points.push_back(point);
        }
    }
    cloud_filtered->width = (uint32_t)  cloud_filtered->points.size();
    cloud_filtered->height = 1;
}
