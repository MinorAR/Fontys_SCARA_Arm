#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
  //load cloud a
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_a.pcd", cloud_a) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file cloud_a.pcd \n");
    return (-1);
  }
  //load cloud b
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_b.pcd", cloud_b) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file cloud_b.pcd \n");
    return (-1);
  }


  // merge the point cloud data
    cloud_c  = cloud_a;
    cloud_c += cloud_b;
  // print info of merged cloud
    std::cerr << "Cloud C: " << std::endl;
    for (size_t i = 0; i < cloud_c.points.size (); ++i)
      std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;


 // Write the merged cloud to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("cloud_merged.pcd", cloud_c, false);

  return (0);
}
