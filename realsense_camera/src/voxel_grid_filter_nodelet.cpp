/*
Purpose of nodelet: subscribe to a point cloud, use a VoxelGrid filter on it with a setting that
clobbers voxels with fewer than a threshold of points.
*/

#include <pluginlib/class_list_macros.h>
#include <realsense_camera/voxel_grid_filter_nodelet.h>

PLUGINLIB_EXPORT_CLASS(realsense_camera::FilterAndPublish, nodelet::Nodelet);

namespace realsense_camera
{
/*
    * Nodelet Constructor
    */
void FilterAndPublish::onInit()
{
    NODELET_DEBUG("Initializing voxel grid filter nodelet...");
    printf("Made object\n");
    nh = getNodeHandle();
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud_filtered", 1);
    sub = nh.subscribe<PointCloud>("pointcloud_raw", 1, &FilterAndPublish::pc_callback, this);
    this->thresh = 15; // This is the minimum number of points that have to occupy a voxel in order for it to survive the downsample.
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud ( std::vector< pcl::PointIndices > & clusters, PointCloud::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = cloud->width;
    colored_cloud->height = cloud->height;
    colored_cloud->is_dense = cloud->is_dense;
    for (size_t i_point = 0; i_point < cloud->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(cloud->points[i_point].data);
      point.y = *(cloud->points[i_point].data + 1);
      point.z = *(cloud->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters.begin (); i_segment != clusters.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

void FilterAndPublish::pc_callback(const PointCloud::ConstPtr &msg)
{
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    *cloud = *msg;

    // What to do here:
    // 1. Take cloud and put it in a voxel grid while restricting the bounding box
    // 2. Go through the voxels and remove all points in a voxel that has less than this.thresh points
    // 3. Publish resulting cloud

    pcl::VoxelGrid<PointXYZ> vox;
    vox.setInputCloud(cloud);
    // The leaf size is the size of voxels pretty much. Note that this value affects what a good threshold value would be.
    vox.setLeafSize(0.05f, 0.05f, 0.05f);
    // I limit the overall volume being considered so lots of "far away" data that is just terrible doesn't even have to be considered.
    vox.setFilterFieldName("z");
    vox.setFilterLimits(.5, 3.0);
    // The line below is perhaps the most important as it reduces ghost points.
    vox.setMinimumPointsNumberPerVoxel(this->thresh);
    vox.filter(*cloud_filtered);
    //pub.publish(cloud_filtered);
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    pcl::EuclideanClusterExtraction<PointXYZ> ece;
    std::vector<pcl::PointIndices> clusters;
    ece.setInputCloud(cloud_filtered);
    ece.setMinClusterSize(10);
    ece.setClusterTolerance(0.1);
    ece.extract(clusters);
   // ROS_INFO_STREAM("No of clusters: " << clusters.size() << std::endl);

    // Define Coloured cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    colored_cloud = getColoredCloud(clusters, cloud_filtered);
    colored_cloud->header=cloud->header;

    pub.publish(colored_cloud);
    
}
};

/* TODO: Make a node out of this using nodelet loader
int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxel_grid_filter");
    FilterAndPublish f = FilterAndPublish();
    ros::spin();
    return 0;
}
*/