#include <ros/ros.h>
// PCL specific includes
#include <skydio_proxy/VoxelOccupancyRunLengthEncoded.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>
#include <tf/transform_listener.h>

using namespace Eigen;

ros::Publisher voxel_pub;
ros::Publisher filtered_voxel_pub;
tf::TransformListener* tf_listener = nullptr;
uint32_t seq = 0;
std::string vehicle_frame("/skydio");
std::string nav_frame("/nav");

void VoxelRleCallback (const skydio_proxy::VoxelOccupancyRunLengthEncoded& voxel_rle) {
  // Create a container for the data.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  auto& points = cloud.points;

  if (!tf_listener) {
    return;
  }

  tf::StampedTransform vehicle_transform;
  try {
    tf_listener->lookupTransform(nav_frame, vehicle_frame, ros::Time(0), vehicle_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  const double scale = voxel_rle.voxel_header.scale;
  const Vector3d voxel_origin_floor = Vector3d::Zero();

  const uint32_t width = voxel_rle.voxel_header.dims[0];
  const uint32_t length = voxel_rle.voxel_header.dims[1];
  const uint32_t height = voxel_rle.voxel_header.dims[2];

  const Array3i dims = (Vector3i() << width, length, height).finished();
  const Array3i origin = (Vector3i() << voxel_rle.voxel_header.origin[0],
                                        voxel_rle.voxel_header.origin[1],
                                        voxel_rle.voxel_header.origin[2]).finished();

  auto IndexToVoxel = [origin, dims](uint32_t index) -> Array3i {
    Array3i canonical_voxel;
    const uint32_t plane_size = dims[0] * dims[1];
    canonical_voxel[2] = index / plane_size;
    index -= canonical_voxel[2] * plane_size;
    canonical_voxel[1] = index / dims[0];
    index -= canonical_voxel[1] * dims[0];
    canonical_voxel[0] = index;

    // voxel in same macro block as origin
    const Array3i block_offset =
        (origin.cast<double>() / dims.cast<double>()).floor().cast<int>() * dims;
    const Array3i voxel_in_origin_block = canonical_voxel + block_offset;
    const Array3i voxel =
        voxel_in_origin_block + (voxel_in_origin_block < origin).cast<int>() * dims;

    return voxel;
  };

  auto VoxelCenterToWorld = [scale] (Array3i voxel) -> Vector3d {
    return ((voxel.cast<double>() + 0.5) * scale).matrix();
  };

  // Standard voxel scale in meters. When a map has a larger scale, we publish more voxels of the
  // minimum voxel size. This might result in huge point clouds when flying at high speeds, so a
  // better method of handling voxel scaling should be implemented.
  const double map_scale = 0.125;

  const uint32_t upsample = scale / map_scale;
  uint32_t counter = 0;
  bool current_run_is_occupied = false;
  for (auto it = voxel_rle.runs.begin(); it != voxel_rle.runs.end(); ++it) {
    const int32_t runs = *it;

    // Add voxels if we are decoding "ones"
    for (uint32_t j = 0; j < runs; ++j) {
      const Array3i voxel = IndexToVoxel(counter + j);
      const Vector3d point = VoxelCenterToWorld(voxel) - voxel_origin_floor;
      const Vector3d edge = point - map_scale * upsample * Vector3d::Identity();
      if (current_run_is_occupied) {

        for (size_t x = 0; x < upsample; ++x) {
          for (size_t y = 0; y < upsample; ++y) {
            for (size_t z = 0; z < upsample; ++z) {
              points.push_back(pcl::PointXYZ(edge[0] + (x + 0.5) * map_scale,
                                             edge[1] + (y + 0.5) * map_scale,
                                             edge[2] + (z + 0.5) * map_scale));
            }
          }
        }
      }
    }
    counter += runs;
    current_run_is_occupied = !current_run_is_occupied;
  }

  // Publish the occupied data.
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "nav";
  output.header.seq = seq;
  voxel_pub.publish(output);

  // Publish the filtered map.
  // NOTE(teo): these are fixed for now, should go into node params.
  auto vehicle_origin = vehicle_transform.getOrigin();
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  const pcl::PointCloud<pcl::PointXYZ>& input_cloud = cloud;
  for (auto it = input_cloud.begin(); it != input_cloud.end(); ++it) {
    if ((it->z < vehicle_origin.z() - 7.0) || (it->z > vehicle_origin.z() + 0.35) ||
        (it->x < vehicle_origin.x() - 7.0) || (it->x > vehicle_origin.x() + 7.0) ||
        (it->y < vehicle_origin.y() - 7.0) || (it->y > vehicle_origin.y() + 7.0)) {
      continue;
    }
    filtered_cloud.push_back(*it);
  }

  sensor_msgs::PointCloud2 filtered_output;
  pcl::toROSMsg(filtered_cloud, filtered_output);
  filtered_output.header.stamp = ros::Time::now();
  filtered_output.header.frame_id = "nav";
  filtered_output.header.seq = seq;
  filtered_voxel_pub.publish(filtered_output);

  ++seq;
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "voxel_republisher");
  ros::NodeHandle nh("~");

  // Create a ROS subscriber for the input voxel occupancy RLE
  std::string voxel_rle_topic;
  std::string voxel_map_topic;
  nh.param<std::string>("voxel_rle_topic", voxel_rle_topic, "/skydio/voxel_rle");
  ros::Subscriber sub = nh.subscribe (voxel_rle_topic, 1, VoxelRleCallback);
  tf::TransformListener listener;
  tf_listener = &listener;

  // Create a ROS publisher for the output point cloud
  voxel_pub = nh.advertise<sensor_msgs::PointCloud2> ("voxel_map", 1);
  filtered_voxel_pub = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);

  // Spin
  ros::spin ();
}
