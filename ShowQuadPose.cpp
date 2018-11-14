#include <vector>
#include <iostream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>

#include "Utils.h"


int main(int argc, char** argv)
{
	std::string pointcloud_ds_dir, pose_file, ply_file;

	/*if (argc!=4)
	{
	std::cout << "viewer.exe pointcloud_ds_dir, pointcloud_dir pose_file\n";
	return -1;
	}
	pointcloud_ds_dir = argv[1];
	pointcloud_dir = argv[2];
	pose_file = argv[3];*/

	// Kintinuous or elastic fusion
	pointcloud_ds_dir = "E:/Rec/CCT/Lab_2/sandbox/pointcloud_ds/";
	pose_file = "E:/Rec/CCT/Kintinuous/lab2/lab2kintinous.poses";
	ply_file = "E:/Rec/CCT/Kintinuous/lab2/Kintinuous.ply";
	QuadPose quad_pose;
	quad_pose.LoadFromFile(pose_file, false);

	// ORB
	/*pointcloud_ds_dir = "E:/Rec/CCT/Lab_2/sandbox/pointcloud_ds/";
	pose_file = "E:/Rec/CCT/ORB-SLAM2/lab1/lab1ORB.txt";
	QuadPose quad_pose;
	quad_pose.LoadFromFile(pose_file, true);*/

	Convert convert;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
	std::vector<int> ids = quad_pose.id;
	for (int i = 0; i < quad_pose.quad.size();++i)
	{
		Eigen::Matrix4d pose = convert.QuadToMatrix4d(quad_pose.quad[i]);
		poses.push_back(pose);
	}

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scene_arr;
	for (int i = 0; i < ids.size();++i)
	{
		int id = ids[i];
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::stringstream dsss1;
		dsss1 << pointcloud_ds_dir << "pointcloud_ds" << id << ".pcd";
		pcl::io::loadPCDFile(dsss1.str(), *scene_ds);
		scene_arr.push_back(scene_ds);
		std::cout << "load point cloud" << id << " Done!\n";
	}

	// transform
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr integrate_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scene_arr_transformed;
	for (int i = 0; i < ids.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*scene_arr[i], *scene_out, poses[i]);
		scene_arr_transformed.push_back(scene_out);

		for (int t = 0; t < scene_out->size();++t)
			integrate_pointcloud->push_back(scene_out->at(t));
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);
	viewer->addPointCloud(integrate_pointcloud, "i");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	pcl::io::savePLYFileASCII(ply_file, *integrate_pointcloud);

	return 0;
}