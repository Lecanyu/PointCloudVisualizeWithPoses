#include <vector>
#include <iostream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>

#include "Utils.h"
#include "VirtualCamera.h"

//static int pick_num = 0;
//static void point_picking_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
//{
//	std::cout << "Picking event active" << std::endl;
//	if (event.getPointIndex() != -1)
//	{
//		float x, y, z;
//		event.getPoint(x, y, z);
//		int index = event.getPointIndex();
//		std::cout << "you select point " << index << " at " << x << "," << y << "," << z << std::endl;
//		pcl::PointXYZ p;
//		p.x = x;	p.y = y;	p.z = z;
//		pcl::PointCloud<pcl::PointXYZ>::Ptr select_point(new pcl::PointCloud<pcl::PointXYZ>);
//		select_point->points.push_back(p);
//
//		boost::shared_ptr<pcl::visualization::PCLVisualizer>* viewer_ptr = static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer>*> (viewer_void);
//		boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer = *viewer_ptr;
//
//		std::stringstream ss;
//		ss << "selected_point" << pick_num;
//		std::string str_id = ss.str();
//		viewer->addPointCloud(select_point, str_id.c_str());
//		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, str_id.c_str());
//		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, str_id.c_str());
//
//		pick_num++;
//	}
//}

int main(int argc, char** argv)
{
	std::string pointcloud_dir, pointcloud_ds_dir, pose_file;

	if (argc!=4)
	{
	    std::cout << "viewer.exe pointcloud_ds_dir, pointcloud_dir pose_file\n";
	    return -1;
	}
	pointcloud_ds_dir = argv[1];
	pointcloud_dir = argv[2];
	pose_file = argv[3];

//	pointcloud_dir = "E:/Rec/ICL_NUIM_living_room/sandbox_25/pointcloud/";
//	pointcloud_ds_dir = "E:/Rec/ICL_NUIM_living_room/sandbox_25/pointcloud_ds/";
//	pose_file = "E:/Rec/ICL_NUIM_living_room/sandbox_25/pose_opt2.txt";

	int flag;
	std::cout << "0: sparse, 1: dense\n";
	std::cin >> flag;

	int n, m;
	std::cout << "Input N and M to show:\n";
	std::cin >> n >> m;

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scene_arr;
	for (int i = n; i < m; ++i)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::stringstream dsss1;
		if (flag == 0)
			dsss1 << pointcloud_ds_dir << "pointcloud_ds" << i << ".pcd";
		else if (flag==1)
			dsss1 << pointcloud_ds_dir << "pointcloud" << i << ".pcd";
		else
		{
			std::cout << "only accept 0 or 1 input\n";
			return 0;
		}
		pcl::io::loadPCDFile(dsss1.str(), *scene_ds);
		scene_arr.push_back(scene_ds);
		std::cout << "load point cloud" << i << " Done!\n";
	}

	RGBDTrajectory pose_arr;
	pose_arr.LoadFromFile(pose_file);

	// transform
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> scene_arr_transformed;
	for (int i = n; i < m; ++i)
	{
		if (i>=pose_arr.data_.size())
			continue;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		assert(pose_arr.data_[i].frame1_ == i);
		pcl::transformPointCloud(*scene_arr[i - n], *scene_out, pose_arr.data_[i].transformation_);
		scene_arr_transformed.push_back(scene_out);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 5, 0, 0, -1, 0, 1, 0);

//	std::set<int> special_camera = { 2, 84, 28, 33, 43, 91 };
    std::set<int> special_camera;
	VirtualCameras virtual_cam(viewer);
	for (int i = n; i < m; ++i)
	{
		if (i >= pose_arr.data_.size())
			continue;
		if (special_camera.find(i) == special_camera.end())
		{
			virtual_cam.addVitualCamera(0.15, i, pose_arr.data_[i].transformation_, 0, 1, 0, 1);
		}
		else
		{
			virtual_cam.addVitualCamera(0.2, i, pose_arr.data_[i].transformation_, 1, 0, 0, 3);
		}
	}

	for (int i = 0; i < scene_arr_transformed.size();++i)
	{
		std::stringstream ss;
		ss << "scene" << i;
		viewer->addPointCloud(scene_arr_transformed[i], ss.str());

		// white point cloud
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, ss.str());

		/*for (int t = 0; t < 10; ++t){
			std::stringstream se;
			se << "x: " << scene_arr_transformed[0]->at(t).x << ", y: " << scene_arr_transformed[0]->at(t).y << ", z: " << scene_arr_transformed[0]->at(t).z;
			std::cout << se.str() << "\n";
		}*/
	}


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}