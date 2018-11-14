#pragma once
#include <vector>
#include <iostream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

class VirtualCameras{
public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;

	VirtualCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer):_viewer(viewer){
		
	}

	void addVitualCamera(float size, int camera_id, const Eigen::Matrix4d& transformation, double r, double g, double b, double linewidth)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr camera_model(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointXYZ a1, a2, a3, a4, n0;
		a1.x = -size;	a1.y = size; a1.z = 0;	
		a2.x = size;	a2.y = size; a2.z = 0;
		a3.x = size;	a3.y = -size; a1.z = 0;
		a4.x = -size;	a4.y = -size; a1.z = 0;
		n0.x = 0;	n0.y = 0;	n0.z = -size*1.5;
		camera_model->push_back(a1);
		camera_model->push_back(a2);
		camera_model->push_back(a3);
		camera_model->push_back(a4);
		camera_model->push_back(n0);

		pcl::PointCloud<pcl::PointXYZ>::Ptr camera_model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*camera_model, *camera_model_transformed, transformation);

		std::stringstream ss;
		ss << "camera" << camera_id;
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(0), camera_model_transformed->at(1), r, g, b, ss.str().append("_01"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(1), camera_model_transformed->at(2), r, g, b, ss.str().append("_12"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(2), camera_model_transformed->at(3), r, g, b, ss.str().append("_23"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(3), camera_model_transformed->at(0), r, g, b, ss.str().append("_30"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(0), camera_model_transformed->at(4), r, g, b, ss.str().append("_04"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(1), camera_model_transformed->at(4), r, g, b, ss.str().append("_14"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(2), camera_model_transformed->at(4), r, g, b, ss.str().append("_24"), 0);
		_viewer->addLine<pcl::PointXYZ>(camera_model_transformed->at(3), camera_model_transformed->at(4), r, g, b, ss.str().append("_34"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_01"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_12"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_23"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_30"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_04"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_14"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_24"), 0);
		_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, linewidth, ss.str().append("_34"), 0);

	}

};