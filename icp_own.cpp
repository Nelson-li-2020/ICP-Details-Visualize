#include "icp_own.h"
#include <pcl/common/common.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

void icp_own::SetInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src)
{
	if (!src->empty())
	{
		_src = src;
	}
}

void icp_own::SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& dst)
{
	if (!dst->empty())
	{
		_dst = dst;
	}
}

void icp_own::SetMaxIters(const int maxInters)
{
	this->_maxiters = maxInters;
}

void icp_own::SetMinTransError(const double minTransError)
{
	this->_min_trans_error = minTransError;
}

void icp_own::SetRMS(const double rms)
{
	this->_rms;
}

void icp_own::Register()
{
	InitVisual();
	Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f FinalMatrix = Eigen::Matrix4f::Identity();
	this->_kd->setInputCloud(this->_dst);

	int iters = 0;
	double rms = INTMAX_MAX;
	double lastRms = 0;
	//
	pcl::PointCloud<pcl::PointXYZ>::Ptr src1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mid(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*this->_src, *src1);

	kb_callback kb_call;
	kb_call.flag = true;
	kb_call.view = this->viewer;
	kb_call.src_H = src1;
	kb_call.src_H_closest = mid;
	viewer->registerKeyboardCallback(&icp_own::KeyboardCallback,(void*)&kb_call);//注册点云可视化过程

	while (iters<this->_maxiters || rms > this->_rms )
	{
		iters++;
 		this->CalClosestPoints(H, src1, mid, rms);
		std::cout << "H:\n" << H << std::endl;
		while (1)
		{
			viewer->spinOnce(100);
			if (!kb_call.flag)
			{
				kb_call.flag = true;
				break;
			}
		}
		if (abs(rms-lastRms)<this->_min_trans_error)
			break;
		this->SVD(src1, mid, H);
		FinalMatrix = H* FinalMatrix;
		lastRms = rms;
		std::cout << iters << std::endl;
		std::cout << "RMS:"<< rms << std::endl;


	}
	cout << "FinalTransFormMatrix:\n" << FinalMatrix << endl;
	cout << "FinalRMS:" << rms << endl;

	//显示最终配准结果
	this->viewer->setBackgroundColor(1.0, 1.0, 1.0);
	this->viewer->removeAllPointClouds();
	this->viewer->addPointCloud(this->_src,"src");
	this->viewer->addPointCloud(this->_dst,"dst");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_align(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*this->_src, *cloud_align, FinalMatrix);
	this->viewer->addPointCloud(cloud_align, "align");
	this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "src");
	this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "dst");
	this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "align");
	this->viewer->spin();
}

void icp_own::InitVisual()
{
	//可视化配准过程
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_H(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_H_closest(new pcl::PointCloud<pcl::PointXYZ>);

	viewer->setBackgroundColor(0.0, 0.0, 0.0);
	viewer->setWindowName("Visual");
	viewer->addPointCloud(this->_src, "src");
	viewer->addPointCloud(this->_dst, "dst");
	viewer->addPointCloud(src_H, "src_H");
	viewer->addPointCloud(src_H_closest, "src_H_closest");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "src");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.0, 0.0, "dst");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.0, "src_H");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.6, "src_H_closest");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "src_H_closest");
}

void icp_own::CalClosestPoints(Eigen::Matrix4f& H, pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& mid, double &meanDistance) const
{
	meanDistance = 0.0;
	pcl::transformPointCloud(*src, *src, H);
	//
	int nums = src->points.size();
	std::vector<int> indices(1);
	std::vector<float> dist(1);
	std::vector<int> closestIndices(nums);

	for (size_t i = 0; i < nums; i++)
	{
		if (this->_kd->nearestKSearch(src->points[i], 1, indices, dist))
		{
			closestIndices[i]=(indices[0]);
			meanDistance += sqrt(dist[0]);
		};
	}
	meanDistance /= nums;
	if (this->_dst->points.size() == closestIndices.size())
	{
		this->_dst->push_back(pcl::PointXYZ());
	}

	pcl::copyPointCloud(*this->_dst, closestIndices, *mid);
}

void icp_own::SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst, Eigen::Matrix4f& transform)const
{
	//1.去中心化
	Eigen::Vector4f center_src, center_dst;
	Eigen::MatrixXf src_demean,dst_demean;
	pcl::compute3DCentroid(*src, center_src);
	pcl::compute3DCentroid(*dst, center_dst);
	pcl::demeanPointCloud(*src, center_src, src_demean);
	pcl::demeanPointCloud(*dst, center_dst, dst_demean);
	
	//2.计算协方差矩阵
	Eigen::Matrix3f cov = (src_demean * dst_demean.transpose()).topLeftCorner(3,3);

	//3.svd分解
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();
	Eigen::Matrix3f R = V * U.transpose();
	Eigen::Vector3f T = center_dst.head(3) - (R * center_src.head(3));

	transform.block<3, 3>(0, 0) = R;
	transform.block<3, 1>(0, 3) = T;
}

void icp_own::KeyboardCallback(const pcl::visualization::KeyboardEvent& event, void* args)
{
	kb_callback* kb = (kb_callback*)args;
	if (event.keyDown() && event.getKeyCode() != -1)
	{
		if (!kb->view->wasStopped())
		{
			kb->flag = false;
			kb->view->updatePointCloud(kb->src_H, "src_H");
			kb->view->updatePointCloud(kb->src_H_closest, "src_H_closest");
			kb->view->spin();
		}
	
	}
}
