#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
class icp_own
{
	struct kb_callback
	{
		bool flag;
		pcl::visualization::PCLVisualizer::Ptr view;
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_H;
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_H_closest;
	};

public :
	icp_own() :_maxiters(100), _min_trans_error(0.001), _rms(0.0001),
				_kd(new pcl::search::KdTree<pcl::PointXYZ>),
				viewer(new pcl::visualization::PCLVisualizer) {};

	~icp_own() {  };

public :
	void SetInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr& src);
	void SetInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& dst);
	void SetMaxIters(const int maxInters);
	void SetMinTransError(const double minTransError);
	void SetRMS(const double rms);
	void Register();
	void VisualICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src_h,const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_h_closest);

private:
	inline void InitVisual();
	void CalClosestPoints(Eigen::Matrix4f& H, pcl::PointCloud<pcl::PointXYZ>::Ptr &src, pcl::PointCloud<pcl::PointXYZ>::Ptr& mid, double& meanDistance)const;
	void SVD(pcl::PointCloud<pcl::PointXYZ>::Ptr& src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst, Eigen::Matrix4f &transform)const;
	void static KeyboardCallback(const pcl::visualization::KeyboardEvent&,void* args);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr _src;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _dst;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr _kd;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	int _maxiters;								//最大迭代次数
	double _min_trans_error;					//前后迭代的最小变化误差
	double _rms;								//最小均方误差
};

