/*
 * cascased_ground_seg.cpp
 *
 * Created on	: May 11, 2018
 * Author	: Patiphon Narksri
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <opencv/cv.h>
#include <vector>
#include <pcl/filters/extract_indices.h>

#include <iostream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class CascasedGroundSeg
{
public:

	CascasedGroundSeg();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber points_node_sub_;
	ros::Publisher groundless_points_pub_;
	ros::Publisher ground_points_pub_;

	std::string point_topic_;
	std::string no_ground_topic;
	std::string ground_topic;

	// Inter-ring filter parameters
	int 		sensor_model_;
	double 		sensor_height_;
	double 		max_slope_;
	double vertical_thres_;

	// Multi-region plane fitting parameters
	double plane_dis_thres_;
	int n_section_;
	double plane_height_thres_;
	double plane_ang_thres_;

	bool		floor_removal_;

	int 		vertical_res_;
	int 		horizontal_res_;
	double	radius_table_[64];
	// cv::Mat 	index_map_;
	int **index_map_;
	std::vector<int> **region_index_;
	std::vector<double> section_bounds_;

	boost::chrono::high_resolution_clock::time_point t1_;
	boost::chrono::high_resolution_clock::time_point t2_;
	boost::chrono::nanoseconds elap_time_;

	const int 	DEFAULT_HOR_RES = 2000;

	void GetSectionBounds();
	double RadiusCal(double theta, double alpha, double beta);
	void InitRadiusTable(int in_model);
	void InitIndexMap();
	void FillIndexMap(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);
	double EstimatedRad(int index_tar, int index_ref);
	void ColumnSegment(int i, const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg, std::vector<int> &v_ring);
	void InitRegionIndex();
	int GetSection(double r);
	void FillRegionIndex(pcl::PointCloud<pcl::PointXYZI>::Ptr &remaining_ground_cloud);
	void SegmentGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
				pcl::PointCloud<pcl::PointXYZI> &out_groundless_points,
				pcl::PointCloud<pcl::PointXYZI> &out_ground_points);
	void VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg);

};

CascasedGroundSeg::CascasedGroundSeg() : node_handle_("~")
{
	ROS_INFO("Inititalizing Cascased Ground Segmentation...");

	node_handle_.param<std::string>("point_topic", point_topic_, "/velodyne_points");
	ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());
	node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
	ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
	node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
	ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());

	node_handle_.param("sensor_model", sensor_model_, 64);
	ROS_INFO("Sensor Model: %d", sensor_model_);
	node_handle_.param("sensor_height", sensor_height_, 1.80);
	ROS_INFO("Sensor Height: %f", sensor_height_);
	node_handle_.param("max_slope", max_slope_, 10.0);
	ROS_INFO("Max Slope: %f", max_slope_);
	node_handle_.param("vertical_thres", vertical_thres_, 0.08);
	ROS_INFO("Vertical Threshold: %f", vertical_thres_);

 	node_handle_.param("remove_floor",  floor_removal_,  true);
 	ROS_INFO("Floor Removal: %d", floor_removal_);

	node_handle_.param("plane_dis_thres", plane_dis_thres_, 1.80);
	ROS_INFO("Plane distance threshold: %f", plane_dis_thres_);
	node_handle_.param("n_section", n_section_, 4);
	ROS_INFO("Number of section: %d", n_section_);
	node_handle_.param("plane_height_thres", plane_height_thres_, 10.0);
	ROS_INFO("Height difference threshold: %f", plane_height_thres_);
	node_handle_.param("plane_ang_thres", plane_ang_thres_, 0.08);
	ROS_INFO("Angular differnce threshold: %f", plane_ang_thres_);

	vertical_res_ = sensor_model_;

	// Calculate expected radius for each laser
	InitRadiusTable(sensor_model_);
	// Calculate section bounds
	GetSectionBounds();

	switch(sensor_model_)
	{
		case 64:
			horizontal_res_ = 2083;
			break;
		case 32:
			horizontal_res_ = 2250;
			break;
		case 16:
			horizontal_res_ = 1800;
			break;
		default:
			horizontal_res_ = DEFAULT_HOR_RES;
			break;
	}

	// Create index map
  index_map_ = new int *[vertical_res_];
  for (int i = 0; i < vertical_res_; i++)
  {
    index_map_[i] = new int[horizontal_res_];
  }

	// Create region index
	region_index_ = new std::vector<int> *[4];
  for (int i = 0; i < 4; i++)
  {
    region_index_[i] = new std::vector<int> [n_section_];
  }

	points_node_sub_ = node_handle_.subscribe(point_topic_, 10000, &CascasedGroundSeg::VelodyneCallback, this);
	groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10000);
	ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 10000);

}

void CascasedGroundSeg::InitIndexMap()
{
  for (int i = 0; i < vertical_res_; i++)
  {
    for (int j = 0; j < horizontal_res_; j++)
    {
      index_map_[i][j] = -1;
    }
  }
}

void CascasedGroundSeg::FillIndexMap(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

	for (size_t i = 0; i < in_cloud_msg->points.size(); i++)
	{
		double u = atan2(in_cloud_msg->points[i].y,in_cloud_msg->points[i].x) * 180/M_PI;
		if (u < 0) { u = 360 + u; }
		int column = horizontal_res_ - (int)((double)horizontal_res_ * u / 360.0) - 1;
		int row = vertical_res_ - 1 - in_cloud_msg->points[i].ring;
		index_map_[row][column] = i;
	}

}

void CascasedGroundSeg::GetSectionBounds()
{
	// Calculate lasers id which define section boundaries
	int boundary_indices[n_section_];
	int section_width = int(ceil(1.0 * vertical_res_ / n_section_));
	for (int i = 0; i < n_section_; i++)
	{
		int new_ind = vertical_res_ - section_width * (i + 1);
		if (new_ind < 0)
		{
			boundary_indices[i] = 0;
		} else {
			boundary_indices[i] = new_ind;
		}
	}

	// Set up geometric parameters according to the sensor model
	double step = 0;
	double initial_angle = 0;
	switch (sensor_model_)
	{
		case 64:
			step = 1.0/3.0;
			initial_angle = -2.0;
			break;
		case 32:
			step = 4.0/3.0;
			initial_angle = -31.0/3.0;
			break;
		case 16:
			step = 2.0;
			initial_angle = -15.0;
			break;
		default:
			step = 1.0/3.0;
			initial_angle = -2.0;
			break;
	}

	section_bounds_.clear();
	int boundary_ind = n_section_ - 1;
	for (int i = 0; i < sensor_model_; i++)
	{
		if (i == boundary_indices[boundary_ind])
		{
			double theta = (i*step + initial_angle)/180*M_PI;
			if (theta != 0)
			{
				section_bounds_.insert(section_bounds_.begin(), sensor_height_ / tan(theta));
			} else {
				ROS_INFO("Please adjust number of sections: (n_section)");
				section_bounds_.insert(section_bounds_.begin(), sensor_height_ / tan(0.0001));
			}
			boundary_ind--;
			if (boundary_ind < 0) break;
		}
		if ((sensor_model_ == 64) && (i == 31))
		{
			step = 0.5;
			initial_angle = -15.0 + 8.83;
		}
	}
}

double CascasedGroundSeg::RadiusCal(double theta, double alpha, double beta)
{

	return fabs(1.0/(tan(theta) + tan(beta))) - (1.0/(tan(alpha + theta) + tan(beta)));

}

void CascasedGroundSeg::InitRadiusTable(int in_model)
{

	double step = 0;
	double initial_angle = 0;

	switch (in_model)
	{
		case 64:
			step = 1.0/3.0;
			initial_angle = -2.0;
			break;
		case 32:
			step = 4.0/3.0;
			initial_angle = -31.0/3.0;
			break;
		case 16:
			step = 2.0;
			initial_angle = -15.0;
			break;
		default:
			step = 1.0/3.0;
			initial_angle = -2.0;
			break;
	}

	double alpha = step/180*M_PI;
	double beta = max_slope_/180*M_PI;

	for (int i = 0; i < in_model; i++)
	{
		double theta = (i*step + initial_angle)/180*M_PI;
		if ((in_model == 64) && (i == 31))
		{
			radius_table_[i] = sensor_height_ * RadiusCal(theta, -1.0*alpha, beta);
			step = 0.5;
			initial_angle = -15.0 + 8.83;
			alpha = step/180*M_PI;
			beta = max_slope_/180*M_PI;
		} else {
			radius_table_[i] = sensor_height_ * RadiusCal(theta, alpha, beta);
		}
	}

}

double CascasedGroundSeg::EstimatedRad(int index_tar, int index_ref)
{
	double r = 0;
	for (int i = index_ref; i < index_tar; i++)
	{
			r += radius_table_[i];
	}
	return r;
}

void CascasedGroundSeg::ColumnSegment(int i, const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg, std::vector<int> &v_ring)
{
	int index_ref = -1;
	double r_ref;
	double z_max = 0;
	double z_min = 0;

	std::vector<int> candidates;
	candidates.clear();

	for (int j = 0; j < vertical_res_; j++)
	{
		if (index_map_[j][i] != -1)
		{
			if (index_ref == -1)
			{
				index_ref = j;
				velodyne_pointcloud::PointXYZIR point_ref = in_cloud_msg->points[index_map_[index_ref][i]];
				r_ref = sqrt(point_ref.x * point_ref.x + point_ref.y * point_ref.y);
				z_max = point_ref.z;
				z_min = point_ref.z;
				candidates.push_back(index_map_[index_ref][i]);
			} else {
				int index_tar = j;
				velodyne_pointcloud::PointXYZIR point_tar = in_cloud_msg->points[index_map_[index_tar][i]];
				double r_tar = sqrt(point_tar.x * point_tar.x + point_tar.y * point_tar.y);

				double r_diff = fabs(r_ref - r_tar);
				if (r_diff < EstimatedRad(index_tar, index_ref))
				{
					candidates.push_back(index_map_[index_tar][i]);
					if (point_tar.z > z_max) z_max = point_tar.z;
					if (point_tar.z < z_min) z_min = point_tar.z;
					r_ref = r_tar;
					index_ref	= index_tar;
				} else {
					if ((candidates.size() > 1) && ((z_max - z_min) > vertical_thres_))
					{
						// append candidates to v_ring
						v_ring.insert(v_ring.end(), candidates.begin(), candidates.end());
						candidates.clear();
					} else {
						candidates.clear();
					}
					candidates.push_back(index_map_[index_tar][i]);
		      z_max = point_tar.z;
		      z_min = point_tar.z;
					r_ref = r_tar;
					index_ref	= index_tar;
				}
			}
		}
	}

	if ((candidates.size() > 1) && ((z_max - z_min) > vertical_thres_))
	{
		// append candidates to v_ring
		v_ring.insert(v_ring.end(), candidates.begin(), candidates.end());
		candidates.clear();
	}

}

void CascasedGroundSeg::InitRegionIndex()
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < n_section_; j++)
    {
      region_index_[i][j].clear();
    }
  }
}

int CascasedGroundSeg::GetSection(double r)
{
	for (int i = 0; i < n_section_; i++)
	{
		if (r < section_bounds_[i])
		{
			return i;
		}
	}
	return n_section_ - 1;
}

void CascasedGroundSeg::FillRegionIndex(pcl::PointCloud<pcl::PointXYZI>::Ptr &remaining_ground_cloud)
{
	for(size_t i = 0; i < remaining_ground_cloud->size(); i++)
	{
		double y = remaining_ground_cloud->points[i].y;
		double x = remaining_ground_cloud->points[i].x;
		double r = sqrt(y*y + x*x);
		double u = atan2(y, x) * 180/M_PI;
		if (u < 0) { u = 360 + u; }

		if (u >= 315) {
		// Quadrant 0
			int s = GetSection(r);
      region_index_[0][s].push_back(i);
		} else if (u >= 225) {
		// Quadrant 3
			int s = GetSection(r);
      region_index_[3][s].push_back(i);
		} else if (u >= 135) {
		// Quadrant 2
			int s = GetSection(r);
      region_index_[2][s].push_back(i);
		} else if (u >= 45) {
		// Quadrant 1
			int s = GetSection(r);
      region_index_[1][s].push_back(i);
		} else {
		// Quadrant 0
			int s = GetSection(r);
      region_index_[0][s].push_back(i);
		}
	}
}

void CascasedGroundSeg::SegmentGround(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg,
			pcl::PointCloud<pcl::PointXYZI> &out_groundless_points,
			pcl::PointCloud<pcl::PointXYZI> &out_ground_points)
{

	///////////////////////////////////////////
	//////////// Inter-ring filter ////////////
	///////////////////////////////////////////
	// Declare vectors to store vertical points index
	std::vector<int> v_ring;
	// Clear index_map_
	InitIndexMap();
	// Fill index_map_
	FillIndexMap(in_cloud_msg);
	// Loop over columns
	for (int i = 0; i < horizontal_res_; i++)
	{
		// Segment each column of the index_map separately
    ColumnSegment(i, in_cloud_msg, v_ring);
  }
	// Convert velodyne_pointcloud to PCL PointCloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*in_cloud_msg, *pcl_in_cloud);
	// Extract vertical points and remaining ground points from the input cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_ground (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointIndices::Ptr ground_indices (new pcl::PointIndices);
	ground_indices->indices = v_ring;
	pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (pcl_in_cloud);
  extract.setIndices (ground_indices);
  extract.setNegative (true);
  extract.filter (*remaining_ground);
  extract.setNegative (false);
  extract.filter (out_groundless_points);

	///////////////////////////////////////////////
	/////////// Quandrants plane fitting //////////
	///////////////////////////////////////////////
	// Clear region_index
	InitRegionIndex();
	// Fill region_index
	FillRegionIndex(remaining_ground);
	// for (int i = 0; i < 4; i++)
	// {
	// 	for (int j = 0; j < n_section_; j++)
	// 	{
	// 		std::cout << "Quandrant, Section" << i << ", " << j << '\n';
	// 		std::cout << "Total points: " << region_index_[i][j].size() << "Points" << '\n';
	// 	}
	// }
	// std::cout << "Totla point for remaining" << remaining_ground->points.size() << '\n';

	pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_vertical (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.3);
	// Perform segmentation
	seg.setInputCloud (remaining_ground);
  seg.segment (*inliers, *coefficients);
	// Extract the segmented points
  extract.setInputCloud (remaining_ground);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter(out_ground_points);
  extract.setNegative (true);
  extract.filter(*remaining_vertical);

	//////////////////////////////////////////
	////////////// Outputs merge /////////////
	//////////////////////////////////////////
	out_groundless_points += *remaining_vertical;
}

void CascasedGroundSeg::VelodyneCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_cloud_msg)
{

	t1_ = boost::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZI> vertical_points;
	pcl::PointCloud<pcl::PointXYZI> ground_points;
	vertical_points.header = in_cloud_msg->header;
	ground_points.header = in_cloud_msg->header;
	vertical_points.clear();
	ground_points.clear();

	SegmentGround(in_cloud_msg, vertical_points, ground_points);

	if (!floor_removal_)
	{
		pcl::copyPointCloud(*in_cloud_msg, vertical_points);
	}

	groundless_points_pub_.publish(vertical_points);
	ground_points_pub_.publish(ground_points);
	t2_ = boost::chrono::high_resolution_clock::now();
	elap_time_ = boost::chrono::duration_cast<boost::chrono::nanoseconds>(t2_-t1_);
	std::cout << "Computational Time for one frame: " << elap_time_ << '\n';

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cascased_ground_seg");
	CascasedGroundSeg node;
	ros::spin();

	return 0;

}
