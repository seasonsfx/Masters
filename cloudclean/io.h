#ifndef CC_IO
#define CC_IO

#include <limits>
#include <string>
#include <fstream>
#include <exception>
#include <stdexcept>
#include <sstream>
#include <stdlib.h>
#include <assert.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include </usr/include/eigen3/Eigen/Core>
#include </usr/include/eigen3/Eigen/Geometry>

typedef pcl::PointXYZI PointType;

pcl::PointCloud<pcl::PointXYZI>::Ptr read_ptx(const char* filename, int subsample){
	assert(subsample%2 == 0 || subsample == 1);

    // Makes things faster
    //std::cin.sync_with_stdio(false);

	std::ifstream ptx_file(filename);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
	// Contains nans
	cloud->is_dense = false;

	// Matrix dimentions
	ptx_file >> cloud->width;
	ptx_file >> cloud->height;

	// Subsample
	cloud->width/=subsample;
	cloud->height/=subsample;

	cloud->points.resize (cloud->width * cloud->height);

	// Camera offset
	ptx_file >> cloud->sensor_origin_[0];
	ptx_file >> cloud->sensor_origin_[1];
	ptx_file >> cloud->sensor_origin_[2];
	cloud->sensor_origin_[3] = 0.0f;
	
	// Registration matrix
	Eigen::Matrix3f reg_mat;

	for(int row = 0; row < 3; row++ )
		for(int col = 0; col < 3; col++ )
			ptx_file >> reg_mat(row,col);

	// Registration quaternion
	cloud->sensor_orientation_ = Eigen::Quaternionf(reg_mat);

	// Registration mat4
	Eigen::Matrix4f reg_mat4;

	for(int row = 0; row < 4; row++ )
		for(int col = 0; col < 4; col++ )
			ptx_file >> reg_mat4(row,col);

	ptx_file >> std::ws;

	// Read points
	std::string line;
	float x, y, z, intensity;
        unsigned int i = 0;
        int sample = 0;

	while(getline( ptx_file, line) && i < cloud->width*cloud->height){
		if( (sample++%subsample) !=0){
			continue;
		}

		std::stringstream ss(std::stringstream::in | std::stringstream::out);
		ss << line;
		ss >> x >> y >> z >> intensity;

		if((x == 0) && (y == 0) & (z == 0) & ( intensity == 0.5f)) {
                        x = y = z = intensity = NAN;
		}

		cloud->points[i].x = x;
		cloud->points[i].y = y;
		cloud->points[i].z = z;
		cloud->points[i].intensity = intensity;

		i++;
	}
	return cloud;
}

#endif// CC_IO