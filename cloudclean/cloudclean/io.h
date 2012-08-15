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

#define IS_NAN(num) (num!=num)

bool save_ptx(const char* filename, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    std::ofstream ptx_file(filename);
    ptx_file << cloud->width << std::endl;
    ptx_file << cloud->height << std::endl;
    ptx_file << cloud->sensor_origin_[0] << " " << cloud->sensor_origin_[1] << " "<< cloud->sensor_origin_[2] << std::endl;

    // File is column major
    Eigen::Matrix3f rmat(cloud->sensor_orientation_);
    Eigen::Matrix4f tmat;
    tmat << rmat(0, 0) , rmat(0, 1) , rmat(0, 2) , cloud->sensor_origin_[0] ,
            rmat(1, 0) , rmat(1, 1) , rmat(1, 2) , cloud->sensor_origin_[1] ,
            rmat(2, 0) , rmat(2, 1) , rmat(2, 2) , cloud->sensor_origin_[2] ,
            0 , 0 , 0 , 1;


    for(int c = 0; c < 3; c++){
        for(int r = 0; r < 3; r++){
            ptx_file << rmat(r, c);
            if(r < 2)
                ptx_file << " ";
            else
                ptx_file << std::endl;
        }
    }

    for(int c = 0; c < 4; c++){
        for(int r = 0; r < 4; r++){
            ptx_file << tmat(r, c);
            if(r < 3)
                ptx_file << " ";
            else
                ptx_file << std::endl;
        }
    }

    // Print points
    for(unsigned int i = 0; i < cloud->points.size(); i++){
        if(IS_NAN(cloud->points[i].x) || IS_NAN(cloud->points[i].y) || IS_NAN(cloud->points[i].z || IS_NAN(cloud->points[i].intensity))){
            ptx_file << "0 0 0 0.5" << std::endl;
        }
        else{
            ptx_file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << cloud->points[i].intensity << std::endl;
        }
    }

    ptx_file.close();
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr read_ptx(const char* filename, int subsample){
	assert(subsample%2 == 0 || subsample == 1);

    // Makes things faster apparently
    std::cin.sync_with_stdio(false);

	std::ifstream ptx_file(filename);

    assert(ptx_file.is_open());

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
	// Contains nans
	cloud->is_dense = false;

	// Matrix dimentions
    int width, height;
    ptx_file >> width;
    ptx_file >> height;

	// Subsample
    cloud->width =  width/subsample;
    cloud->height = height/subsample;

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
    cloud->sensor_orientation_ = Eigen::Quaternionf(reg_mat.transpose());

    // Discard registration mat4
	Eigen::Matrix4f reg_mat4;
    for(int col = 0; col < 4; col++ )
        for(int row = 0; row < 4; row++ )
			ptx_file >> reg_mat4(row,col);

	ptx_file >> std::ws;

    ///////////// Start processing points ////////////////
	std::string line;
	float x, y, z, intensity;

    // Determine format
    /*
    getline( ptx_file, line);
    int tokens = 1;
    for(int i = 0; i< line.length(); i++)
        if(line[i] == ' ') tokens++;
    assert(tokens == 4);

    // Read first line to out sniff format
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss << line;
    ss >> x >> y >> z >> intensity;

    if((x == 0) && (y == 0) && (z == 0) && ( fabs(intensity - 0.5f) < 0.0001 )) {
                    x = y = z = intensity = NAN;
    }

    cloud->points[0].x = x;
    cloud->points[0].y = y;
    cloud->points[0].z = z;
    cloud->points[0].intensity = intensity;
*/
    unsigned int i = 0;
    int sample = 0;
    int row = 0, col = 0;

    while(sample < width*height){
        row = sample / width;
        col = sample % width;

        //qDebug("%d::%d", row, col);

        // Only process every subsample-ith row and column
        if((row+1)%subsample != 0 || (col+1)%subsample != 0){
            //getline( ptx_file, line);
            ptx_file >> x >> y >> z >> intensity;
            sample++;
            continue;
        }
        //qDebug("Sample number %d", sample);
        sample++;

        ptx_file >> x >> y >> z >> intensity;

        if((x == 0) && (y == 0) && (z == 0)) {
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
