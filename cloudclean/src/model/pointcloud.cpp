#include "model/pointcloud.h"

#include <limits>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cassert>

#include <pcl/io/io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef _WIN32
#   define INFINITY (DBL_MAX+DBL_MAX)
#   define NAN (INFINITY-INFINITY)
#endif

inline bool isNaN(float val){
    return (val != val);
}

PointCloud::PointCloud()
    : pcl::PointCloud<pcl::PointXYZI>() {
    cloud_dirty_ = true;
    labels_dirty_ = true;
    flags_dirty_ = true;
    pc_mutex.reset(new std::mutex());
}

bool PointCloud::save_ptx(const char* filename){
    pc_mutex->lock();
    std::ofstream ptx_file(filename);
    ptx_file << this->width << std::endl;
    ptx_file << this->height << std::endl;
    ptx_file << this->sensor_origin_[0] << " " << this->sensor_origin_[1]
             << " "<< this->sensor_origin_[2] << std::endl;

    // File is column major
    Eigen::Matrix3f rmat(this->sensor_orientation_);
    Eigen::Matrix4f tmat;
    tmat << rmat(0, 0) , rmat(0, 1) , rmat(0, 2) , this->sensor_origin_[0] ,
            rmat(1, 0) , rmat(1, 1) , rmat(1, 2) , this->sensor_origin_[1] ,
            rmat(2, 0) , rmat(2, 1) , rmat(2, 2) , this->sensor_origin_[2] ,
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

    // Write points
    for(unsigned int i = 0; i < this->points.size(); i++) {
        if(isNaN(this->points[i].x) || isNaN(this->points[i].y)
                || isNaN(this->points[i].z
                || isNaN(this->points[i].intensity))){
            ptx_file << "0 0 0 0.5" << std::endl;
        }
        else{
            ptx_file << this->points[i].x << " " << this->points[i].y
                     << " " << this->points[i].z << " "
                     << this->points[i].intensity << std::endl;
        }
    }

    ptx_file.close();
    pc_mutex->unlock();
    return true;
}

bool PointCloud::load_ptx(const char* filename, int subsample) {
    pc_mutex->lock();
	assert(subsample%2 == 0 || subsample == 1);

    // Makes things faster apparently
    std::cin.sync_with_stdio(false);

	std::ifstream ptx_file(filename, std::ios::binary);

    assert(ptx_file.is_open());

	// Contains nans
	this->is_dense = false;

	// Matrix dimentions
    int width, height;
    ptx_file >> width;
    ptx_file >> height;

	// Subsample
    this->width =  width/subsample;
    this->height = height/subsample;

    this->points.resize (this->width * this->height);
    labels_.resize(this->width * this->height, 0);
    flags_.resize(this->width * this->height);

    // original dimensions saved
    this->scan_width_ = width;
    this->scan_height_ = height;

	// Camera offset
	ptx_file >> this->sensor_origin_[0];
	ptx_file >> this->sensor_origin_[1];
	ptx_file >> this->sensor_origin_[2];
	this->sensor_origin_[3] = 0.0f;
	
	// Registration matrix
	Eigen::Matrix3f reg_mat;

    for(int row = 0; row < 3; row++ )
        for(int col = 0; col < 3; col++ )
            ptx_file >> reg_mat(row,col);

	// Registration quaternion
    this->sensor_orientation_ = Eigen::Quaternionf(reg_mat.transpose());

    // Discard registration mat4
	Eigen::Matrix4f reg_mat4;
    for(int col = 0; col < 4; col++ )
        for(int row = 0; row < 4; row++ )
			ptx_file >> reg_mat4(row,col);

	ptx_file >> std::ws;

    ///////////// Start processing points ////////////////
    // std::string line;
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

    if((x == 0) && (y == 0) && (z == 0)
            && ( fabs(intensity - 0.5f) < 0.0001 )) {
                    x = y = z = intensity = NAN;
    }

    this->points[0].x = x;
    this->points[0].y = y;
    this->points[0].z = z;
    this->points[0].intensity = intensity;
*/
    unsigned int i = 0;
    int sample = 0;
    int row = 0, col = 0;

    while(sample < width*height){
        row = sample / width;
        col = sample % width;

        // Only process every subsample-ith row and column
        if((row+1)%subsample != 0 || (col+1)%subsample != 0){
            //getline( ptx_file, line);
            ptx_file >> x >> y >> z >> intensity;
            sample++;
            continue;
        }

        sample++;

        ptx_file >> x >> y >> z >> intensity;

        if((x == 0) && (y == 0) && (z == 0)) {
                        x = y = z = intensity = NAN;
		}

		this->points[i].x = x;
		this->points[i].y = y;
		this->points[i].z = z;
		this->points[i].intensity = intensity;

        i++;
	}
    pc_mutex->unlock();
	return this;
}
