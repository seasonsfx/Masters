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

void EventDispatcher::updateProgress(int value){
    emit progress(value);
}

void EventDispatcher::emitlabelUpdate() {
    emit labelUpdate();
}

void EventDispatcher::emitflagUpdate() {
    emit flagUpdate();
}

PointCloud::PointCloud()
    : pcl::PointCloud<pcl::PointXYZI>() {
    pc_mutex.reset(new std::mutex());
    ed_.reset(new EventDispatcher());
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

bool PointCloud::load_ptx(const char* filename, int decimation_factor) {
    pc_mutex->lock();
    ed_->updateProgress(0);
    assert(decimation_factor%2 == 0 || decimation_factor == 1);

    // Makes things faster apparently
    std::cin.sync_with_stdio(false);

	std::ifstream ptx_file(filename, std::ios::binary);

    assert(ptx_file.is_open());

	// Contains nans
	this->is_dense = false;

	// Matrix dimentions
    int file_width, file_height;
    ptx_file >> file_width;
    ptx_file >> file_height;

	// Subsample
    this->scan_width_ =  file_width/decimation_factor;
    this->scan_height_ = file_height/decimation_factor;

	// Camera offset
	ptx_file >> this->sensor_origin_[0];
	ptx_file >> this->sensor_origin_[1];
	ptx_file >> this->sensor_origin_[2];
	this->sensor_origin_[3] = 0.0f;
	
    Eigen::Matrix3f orientation_mat;
    for(int row = 0; row < 3; row++ )
        for(int col = 0; col < 3; col++ )
            ptx_file >> orientation_mat(row,col);
    this->sensor_orientation_ = Eigen::Quaternionf(orientation_mat.transpose());

    // Discard registration mat4
	Eigen::Matrix4f reg_mat4;
    for(int col = 0; col < 4; col++ )
        for(int row = 0; row < 4; row++ )
			ptx_file >> reg_mat4(row,col);

	ptx_file >> std::ws;

	float x, y, z, intensity;

    unsigned int sampled_idx = 0;
    int file_sample_idx = 0;
    int row = 0, col = 0;

    int line_count = file_width*file_height;
    int update_interval = line_count/100;

    while(file_sample_idx < line_count){
        if(file_sample_idx % update_interval == 0)
            ed_->updateProgress(100*file_sample_idx/static_cast<float>(line_count));

        row = file_sample_idx / file_width;
        col = file_sample_idx % file_width;
        file_sample_idx++;

        // Only process every decimation_factor-ith row and column
        if((row+1)%decimation_factor != 0 || (col+1)%decimation_factor != 0){
            ptx_file >> x >> y >> z >> intensity;
            continue;
        }

        ptx_file >> x >> y >> z >> intensity;
        sampled_idx++;

        // Skip points that are invalid
        if((x == 0) && (y == 0) && (z == 0)) {
            continue;
		}

        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = intensity;;
        this->points.push_back(point);
        this->cloud_to_grid_map_.push_back(sampled_idx);
	}

    this->width = this->points.size();
    this->height = 1;
    this->is_dense = true; // Cloud is dense and unorganised
    labels_.resize(this->width * this->height, 0);
    flags_.resize(this->width * this->height);

    ed_->updateProgress(100);
    pc_mutex->unlock();
	return this;
}
