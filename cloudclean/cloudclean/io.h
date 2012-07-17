#ifndef CC_IO
#define CC_IO

#include <limits>
#include <string>
#include <fstream>
#include <exception>
#include <stdexcept>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <assert.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include </usr/include/eigen3/Eigen/Core>
#include </usr/include/eigen3/Eigen/Geometry>

typedef pcl::PointXYZI PointType;

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
        if(cloud->points[i].x != cloud->points[i].x){
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
    //std::cin.sync_with_stdio(false);

    //std::ifstream ptx_file(filename);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	
    FILE * fp;
    fp = fopen (filename,"r");
    if (fp==NULL)
    {
        assert(false);
    }

	// Contains nans
	cloud->is_dense = false;

    // Scan dimentions
    int width, height;
    assert(std::fscanf(fp,"%i\n",&width) != 0);
    assert(std::fscanf(fp,"%i\n",&height) != 0);

	// Subsample
    cloud->width=width/subsample;
    cloud->height=height/subsample;

	cloud->points.resize (cloud->width * cloud->height);

    float x, y, z, intensity;

	// Camera offset

    assert(std::fscanf(fp,"%f %f %f\n", &x, &y, &z) != 0);
    cloud->sensor_origin_[0] = x;
    cloud->sensor_origin_[1] = y;
    cloud->sensor_origin_[2] = z;
	cloud->sensor_origin_[3] = 0.0f;
	
	// Registration matrix
	Eigen::Matrix3f reg_mat;

    for(int row = 0; row < 3; row++ ){
        int result = std::fscanf(fp,"%f %f %f\n", &x, &y, &z);
        printf("Result %i\n", result);
        fflush(stdout);
        assert(result != 0);
        reg_mat(row,0) = x;
        reg_mat(row,1) = y;
        reg_mat(row,2) = z;
    }

	// Registration quaternion
    cloud->sensor_orientation_ = Eigen::Quaternionf(reg_mat.transpose());

    // Discard registration mat4
    float t1, t2, t3, t4;
    assert(std::fscanf(fp,"%f %f %f %f\n", &t1, &t2, &t3, &t4) != 0);
    assert(std::fscanf(fp,"%f %f %f %f\n", &t1, &t2, &t3, &t4) != 0);
    assert(std::fscanf(fp,"%f %f %f %f\n", &t1, &t2, &t3, &t4) != 0);
    assert(std::fscanf(fp,"%f %f %f %f\n", &t1, &t2, &t3, &t4) != 0);

    // Detect file format
    char linebuf[256];
    int ii=0;
    fread(&(linebuf[ii++]),1,1,fp);
    while(linebuf[ii-1] != '\n')
        if ( fread(&(linebuf[ii++]),1,1,fp)==0 ) assert(false);;
    linebuf[ii-1] = '\0'; // terminate the string
    int numtokens = 1;
    bool hascolor = false;
    for(ii=0; ii<(int)strlen(linebuf); ii++)
        if(linebuf[ii] == ' ') numtokens++;
    if(numtokens == 4)
        hascolor = false;
    else if(numtokens == 7){
        hascolor = true;
        assert(false);
    }
    else
        assert(false);

    // Read first line
    ///float x, y, z, intensity;
    sscanf(linebuf,"%f %f %f %f", &x, &y, &z, &intensity);

    cloud->points[0].x = x;
    cloud->points[0].y = y;
    cloud->points[0].z = z;
    cloud->points[0].intensity = intensity;

    int s = 1;

    for(int i = 1; i < width*height; i++){
        if( (i%subsample) !=0){
			continue;
        }

        std::fscanf(fp,"%f %f %f %f", &x, &y, &z, &intensity);

		if((x == 0) && (y == 0) & (z == 0) & ( intensity == 0.5f)) {
                        x = y = z = intensity = NAN;
		}

        cloud->points[s].x = x;
        cloud->points[s].y = y;
        cloud->points[s].z = z;
        cloud->points[s].intensity = intensity;
        s++;
	}

    fclose (fp);
	return cloud;
}

#endif// CC_IO
