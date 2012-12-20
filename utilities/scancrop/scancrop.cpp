#include <limits>
#include <string>
#include <fstream>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <sstream>
#include <stdlib.h>
#include <assert.h>
#include <vector>
#include <cstdio>
#include <math.h>

#define IS_NAN(num) (num!=num)

class point{
public:
    float x, y, z, intensity;
};

class pointcloud{
public:
    int width;
    int height;
    float origin[3];
    float orientation_mat[9];
    float transform_mat[16];
    std::vector<point> points;
};


void read_ptx(const char* filename, pointcloud& cloud){
    int valid = 0;
    int nans = 0;
    int nulls = 0;

    // Makes things faster apparently
    //std::cin.sync_with_stdio(false);
	std::ifstream ptx_file(filename, std::ios_base::binary);
    assert(ptx_file.is_open());

	// Cloud dimentions
    ptx_file >> cloud.width;
    ptx_file >> cloud.height;
	cloud.points.resize (cloud.width * cloud.height);

	// Camera offset
	ptx_file >> cloud.origin[0];
	ptx_file >> cloud.origin[1];
	ptx_file >> cloud.origin[2];
	
    for(int i = 0; i < 9; i++)
        ptx_file >> cloud.orientation_mat[i];

	for(int i = 0; i < 16; i++)
        ptx_file >> cloud.transform_mat[i];

	ptx_file >> std::ws;

	// Read points
	std::string line;
	float x, y, z, intensity;

    // determine format
    getline( ptx_file, line);
    int tokens = 1;
    for(int i = 0; i< line.length(); i++)
        if(line[i] == ' ') tokens++;
    assert(tokens == 4);

    // read first line
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss << line;
    ss >> x >> y >> z >> intensity;

    if((x == 0) && (y == 0) & (z == 0) & ( fabs(intensity - 0.5f) < 0.0001 )) {
                    x = y = z = intensity = NAN;
    }

    cloud.points[0].x = x;
    cloud.points[0].y = y;
    cloud.points[0].z = z;
    cloud.points[0].intensity = intensity;

    for(int i = 1; i < cloud.width*cloud.height; i++){
        ptx_file >> x >> y >> z >> intensity;

        if((x == 0) && (y == 0) && (z == 0)) {
            x = y = z = intensity = NAN;
            nans++;
		}
        else{
            valid++;
        }

		cloud.points[i].x = x;
		cloud.points[i].y = y;
		cloud.points[i].z = z;
		cloud.points[i].intensity = intensity;
	}

    ptx_file.close();

}

bool save_ptx(const char* filename, pointcloud& cloud){
    std::ofstream ptx_file(filename);

    ptx_file << cloud.width << std::endl;
    ptx_file << cloud.height << std::endl;
    ptx_file << cloud.origin[0] << " " << cloud.origin[1] << " "<< cloud.origin[2] << std::endl;


    for(int i = 0; i < 9; i++){
        ptx_file << cloud.orientation_mat[i];
        if((i+1) % 3 != 0)
            ptx_file << " ";
        else
            ptx_file << std::endl;
        
    }

    for(int i = 0; i < 16; i++){
        ptx_file << cloud.transform_mat[i];
        if((i+1) % 4 != 0)
            ptx_file << " ";
        else
            ptx_file << std::endl;
    }

    // Print points
    for(unsigned int i = 0; i < cloud.points.size(); i++){
        if(IS_NAN(cloud.points[i].x)){
            ptx_file << "0 0 0 0.5" << std::endl;
        }
        else{
            ptx_file << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << " " << cloud.points[i].intensity << std::endl;
        }
    }

    ptx_file.close();
    return true;
}

void crop(pointcloud& cloud){

}


int main(int argc, char**argv){
    assert(argc == 3);
    pointcloud cloud;
    read_ptx(argv[1], cloud);

    std::vector<int> nans_in_row(cloud.height, 0);
    std::vector<int> nans_in_col(cloud.width, 0);

    int valid = 0;

    // Count nans
    for(int i = 0; i < cloud.points.size(); i++){
        if(IS_NAN(cloud.points[i].x) || IS_NAN(cloud.points[i].y) || IS_NAN(cloud.points[i].z) || IS_NAN(cloud.points[i].intensity)){
            int x = i%cloud.width;
            int y = i/cloud.width;
            nans_in_row[y]++;
            nans_in_col[x]++;
        }
        else{
            //printf("%f ", cloud.points[i].intensity);
            valid++;
        }
    }

    printf("Valid points: %d \n", valid);
/*
    printf("y\n");
    for(int i = 0; i < nans_in_row.size(); i++)
        printf("%d ",nans_in_row[i]);
    printf("\n");

    printf("x\n");
    for(int i = 0; i < nans_in_col.size(); i++)
        printf("%d ",nans_in_col[i]);
    printf("\n");
*/
    pointcloud croppedcloud;

    croppedcloud.width = 0;
    croppedcloud.height = 0;

    // Calc new dimentions
    for(int i = 0; i < nans_in_row.size(); i++)
        if(nans_in_row[i] != cloud.width)
            croppedcloud.height++;

    for(int i = 0; i < nans_in_col.size(); i++)
        if(nans_in_col[i] != cloud.height)
            croppedcloud.width++;

    printf("New dim %d, %d\n", croppedcloud.width, croppedcloud.height);

    croppedcloud.points.resize(croppedcloud.width*croppedcloud.height);

    for(int i = 0; i < 3; i++)
        croppedcloud.origin[i] = cloud.origin[i];
    for(int i = 0; i < 9; i++)
        croppedcloud.orientation_mat[i] = cloud.orientation_mat[i];
    for(int i = 0; i < 16; i++)
        croppedcloud.transform_mat[i] = cloud.transform_mat[i];

    int crop_i = 0;

    for(int i = 0; i < cloud.points.size(); i++){
        int y = i%cloud.width;
        int x = i/cloud.width;

        if(nans_in_row[x] == cloud.width || nans_in_col[y] == cloud.height)
            continue;

        if(crop_i > croppedcloud.points.size()-1){
            printf("Over size, %d > %d\n", crop_i, (int)croppedcloud.points.size());
            return 1;
        }

        croppedcloud.points[crop_i++] = cloud.points[i];
    }

    // write file back
    save_ptx(argv[2], croppedcloud);
}