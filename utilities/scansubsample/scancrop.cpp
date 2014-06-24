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


void resize_ptx(const char* filename, const char* out_filename, int factor){
    int width = -1;
    int height = -1;
    float origin[3];
    float tmp;
    std::string tmp_str;

    // Makes things faster apparently
    //std::cin.sync_with_stdio(false);
	std::ifstream ptx_file(filename, std::ios_base::binary);
    assert(ptx_file.is_open());
    std::ofstream out_ptx_file(out_filename);
    assert(out_ptx_file.is_open());

	// Dimentions
    ptx_file >> width;
    ptx_file >> height;
    out_ptx_file << width/factor << std::endl;
    out_ptx_file << height/factor << std::endl;

	// Camera offset
	ptx_file >> origin[0];
	ptx_file >> origin[1];
	ptx_file >> origin[2];
	out_ptx_file << origin[0] << " " << origin[1] << " "<< origin[2] << std::endl;
	
	for(int i = 0; i < 9; i++){
		ptx_file >> tmp;
	    out_ptx_file << tmp;
	    if((i+1) % 3 != 0)
	        out_ptx_file << " ";
	    else
	        out_ptx_file << std::endl;
	    
	}

	for(int i = 0; i < 16; i++){
		ptx_file >> tmp;
	    out_ptx_file << tmp;
	    if((i+1) % 4 != 0)
	        out_ptx_file << " ";
	    else
	        out_ptx_file << std::endl;
	}

	ptx_file >> std::ws;

	// Read points
	std::string line;
	float x, y, z, intensity;

    // Determine format
    getline( ptx_file, line);
    int tokens = 1;
    for(int i = 0; i< line.length(); i++)
        if(line[i] == ' ') tokens++;

    if(tokens == 7){
    	printf("The scan has color but we are discarding the information.\n");
    }

    // Read first line
    std::stringstream ss(std::stringstream::in | std::stringstream::out);
    ss << line;
    ss >> x >> y >> z >> intensity;

    // Write first line
    out_ptx_file << x << " " << y << " " << z << " " << intensity << std::endl;

    const int status_interval = width*height/100;

    for(int i = 1; i < width*height; i++){
        ptx_file >> x >> y >> z >> intensity;

        // strip out colors etc
        if(tokens > 4){
        	for(int j = 4; j < tokens; j++){
        		ptx_file >> tmp_str;
        	}
        }

        if(i % status_interval == 0){
			printf("%0.f percent done\n", (100.0*i)/(width*height));
		}

        int iy = i%height;
        int ix = i/height;

        if((iy+1)%factor != 0)
        	continue;
        if((ix+1)%factor != 0)
        	continue;

		// WRITE OUT
		out_ptx_file << x << " " << y << " " << z << " " << intensity << std::endl;

	}

    ptx_file.close();
    out_ptx_file.close();

}


int main(int argc, char**argv){
    assert(argc == 4 && "Too few pareters.");
  
    const char* in_file_name = argv[1];
    const char* out_file_name = argv[2];
    int factor = atoi(argv[3]);

    resize_ptx(in_file_name, out_file_name, factor);
}