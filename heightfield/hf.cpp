#include <fstream>
#include <cstdlib>
#include <cassert>
#include <sstream>
#include <cmath>

int getSizeSkipHeader(std::ifstream & ptx_file, int & width, int & height){

	int size;
	float temp;

	// Matrix dimentions
	ptx_file >> width;
	ptx_file >> height;
	size = width * height;

	// Skip Camera offset
	ptx_file >> temp >> temp >> temp;

	// Skip registration matrix
    for(int col = 0; col < 3; col++ )
        for(int row = 0; row < 3; row++ )
            ptx_file >> temp;


    // Skip registration mat4
    for(int col = 0; col < 4; col++ )
        for(int row = 0; row < 4; row++ )
			ptx_file >> temp;

	ptx_file >> std::ws;

	return size;

}

void setpoint(std::string & line, float & x, float & y, float &z, float &intensity){
	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << line;
	ss >> x >> y >> z >> intensity;

	if((x == 0) && (y == 0) & (z == 0)) {
    	x = y = z = intensity = 0.0f;
	}
}


struct DI{
	float distance, intensity;
};

void writeImage(const char * filename, int width, int height, DI* di){

	// Calculate max depth
	float max_depth = 0.0f;
	for(int i = 0; i< width*height; i++){
		if(di[i].distance > max_depth)
			max_depth = di[i].distance;
	}

	std::ofstream outfile(filename, std::ios::binary | std::ios::trunc);

	int tmp = height;
	height = width;
	width = tmp;

    outfile << "P5\n";
	outfile << "#Comment!\n" << width << " " <<
			height << "\n" << "255" << std::endl;

	for(int i = 0; i< width*height; i++){
		char val = (di[i].distance/max_depth)*255;
		//char val = di[i].intensity*255;
	    //char val = (di[i].distance/max_depth)*di[i].intensity*255;
		outfile.write(&val, 1);
	}

	assert(!outfile.fail());
	outfile.close();
}



int main(int argc, char** argv){

	assert(argc == 3);

	std::ifstream file(argv[1]);
	int width, height;
	int size = getSizeSkipHeader(file, width, height);


	DI * di = new DI[size];


	// Read points
	std::string line, line_b;
	float x, y, z, intensity;

	for(int i = 0; i < size; i++){
		getline(file, line);
		setpoint(line, x, y, z, intensity);

		di[i].distance = sqrt(x*x + y*y + z*z);
		di[i].intensity = intensity;

	}

	writeImage(argv[2], width, height, di);

	delete [] di;

	return 0;
}