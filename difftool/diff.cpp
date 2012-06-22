#include <fstream>
#include <cstdlib>
#include <cassert>
#include <sstream>

int getSizeSkipHeader(std::ifstream & ptx_file){

	int width, height, size;
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

	if((x == 0) && (y == 0) & (z == 0) & ( intensity == 0.5f)) {
                    x = y = z = intensity = -1.0f;
	}
}

int main(int argc, char** argv){

	assert(argc == 3);

	std::ifstream file_a(argv[1]);
	std::ifstream file_b(argv[2]);
	int size_a = getSizeSkipHeader(file_a);
	int size_b = getSizeSkipHeader(file_b);

	assert(size_a == size_b);

	int diff = 0;

	// Read points
	std::string line_a, line_b;
	float x_a, y_a, z_a, intensity_a;
	float x_b, y_b, z_b, intensity_b;

	unsigned int i = 0;

	while(getline( file_a, line_a) && getline( file_b, line_b) && i < size_a){
		setpoint(line_a, x_a, y_a, z_a, intensity_a);
		setpoint(line_b, x_b, y_b, z_b, intensity_b);

		float delta = 0.00f;

		if(abs(x_a - x_b) > delta && abs(y_a - y_b) > delta && abs(z_a - z_b) > delta && abs(intensity_a - intensity_b) > delta)
			diff++;

		i++;
	}

	std::printf("%d/%d points changed (%.2f%%). \n", diff, size_a, diff/(float)size_a);

	return 0;
}