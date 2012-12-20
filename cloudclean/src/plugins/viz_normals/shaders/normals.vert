#version 330
 
layout(location = 0) in vec4 vertex;
layout(location = 1) in vec4 pointnormal;

out vec4 normal;

uniform vec3 lineColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main( void )
{
    vec4 vertex_out = vec4(vertex.x, vertex.y, vertex.z, 1.0f);
    normal = pointnormal;
    gl_Position = vertex_out;
}
