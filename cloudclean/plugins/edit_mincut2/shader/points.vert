#version 330
 
layout(location = 0) in vec4 vertex;

uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main( void )
{
    vec4 vertex_out = vec4(vertex.xyz, 1.0f);
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * vertex_out;
}
