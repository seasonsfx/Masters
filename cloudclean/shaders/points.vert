#version 330
 
layout(location = 0) in vec4 vertex; 
out vec4 colour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main( void )
{
    vec4 vertex_out = vec4(vertex.x, vertex.y, vertex.z, 1.0f);

    colour = vec4(vertex.w, vertex.w, vertex.w, 1.0f);
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * vertex_out;
}
