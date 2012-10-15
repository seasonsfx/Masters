#version 330
 
layout(location = 0) in vec4 vertex;
out vec4 colour;
uniform vec3 elColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main( void )
{
    vec4 vertex_out = vec4(vertex.x, vertex.y, vertex.z, 1.0f);

    colour = vec4(elColour, 1.0f);
    //colour = vec4(1.0f, 0.0f, 0.0f, 1.0f);
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * vertex_out;
}
