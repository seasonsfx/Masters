#version 330
 
layout(location = 0) in vec2 vertex;
uniform vec3 colour;

void main( void )
{
    gl_Position = vec4(vertex.xy, 0, 1);
}
