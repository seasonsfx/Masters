#version 330
 
layout(location = 0) in vec3 point;
out vec4 colour;
uniform mat4 ortho;

void main( void )
{
    vec4 vertex_out = vec4(point.x, point.y, 1.0f, 1.0f);
    colour = vec4(0.0f, 1.0f, 0.0f, 1.0f);
    gl_Position = ortho * vertex_out;
}
