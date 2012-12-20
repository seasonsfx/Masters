#version 330

layout(location = 0) in vec2 point;
out vec4 colour;
uniform float depth;
uniform vec4 planeColour;
uniform mat4 modelview;
uniform mat4 projection;

void main( void )
{
    vec4 vertex_out = vec4(point.x, point.y, depth , 1.0f);
    colour = planeColour;
    gl_Position = projection *vertex_out;
}
