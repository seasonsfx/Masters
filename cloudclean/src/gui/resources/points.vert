#version 330
 
layout(location = 0) in vec4 vertex;
layout(location = 1) in float intensity;
layout(location = 2) in int color_index;
uniform samplerBuffer sampler;
uniform mat4 projection;
uniform mat4 modelview;
out vec4 colour;

void main( void )
{
    vec4 layer_colour;
    layer_colour = texelFetch(sampler, color_index);
    colour = layer_colour;
    gl_Position = projection * modelview * vertex;
}
