#version 330
 
layout(location = 0) in vec4 vertex;
layout(location = 1) in int color_index;
uniform samplerBuffer sampler;
uniform mat4 projection;
uniform mat4 modelview;
out vec4 colour;

void main( void )
{
    vec4 layer_colour = texelFetch(sampler, color_index);
    vec4 vertex_out = vec4(vertex.x, vertex.y, vertex.z, 1.0f);
    //colour = vec4(vertex.w, vertex.w, vertex.w, 1.0f) * layer_colour;
    colour = vec4(0, 0, 0, 1.0f);
    gl_Position = vec4(0, 0, 0.5, 1);
    //gl_Position = vertex_out;
    //gl_Position = projection * modelview * vertex_out;
}
