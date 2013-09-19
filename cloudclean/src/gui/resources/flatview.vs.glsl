#version 330
layout(location = 0) in float intensity;
layout(location = 1) in int color_index;
layout(location = 2) in int flags;
layout(location = 3) in int position;

uniform samplerBuffer sampler;
uniform vec4 select_color;
//uniform int width;
uniform int height;
uniform mat3 camera;

out vec4 colour;

const int SELECTED = 0x001;

void main( void ) {
    vec4 layer_colour = texelFetch(sampler, color_index);
    colour = layer_colour * intensity;
    if(bool(flags & SELECTED)){
        colour = colour * 0.7f + select_color * 0.3f;
    }

    vec3 pos;
    pos.y = mod(position, height);
    pos.x = float(position/float(height));
    pos.z = 1;

    gl_Position = vec4((camera*pos).xy, 0, 1);
}
