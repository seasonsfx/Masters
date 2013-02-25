#version 330

layout(location = 1) in float intensity;
layout(location = 2) in int color_index;
layout(location = 3) in int flags;
layout(location = 4) in int position;

uniform samplerBuffer sampler;
uniform vec4 select_color;
uniform int width;
uniform int height;

out vec4 colour;

const int SELECTED = 0x001;

void main( void ) {
    vec4 layer_colour = texelFetch(sampler, color_index);
    colour = layer_colour * intensity;
    if(bool(flags & SELECTED)){
        colour = colour * 0.7f + select_color * 0.3f;
    }

    float x = float(position%width);
    float y = float(position/width);

    // put in ndc space
    x = (x - width/2.0f) * 2.0f;
    y = -(y - height/2.0f) * 2.0f;

    gl_Position = vec4(x, y, 0, 1);
}
