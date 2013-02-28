#version 330
layout(location = 0) in vec4 vertex;
layout(location = 1) in float intensity;
layout(location = 2) in int color_index;
layout(location = 3) in int flags;
layout(location = 4) in int position;

uniform samplerBuffer sampler;
uniform vec4 select_color;
uniform int width;
uniform int height;
uniform float scale;
uniform vec2 offset;
uniform vec2 aspect_ratio;

out vec4 colour;

const int SELECTED = 0x001;

void main( void ) {
    vec4 layer_colour = texelFetch(sampler, color_index);
    colour = layer_colour * intensity;
    if(bool(flags & SELECTED)){
        colour = colour * 0.7f + select_color * 0.3f;
    }

    float y = float(position%height) - offset.y;
    float x = float(position/float(height)) + offset.x;

    // put in ndc space
    x = (x/width - 0.5f) * 2.0f;
    y = (y/height - 0.5f) * 2.0f;

    x *= scale * aspect_ratio.x;
    y *= scale * aspect_ratio.y;

    gl_Position = vec4(x, y, 0, 1);
}
