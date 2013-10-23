#version 330

layout(location = 0) in vec4 vertex;
layout(location = 1) in float intensity;
layout(location = 2) in int color_index;
layout(location = 3) in int flags;
uniform samplerBuffer sampler;
uniform mat4 projection;
uniform mat4 modelview;
out vec4 colour;

const vec4 select_color = vec4(0.0f, 0.0f, 1.0f, 1.0f);
const vec4 select_color2 = vec4(1.0f, 0.0f, 0.0f, 1.0f);

const int SELECTED = 0x001;
const int SELECTED2 = 0x002;

void main( void ) {
    vec4 layer_colour = texelFetch(sampler, color_index);
    colour = layer_colour * intensity;

    if(bool(flags & SELECTED) && bool(flags & SELECTED2)){
        colour = colour * 0.7f + select_color * 0.15f + select_color2 * 0.15f;
    } else if (bool(flags & SELECTED)) {
        colour = colour * 0.7f + select_color * 0.3f;
    } else if (bool(flags & SELECTED2)) {
        colour = colour * 0.7f + select_color2 * 0.3f;
    }

    // Selections will make this fail
    if(colour.a == 0) {
        gl_Position = vec4(1e6, 1e6, 1e6, 1);
        return;
    }

    gl_Position = projection * modelview * vertex;
}
