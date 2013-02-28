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
    pos.y = float(position%height);
    pos.x = float(position/float(height));
    pos.z = 1;

    // put in ndc space
    pos = (pos/vec3(width, height, 1) - 0.5f) * 2.0f;

    /*vec2 s = scale * aspect_ratio;
    vec2 tr = (offset/vec2(width, height) - 0.5f) * 2.0f;
    mat3 camera = mat3(
        s.x, 0.0f, 0.0f,
        0.0f, s.y, 0.0f,
        tr.x, tr.y, 1.0f
    );
    */

    gl_Position = vec4((pos*camera).xy, 0, 1);
}
