#version 330

layout(location = 0) in vec4 vertex;
layout(location = 1) in float intensity;
layout(location = 2) in int color_index;
layout(location = 3) in int flags;

uniform samplerBuffer sampler;
uniform mat4 projection;
uniform mat4 modelview;

void main( void ) {
//    vec4 layer_colour = texelFetch(sampler, color_index);

    // Adjust the colour intensity
    int layer_id = color_index;
    bool is_selected = bool(flags);


    // Selections will make this fail
//    if(layer_colour.a == 0) {
//        gl_Position = vec4(1e6, 1e6, 1e6, 1);
//        return;
//    }

    gl_Position = projection * modelview * vertex;
}
