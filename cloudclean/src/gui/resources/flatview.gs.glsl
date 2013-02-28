#version 330

layout (points) in;
//layout (triangle_strip, max_vertices=4) out;
layout (triangle_strip, max_vertices=4) out;

uniform int width;
uniform int height;
uniform float scale;
uniform vec2 aspect_ratio;

in vec4 colour[];
out vec4 fcolour;

void main(void){
    vec4 point = gl_in[0].gl_Position;
    fcolour = colour[0];

    float w = aspect_ratio.x*scale*(2.0/width);
    float h = aspect_ratio.y*scale*(2.0/height);

    gl_Position = point + vec4(-w, h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(-w, -h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(w, h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(w, -h, 0, 0);
    EmitVertex();

}
