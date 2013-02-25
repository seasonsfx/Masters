#version 330

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

uniform int width;
uniform int height;

out vec4 colour;

void main(void){
    vec4 point = gl_in[0].gl_Position;
    colour = vec4(gl_in[0].colour);

    float w = 2.0/width;
    float h = 2.0/height;

    gl_Position = point + vec4(-w, -h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(-w, h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(w, -h, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(w, h, 0, 0);
    EmitVertex();
}
