#version 330

layout (points) in;
layout (triangle_strip, max_vertices=4) out;

uniform int width;
uniform int height;
uniform mat3 camera;

in vec4 colour[];
out vec4 fcolour;

void main(void){
    vec4 point = gl_in[0].gl_Position;
    fcolour = colour[0];

    vec3 pd = vec3(2.0/width, 2.0/height, 0);
    pd = camera * pd;

    gl_Position = point + vec4(-pd.x, pd.y, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(-pd.x, -pd.y, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(pd.x, pd.y, 0, 0);
    EmitVertex();

    gl_Position = point + vec4(pd.x, -pd.y, 0, 0);
    EmitVertex();

}
