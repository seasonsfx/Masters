#version 330

layout (points) in;
layout (line_strip, max_vertices=2) out;

in vec4 normal[];
out vec4 colour;

uniform vec3 line_colour;
uniform mat4 proj;
uniform mat4 mv;

void main(void)
{
    gl_Position = proj * mv * gl_in[0].gl_Position;
    colour = vec4(line_colour, 1.0f);
    EmitVertex();
    
    vec4 short_normal = vec4(0.3f, 0.3f, 0.3f, 0.00f) * normal[0];
    gl_Position = proj * mv * (gl_in[0].gl_Position + short_normal);
    colour = vec4(line_colour, 1.0f);
    EmitVertex();
    
    EndPrimitive();
}
