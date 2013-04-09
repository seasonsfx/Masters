#version 330

layout (points) in;
layout (line_strip, max_vertices=2) out;

in vec4 normal[];
out vec4 colour;

uniform vec3 line_colour;
uniform mat4 proj;
uniform mat4 mv;

void main(void) {
    vec4 n = normal[0];
    gl_Position = proj * mv * gl_in[0].gl_Position;
    colour = vec4(line_colour, 1.0f);

    // set normal red if it looks wrong
    /*float len = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
    if(len < 0.95 || len > 1.05){
        colour = vec4(1, 0, 0, 1.0f);
        n = vec4(0, 0, 1, 0);
    }*/

    EmitVertex();
    
    vec4 short_normal = vec4(0.1f, 0.1f, 0.1f, 0.00f) * n;
    gl_Position = proj * mv * (gl_in[0].gl_Position + short_normal);
    colour = vec4(line_colour, 1.0f);
    EmitVertex();
    
    EndPrimitive();
}
