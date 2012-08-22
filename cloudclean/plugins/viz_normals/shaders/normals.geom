#version 330

layout (points) in;
layout (line_strip, max_vertices=2) out;

in vec4 normal[];
out vec4 colour;

uniform vec3 lineColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main(void)
{
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * gl_in[0].gl_Position;
    colour = vec4(lineColour, 1.0f);
    EmitVertex();
    
    vec4 short_normal = vec4(0.09f, 0.09f, 0.09f, 0.00f) * normal[0];
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (gl_in[0].gl_Position + short_normal);
    colour = vec4(lineColour, 1.0f);
    EmitVertex();
    
    EndPrimitive();
}
