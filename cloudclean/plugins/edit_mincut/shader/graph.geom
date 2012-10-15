#version 330

layout (lines) in;
layout (triangle_strip, max_vertices=4) out;

in vec4 normal[];
out vec4 colour;

uniform vec3 lineColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main(void)
{
    // emit start
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * gl_in[0].gl_Position;
    colour = vec4(lineColour, 1.0f);
    EmitVertex();

    // emit end
    vec4 short_normal = vec4(0.3f, 0.3f, 0.3f, 0.00f) * normal[0];
    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (gl_in[0].gl_Position + short_normal);
    colour = vec4(lineColour, 1.0f);
    EmitVertex();

    EndPrimitive();
}
