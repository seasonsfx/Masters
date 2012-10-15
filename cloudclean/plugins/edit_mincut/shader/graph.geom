#version 330

layout (lines) in;
layout (triangle_strip, max_vertices=4) out;

in float line_width[];
out vec4 colour;

uniform vec3 elColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main(void)
{
    float width = line_width[0];
    vec4 start = gl_in[0].gl_Position;
    vec4 end = gl_in[0].gl_Position;

    // emit start

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * start;
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (start + vec4(0.1f, 0.1f, 0.1f, 0.0f));
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    // emit end

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * end;
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (end + vec4(0.1f, 0.1f, 0.1f, 0.0f));
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    EndPrimitive();
}
