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
    vec4 end = gl_in[1].gl_Position;

    // find orhogonal vector to set width along
    vec3 line_sight = end.xyz * 0.5f + start.xyz * 0.5f;
    vec3 line_dir = end.xyz - start.xyz;
    vec4 width_dir = vec4(normalize(cross(line_sight, line_dir)), 0.0f);

    // emit start

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * start;
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (start + 0.005 * width_dir);
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    // emit end

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * end;
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    gl_Position = cameraToClipMatrix * modelToCameraMatrix * (end + 0.005 * width_dir);
    colour = vec4(elColour, 1.0f);
    EmitVertex();

    EndPrimitive();
}
