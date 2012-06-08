#version 330
 
layout(location = 0) in vec2 vertex;
out vec4 colour;
uniform mat4 ortho;

void main( void )
{
    vec4 vertex_out = vec4(vertex.x, vertex.y, 1.0f, 1.f);
    colour = vec4(0.0f, 1.0f, 0.0f, 1.0f);
    gl_Position = ortho * vertex_out;
    /*mat4 orth = mat4(
                1.0f, 0.0f, 0.0f, 1.0f,
                0.0f, 1.0f, 0.0f, 1.0f,
                0.0f, 0.0f, 1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, 1.0f
                );
    gl_Position = vertex_out;
    */
}
