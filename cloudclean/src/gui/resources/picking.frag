#version 330
 
layout(location = 0) out vec4 fragColor;

in int gl_PrimitiveID​;
in vec4 colour;
out uvec3 FragColor;

void main( void )
{
    fragColor = uvec3(gl_PrimitiveID, 0, 0)​;
}
