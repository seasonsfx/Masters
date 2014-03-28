#version 330

layout(location = 0) out uvec3 fragColor;

void main( void ){
    fragColor = uvec3(gl_PrimitiveID, 0, 0);
}
