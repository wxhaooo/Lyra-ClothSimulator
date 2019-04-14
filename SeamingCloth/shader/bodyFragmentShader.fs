#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_diffuse1;
uniform bool drawTexture;

void main()
{    
	if(drawTexture)
    	FragColor = texture(texture_diffuse1, TexCoords);
    else
    	FragColor = vec4(1.0,1.0,1.0,1.0);
}