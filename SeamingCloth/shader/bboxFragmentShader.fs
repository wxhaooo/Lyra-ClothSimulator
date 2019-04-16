#version 330 core
out vec4 FragColor;

uniform vec3 RGB;

void main()
{    
	//FragColor = vec4(1.,0.,1.,1.0);
	FragColor = vec4(RGB,1.0);
}