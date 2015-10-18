	#version 150
	
	in vec4 vel3d;
	in vec4 coord3d;
    in vec4 color;

	out vec4 gColor;
	out vec4 gVel;



    void main(void) {
		gl_Position = coord3d;
		gColor = color;
		gVel = normalize(vel3d);
	}