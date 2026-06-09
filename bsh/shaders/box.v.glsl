	#version 150

	attribute vec4 coord;
    uniform vec4 color;
    varying vec4 f_color;
	uniform mat4 m_transform;

    void main(void) {
		gl_Position = coord;
		f_color = color;
    }