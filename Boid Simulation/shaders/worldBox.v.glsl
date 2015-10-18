	#version 150
	
	attribute vec4 coord3d;
    attribute vec4 v_color;
    varying vec4 f_color;
    uniform mat4 m_transform;

    void main(void) {
		gl_Position = m_transform * coord3d;
		f_color = v_color;
    }