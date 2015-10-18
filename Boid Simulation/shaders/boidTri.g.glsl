#version 150

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

in vec4 gColor[]; 
in vec4 gVel[];

out vec4 fColor;
uniform mat4 m_transform;

void main() {
    fColor = gColor[0]; 

	float x = gVel[0].x;
	float z = gVel[0].z;
	float y = gVel[0].y;

	vec4 p = vec4(-z, 0.0f, x, 0.0f) / 6;
	vec4 p2 = vec4(0.0f, -z, y, 0.0f) / 6;

    gl_Position = m_transform * (gl_in[0].gl_Position + gVel[0]);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position + p);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position - p);
    EmitVertex();

    EndPrimitive();

	gl_Position = m_transform * (gl_in[0].gl_Position + gVel[0]);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position + p2);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position - p2);
    EmitVertex();

    EndPrimitive();
}


/*layout(points) in;
layout(line_strip, max_vertices = 2) out;

in vec4 gColor[]; 
in vec4 gVel[];

out vec4 fColor;

void main() {
    fColor = gColor[0]; 

    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

	gl_Position = gl_in[0].gl_Position + gVel[0];
    EmitVertex();

    EndPrimitive();
}*/