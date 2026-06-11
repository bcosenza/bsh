#version 150

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

in vec4 gColor[]; 
in vec4 gVel[];

out vec4 fColor;
uniform mat4 m_transform;

void main() {
    fColor = gColor[0];

	// constant on-screen size, oriented along the (normalized) velocity
	float bodyLength = 5.0f;
	float halfWidth = 1.25f;

	vec3 v = gVel[0].xyz;
	vec3 dir = v / max(length(v), 0.0001f);

	float x = dir.x;
	float z = dir.z;
	float y = dir.y;

	vec4 nose = vec4(dir, 0.0f) * bodyLength;
	vec4 p = vec4(-z, 0.0f, x, 0.0f) * halfWidth;
	vec4 p2 = vec4(0.0f, -z, y, 0.0f) * halfWidth;

    gl_Position = m_transform * (gl_in[0].gl_Position + nose);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position + p);
    EmitVertex();

	gl_Position = m_transform * (gl_in[0].gl_Position - p);
    EmitVertex();

    EndPrimitive();

	gl_Position = m_transform * (gl_in[0].gl_Position + nose);
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