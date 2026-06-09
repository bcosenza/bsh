#version 150

in vec3 vp;
uniform mat4 m_transform;
out vec3 texcoords;

void main () {
  texcoords = vp;
  gl_Position =m_transform * vec4 (vp, 1.0);
}