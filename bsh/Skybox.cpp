#include "skyBox.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

Skybox::Skybox(const char* directory, const char* sFront, const char* sBack, const char* sLeft, const char* sRight, const char* sTop, const char* sBottom){
	float points[] = {
		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,

		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,

		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,

		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,

		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE,
		-SKYBOX_SIZE, SKYBOX_SIZE, -SKYBOX_SIZE,

		-SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, -SKYBOX_SIZE,
		-SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE,
		SKYBOX_SIZE, -SKYBOX_SIZE, SKYBOX_SIZE
	};

	visible = true;

	shader = new Shader("skybox.v.glsl", "skybox.f.glsl");

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * 36 * sizeof (float), &points, GL_STATIC_DRAW);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	// generate a cube-map texture to hold all the sides
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tex);


	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, sFront);
	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, sBack);
	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, sTop);
	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, sBottom);
	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, sLeft);
	loadCubeMapSide(tex, GL_TEXTURE_CUBE_MAP_POSITIVE_X, sRight);
	// format cube map texture
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

Skybox::~Skybox(){

}

void Skybox::loadCubeMapSide(GLuint texture, GLenum side_target, const char* file_name)
{
	glBindTexture(GL_TEXTURE_CUBE_MAP, texture);

	int x, y, n;
	int force_channels = 4;
	unsigned char*  image_data = stbi_load(
		file_name, &x, &y, &n, force_channels);
	if (!image_data) {
		fprintf(stderr, "ERROR: could not load %s\n", file_name);
	}
	// non-power-of-2 dimensions check
	if ((x & (x - 1)) != 0 || (y & (y - 1)) != 0) {
		fprintf(
			stderr, "WARNING: image %s is not power-of-2 dimensions\n", file_name
			);
	}

	// copy image data into 'target' side of cube map
	glTexImage2D(
		side_target,
		0,
		GL_RGBA,
		x,
		y,
		0,
		GL_RGBA,
		GL_UNSIGNED_BYTE,
		image_data
		);
	free(image_data);
}

void Skybox::render(){
	if (visible){
		glDepthMask(GL_FALSE);
		shader->bind();
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, tex);
		glBindVertexArray(vao);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		shader->unbind();
		glDepthMask(GL_TRUE);
	}
}

void Skybox::bindShader(){
	shader->bind();
}

void Skybox::unbindShader(){
	shader->unbind();
}

Shader* Skybox::getShader(){
	return shader;
}

void Skybox::toggleVisibility(){
	visible = !visible;
}