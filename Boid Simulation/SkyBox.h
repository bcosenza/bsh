// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.


// Note: used tutorial from http://www.mbsoftworks.sk/index.php?page=tutorials&series=1&tutorial=13

#include "shader.h"
#include "simParam.h"
#include "renderable.h"

class Skybox : public Renderable{
public:
	Skybox(const char* directory, const char* sFront, const char* sBack, const char* sLeft, const char* sRight, const char* sTop, const char* sBottom);
	~Skybox();
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();
	
	void toggleVisibility();

private:
	void loadCubeMapSide(GLuint texture, GLenum side_target, const char* file_name);

	Shader* shader;

	GLuint vao;
	GLuint vbo;
	GLuint tex;

	bool visible;

	std::string sDirectory;
	std::string sFront, sBack, sLeft, sRight, sTop, sBottom;
};