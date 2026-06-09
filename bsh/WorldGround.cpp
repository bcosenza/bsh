#include "stdafx.h"
#include "worldGround.h"
#include "simParam.h"
#include "vectorTypes.h"

WorldGround::WorldGround(bool visible, unsigned int gridSizeX, unsigned int gridSizeY, unsigned int gridSizeZ){
	visibility = visible;
	std::vector<Vec4> newDataVertex(4);
	
	colorGround[0] = .9f;
	colorGround[1] = .9f;
	colorGround[2] = .9f;
	colorGround[3] = 1.0f;

	float x = gridSizeX * CELL_SIZE_X;
	float z = gridSizeZ * CELL_SIZE_Z;
	float y = -0.5f;

	newDataVertex[0] = Vec4(0.0f, y, 0.0f, 1.0f);
	newDataVertex[1] = Vec4(x, y, 0.0f, 1.0f);
	newDataVertex[2] = Vec4(x, y, z, 1.0f);
	newDataVertex[3] = Vec4(0.0f, y, z, 1.0f);

	shader = new Shader("worldGround.v.glsl", "worldGround.f.glsl");

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetUniformLocation(shader->id(), "color");


	glGenVertexArrays(1, &worldGroundAttributeObject[0]); // Create our Vertex Array Object  
	glBindVertexArray(worldGroundAttributeObject[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &worldGroundBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, worldGroundBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &newDataVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	shader->bind();
	glUniform4fv(colorLoc, 1, colorGround);
	shader->unbind();
}

void WorldGround::render(){
	if (visibility){
		shader->bind();
		glBindVertexArray(worldGroundAttributeObject[0]);
		glDrawArrays(GL_QUADS, 0, 4);
		glBindVertexArray(0);
		shader->unbind();
	}
}

void WorldGround::bindShader(){
	shader->bind();
}

void WorldGround::unbindShader(){
	shader->unbind();
}

Shader* WorldGround::getShader(){
	return shader;
}

WorldGround::~WorldGround(){

}

void WorldGround::toggleVisibility(){
	visibility = !visibility;
}