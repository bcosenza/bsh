// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "BoidModelSimpleView.h"
#include "vectorTypes.h"

/* Create a VBO, upload data and leave it bound (backend-agnostic, raw GL). */
static GLuint makeVBO(const void* data, size_t dataSize){
	GLuint id;
	glGenBuffers(1, &id);
	glBindBuffer(GL_ARRAY_BUFFER, id);
	glBufferData(GL_ARRAY_BUFFER, dataSize, data, GL_DYNAMIC_DRAW);
	return id;
}

void BoidModelSimpleView::createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel){

	std::vector<Vec4> newDataColor(num);
	for (int i = 0; i < num; i++)
		newDataColor[i] = BOID_COLOR;

	GLuint id[1];
	size_t array_size = num * sizeof(Vec4);

	shader = new Shader("boidTri.v.glsl", "boidTri.f.glsl", "boidTri.g.glsl");
	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "color");
	GLint velLoc = glGetAttribLocation(shader->id(), "vel3d");

	//------VBO 1--------- (in)
	glGenVertexArrays(1, &pos_vao[0]);
	glBindVertexArray(pos_vao[0]);

	pos_vbo[0] = makeVBO(&pos[0], array_size);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	vel_vbo[0] = makeVBO(&vel[0], array_size);
	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	//------VBO 2--------- (out)
	glGenVertexArrays(1, &pos_out_vao[0]);
	glBindVertexArray(pos_out_vao[0]);

	pos_out_vbo[0] = makeVBO(&pos[0], array_size);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	vel_out_vbo[0] = makeVBO(&vel[0], array_size);
	glVertexAttribPointer(velLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(velLoc);

	glGenBuffers(1, &id[0]);
	glBindBuffer(GL_ARRAY_BUFFER, id[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* num, &newDataColor[0], GL_STATIC_DRAW);
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
}

void BoidModelSimpleView::destroyGLResources(){
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, pos_vbo);
	glDeleteBuffers(1, pos_out_vbo);

	glBindVertexArray(0);
	glDeleteVertexArrays(1, pos_vao);
	glDeleteVertexArrays(1, pos_out_vao);

	delete shader;
}

GLuint BoidModelSimpleView::getPosVAO(){
	if (helper % 2 == 0)
		return pos_out_vao[0];
	else
		return pos_vao[0];
}

GLuint BoidModelSimpleView::getPosVBO(){
	if (helper % 2 == 0)
		return pos_out_vbo[0];
	else
		return pos_vbo[0];
}

GLuint BoidModelSimpleView::getVelVBO(){
	if (helper % 2 == 0)
		return vel_out_vbo[0];
	else
		return vel_vbo[0];
}

int BoidModelSimpleView::getNumBoid(){
	return num;
}

void BoidModelSimpleView::render(){
	shader->bind();
	glBindVertexArray(getPosVAO());
	glDrawArrays(GL_POINTS, 0, num);
	glBindVertexArray(0);
	shader->unbind();
}

void BoidModelSimpleView::bindShader(){
	shader->bind();
}

void BoidModelSimpleView::unbindShader(){
	shader->unbind();
}

Shader* BoidModelSimpleView::getShader(){
	return shader;
}

void BoidModelSimpleView::getFollowedBoid(unsigned int* boidIndex, Vec4* pos, Vec4* vel){
	Vec4 v;
	GLuint vbo = getVelVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &v);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*vel).set(v.x, v.y, v.z, 0.0);

	Vec4 p;
	vbo = getPosVBO();
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glGetBufferSubData(GL_ARRAY_BUFFER, sizeof(Vec4)* *boidIndex, sizeof(Vec4), &p);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	(*pos).set(p.x, p.y, p.z, 0.0);
}
