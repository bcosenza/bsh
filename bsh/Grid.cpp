// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "Grid.h"
#include "SimParam.h"
#include "vectorTypes.h"

// light yellow, slightly transparent so the flock stays readable through it
static const Vec4 GRID_COLOR(1.0f, 1.0f, 0.6f, 0.4f);

Grid::Grid(float cellSize, unsigned int dimX, unsigned int dimY, unsigned int dimZ, bool visible){
	// two vertices per line; (dim+1)^2 lines for each of the three axis families
	vertexNum = 2 * ((dimX + 1) * (dimY + 1) + (dimX + 1) * (dimZ + 1) + (dimZ + 1) * (dimY + 1));
	visibility = visible;

	std::vector<Vec4> newDataVertex(vertexNum);
	std::vector<Vec4> newDataColor(vertexNum);

	int i = 0;
	float posX, posY, posZ;
	const float minX = WORLD_ORIGIN_X, minY = WORLD_ORIGIN_Y, minZ = WORLD_ORIGIN_Z;
	const float maxX = minX + cellSize * dimX;
	const float maxY = minY + cellSize * dimY;
	const float maxZ = minZ + cellSize * dimZ;
	const float eps = 0.5f;   // absorb float accumulation so the last line is included

	// lines parallel to Z (one per (x,y) grid corner)
	for (posX = minX; posX <= maxX + eps; posX += cellSize)
		for (posY = minY; posY <= maxY + eps; posY += cellSize){
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(posX, posY, minZ, 1.f);
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(posX, posY, maxZ, 1.f);
		}

	// lines parallel to X (one per (y,z) grid corner)
	for (posY = minY; posY <= maxY + eps; posY += cellSize)
		for (posZ = minZ; posZ <= maxZ + eps; posZ += cellSize){
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(minX, posY, posZ, 1.f);
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(maxX, posY, posZ, 1.f);
		}

	// lines parallel to Y (one per (x,z) grid corner)
	for (posX = minX; posX <= maxX + eps; posX += cellSize)
		for (posZ = minZ; posZ <= maxZ + eps; posZ += cellSize){
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(posX, minY, posZ, 1.f);
			newDataColor[i] = GRID_COLOR; newDataVertex[i++] = Vec4(posX, maxY, posZ, 1.f);
		}

	shader = new Shader("worldBox.v.glsl", "worldBox.f.glsl");

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "v_color");

	glGenVertexArrays(1, &gridAttributeObject[0]);
	glBindVertexArray(gridAttributeObject[0]);

	glGenBuffers(1, &gridBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, gridBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * vertexNum, &newDataVertex[0], GL_STATIC_DRAW);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);

	glGenBuffers(1, &gridBufferObject[1]);
	glBindBuffer(GL_ARRAY_BUFFER, gridBufferObject[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * vertexNum, &newDataColor[0], GL_STATIC_DRAW);
	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
}

void Grid::render(){
	if (visibility){
		shader->bind();
		glBindVertexArray(gridAttributeObject[0]);
		glDrawArrays(GL_LINES, 0, vertexNum);
		glBindVertexArray(0);
		shader->unbind();
	}
}

void Grid::bindShader(){
	shader->bind();
}

void Grid::unbindShader(){
	shader->unbind();
}

Shader* Grid::getShader(){
	return shader;
}

Grid::~Grid(){
}

void Grid::toggleVisibility(){
	visibility = !visibility;
}
