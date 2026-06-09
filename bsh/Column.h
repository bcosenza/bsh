// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _COLUMN_H_
#define _COLUMN_H_

#include "stdafx.h"
#include "renderable.h"
#include "shader.h"
#include "vectorTypes.h"

/*
	Worldbox is a renderable cube which outlines the space for the model.
*/
class Column : public Renderable{
private:
	Shader* shader;
	std::vector<std::string> attribName;

	unsigned int columnBufferObject[1];
	unsigned int columnAttributeObject[1];
	unsigned int vertexNum;

	// true to draw the cube
	bool visibility;

	float xMin;
	float xMax;

	float yMin;
	float yMax;

	float zMin;
	float zMax;

	unsigned int h;
	float lHeight;

public:
	// create world box with cell size * grid size on each axis. A line is drawn at every position where pos = factor * gridSize * cellSize per axis.
	Column(bool visible, float cellSizeX, float cellSizeY, float cellSizeZ, int posX, int posY, int posZ, int hH);
	~Column();
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();
	// make the cube visible/invisible
	void setVisibility(bool visibile);
	void getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle, unsigned int offset);
};

#endif