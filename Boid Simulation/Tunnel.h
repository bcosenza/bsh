// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _TUNNEL_H_
#define _TUNNEL_H_

#include "stdafx.h"
#include "renderable.h"
#include "shader.h"
#include "vectorTypes.h"

/* 
	Renderable tunnel displayed for the tunnel test banchmark. 
*/
class Tunnel : public Renderable{
private:
	Shader* shader;
	std::vector<std::string> attribName;

	GLfloat colorTunnel[4];
	GLfloat colorLines[4];

	unsigned int tunnelBufferObject[1];
	unsigned int tunnelAttributeObject[1];
	unsigned int vertexNum;

	unsigned int tunnelLineBufferObject[1];
	unsigned int tunnelLineAttributeObject[1];
	unsigned int vertexLineNum;

	//true if the cube should be drawn
	bool visibility;

	float xMin;
	float xMax;

	float yMin;
	float yMax;

	float zMin;
	float zMax;

	unsigned int width;
	float lHeight;
	float xTunnel1;
	float xTunnel2;
	float xTunnel3;
	float xTunnel4;

	float yTunnel1;
	float yTunnel2;

	float xSize, ySize, zSize;

public:
	//create world box with cell size * grid size on each axis. a line is drawn at every position where pos = factor * gridSize * cellSize per axis.
	Tunnel(bool visible, float cellSizeX, float cellSizeY, float cellSizeZ, int posX, int posY, int posZ, int hH);
	~Tunnel();
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

	//make the cube visible/invisible
	void setVisibility(bool visibile);
	void getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle, unsigned int offset);
};

#endif