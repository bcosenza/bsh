// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _WORLDGROUND_H_
#define _WORLDGROUND_H_

#include "stdafx.h"
#include "renderable.h"
#include "shader.h"

/*
	Worldground is a renderable plane used to create a better visibility for the 2D versions of the boid simulation.
*/
class WorldGround : public Renderable {
private:
	Shader* shader;
	std::vector<std::string> attribName;

	unsigned int worldGroundBufferObject[2];
	unsigned int worldGroundAttributeObject[1];
	unsigned int vertexNum;

	// ground color
	GLfloat colorGround[4];

	// groud visibility 
	bool visibility;

public:
	WorldGround(bool visible, unsigned int gridSizeX, unsigned int gridSizeY, unsigned int gridSizeZ);
	~WorldGround();
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();
	void toggleVisibility();
};

#endif