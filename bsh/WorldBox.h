// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _WORLDBOX_H_
#define _WORLDBOX_H_

#include "stdafx.h"
#include "renderable.h"
#include "shader.h"

/* 
	Worldbox is renderable a cube which outlines the space for the model. 
*/
class WorldBox : public Renderable{
	private:
		Shader* shader;
		std::vector<std::string> attribName;

		unsigned int worldBoxBufferObject[2];
		unsigned int worldBoxAttributeObject[1];
		unsigned int vertexNum;

		// true if the cube should be drawn
		bool visibility;

	public:
		// create world box with cell size * grid size on each axis. a line is drawn at every position where pos = factor * gridSize * cellSize per axis
		WorldBox(unsigned int factor, bool visible, unsigned int gridSizeX, unsigned int gridSizeY, unsigned int gridSizeZ);
		~WorldBox();
		void render();
		Shader* getShader();
		void bindShader();
		void unbindShader();
		
		//make the cube visible/invisible
		void toggleVisibility();
};

#endif