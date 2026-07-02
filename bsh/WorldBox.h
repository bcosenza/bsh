// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _WORLDBOX_H_
#define _WORLDBOX_H_

#include "common.h"
#include "Renderable.h"
#include "Shader.h"

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