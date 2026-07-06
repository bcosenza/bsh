// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _GRID_H_
#define _GRID_H_

#include "common.h"
#include "Renderable.h"
#include "Shader.h"

/*
	Grid draws the uniform acceleration grid (as used by the grid-accelerated
	model, sycl/BoidGridBase) as a wireframe of light-yellow lines. It reuses the
	worldBox line shader and, like WorldBox, is a toggleable Renderable.
*/
class Grid : public Renderable {
	private:
		Shader* shader;
		unsigned int gridBufferObject[2];
		unsigned int gridAttributeObject[1];
		unsigned int vertexNum;

		// true if the grid should be drawn
		bool visibility;

	public:
		// wireframe spanning dimX x dimY x dimZ cells of size cellSize (world
		// units), anchored at WORLD_ORIGIN
		Grid(float cellSize, unsigned int dimX, unsigned int dimY, unsigned int dimZ, bool visible);
		~Grid();
		void render();
		Shader* getShader();
		void bindShader();
		void unbindShader();

		//make the grid visible/invisible
		void toggleVisibility();
};

#endif
