// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.


#ifndef _RENDERABLE_H_
#define _RENDERABLE_H_

#include "common.h"

class Shader;

/*
	Abstract class (interface), implemented by classes to be rendered.
*/
class Renderable {
	public:
		//method which should draw the vbo
		virtual void render() = 0;
		//return shader of object
		virtual Shader* getShader() = 0;
		//bind the object shader to pipeline
		virtual void bindShader() = 0;
		//unbind the object shader from pipeline
		virtual void unbindShader() = 0;
};

#endif