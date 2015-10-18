// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.


#ifndef _RENDERABLE_H_
#define _RENDERABLE_H_

#include "stdafx.h"

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