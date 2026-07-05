// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _BOIDMODELSIMPLEVIEW_H_
#define _BOIDMODELSIMPLEVIEW_H_

#include "common.h"
#include "BoidModel.h"
#include "vectorTypes.h"

/*
	GL rendering (shared by both OpenCL and SYCL).
	It implements the double-buffered pos/vel VBO/VAO sets, the boidTri shader and all
	the Renderable plumbing (creation, ping-pong VAO selection, drawing, boid pick-up). 
	The OpenCL (cl/BoidModelSimple) and SYCL (sycl/BoidModelSimpleSYCL) models derive from this and 
	add only their backend-specific compute (device buffers, simulate() and the timing, which stay 
	pure virtual here().
*/
class BoidModelSimpleView : public BoidModel
{
public:
	virtual ~BoidModelSimpleView() {}

	// Renderable / BoidModel interface implemented here (shared by both backends)
	GLuint getPosVBO();
	GLuint getVelVBO();
	GLuint getPosVAO();
	int getNumBoid();
	void getFollowedBoid(unsigned int* boidIndex, Vec4 *pos, Vec4 *vel);
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();

protected:
	BoidModelSimpleView() : helper(0), num(0), shader(NULL) {}

	/* Create the two VAO/VBO sets (in/out ping-pong) + boidTri shader. */
	void createVboBindShader(std::vector<Vec4> pos, std::vector<Vec4> vel, std::vector<Vec4> color = std::vector<Vec4>());
	/* Delete the GL objects and the shader (call from the concrete destructor). */
	void destroyGLResources();

	// helper is used to switch between input and output position buffer
	int helper;
	GLuint pos_vbo[1];
	GLuint pos_vao[1];
	GLuint pos_out_vbo[1];
	GLuint pos_out_vao[1];
	GLuint vel_vbo[1];
	GLuint vel_out_vbo[1];

	// number of boids
	int num;

	// boidTri shader and its attribute names
	Shader* shader;
	std::vector<std::string> attribName;
};

#endif // _BOIDMODELSIMPLEVIEW_H_
