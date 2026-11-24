// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#include "common.h"
#include "PlateLanes.h"
#include "AccuracyLanes.h"
#include <cmath>

// samples per face along y and z (points at the edges included)
static int samplesY(){ return (int)std::lround(2.0f * AccLanes::PLATE_HALF_Y / AccLanes::PLATE_POINT_SPACING) + 1; }
static int samplesZ(){ return (int)std::lround(2.0f * AccLanes::PLATE_HALF_Z / AccLanes::PLATE_POINT_SPACING) + 1; }

PlateLanes::PlateLanes(bool visible, float worldX, float worldZ){
	visibility = visible;
	halfExtent = Vec4(AccLanes::PLATE_HALF_X, AccLanes::PLATE_HALF_Y, AccLanes::PLATE_HALF_Z, 0.0f);
	for (int l = 0; l < AccLanes::LANES; l++)
		centres.push_back(Vec4(0.5f * worldX, AccLanes::laneY(l), 0.5f * worldZ, 1.0f));

	// six quads per plate, as Column draws its box
	vertexNum = 24 * AccLanes::LANES;
	std::vector<Vec4> v;
	v.reserve(vertexNum);
	for (const Vec4& c : centres){
		const float x0 = c.x - halfExtent.x, x1 = c.x + halfExtent.x;
		const float y0 = c.y - halfExtent.y, y1 = c.y + halfExtent.y;
		const float z0 = c.z - halfExtent.z, z1 = c.z + halfExtent.z;
		const Vec4 quads[24] = {
			Vec4(x0, y0, z0, 1), Vec4(x1, y0, z0, 1), Vec4(x1, y1, z0, 1), Vec4(x0, y1, z0, 1),
			Vec4(x1, y0, z0, 1), Vec4(x1, y0, z1, 1), Vec4(x1, y1, z1, 1), Vec4(x1, y1, z0, 1),
			Vec4(x1, y0, z1, 1), Vec4(x0, y0, z1, 1), Vec4(x0, y1, z1, 1), Vec4(x1, y1, z1, 1),
			Vec4(x0, y0, z0, 1), Vec4(x0, y0, z1, 1), Vec4(x0, y1, z1, 1), Vec4(x0, y1, z0, 1),
			Vec4(x0, y0, z0, 1), Vec4(x1, y0, z0, 1), Vec4(x1, y0, z1, 1), Vec4(x0, y0, z1, 1),
			Vec4(x0, y1, z0, 1), Vec4(x1, y1, z0, 1), Vec4(x1, y1, z1, 1), Vec4(x0, y1, z1, 1) };
		v.insert(v.end(), quads, quads + 24);
	}

	GLfloat color[4] = { 0.6f, 0.6f, 0.6f, 1.0f };
	shader = new Shader("column.v.glsl", "column.f.glsl");
	GLint vertLoc  = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetUniformLocation(shader->id(), "color");

	glGenVertexArrays(1, &vao[0]);
	glBindVertexArray(vao[0]);
	glGenBuffers(1, &vbo[0]);
	glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4) * vertexNum, v.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(vertLoc);
	glBindVertexArray(0);

	shader->bind();
	glUniform4fv(colorLoc, 1, color);
	shader->unbind();
}

PlateLanes::~PlateLanes(){
	glDeleteBuffers(1, &vbo[0]);
	glDeleteVertexArrays(1, &vao[0]);
	delete shader;
}

void PlateLanes::render(){
	if (visibility){
		shader->bind();
		glBindVertexArray(vao[0]);
		glDrawArrays(GL_QUADS, 0, vertexNum);
		glBindVertexArray(0);
		shader->unbind();
	}
}

Shader* PlateLanes::getShader(){ return shader; }
void PlateLanes::bindShader(){ shader->bind(); }
void PlateLanes::unbindShader(){ shader->unbind(); }
void PlateLanes::setVisibility(bool visible){ visibility = visible; }

int PlateLanes::pointsPerPlate() const {
	return 2 * samplesY() * samplesZ();
}

void PlateLanes::getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start,
                                  std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle) const {
	const int ny = samplesY(), nz = samplesZ();
	const float L = AccLanes::PLATE_VECTOR_LEN;
	const float d = L / std::sqrt(2.0f);   // diagonal components, same length L
	cor->clear(); start->clear(); end->clear(); posObstacle->clear();

	for (const Vec4& c : centres){
		for (int face = -1; face <= 1; face += 2){
			for (int iy = 0; iy < ny; iy++){
				for (int iz = 0; iz < nz; iz++){
					const float y = -halfExtent.y + iy * (2.0f * halfExtent.y) / (ny - 1);
					const float z = -halfExtent.z + iz * (2.0f * halfExtent.z) / (nz - 1);
					const int ey = (iy == 0) ? -1 : (iy == ny - 1) ? 1 : 0;   // on a y edge
					const int ez = (iz == 0) ? -1 : (iz == nz - 1) ? 1 : 0;   // on a z edge

					posObstacle->push_back(Vec4(c.x + face * halfExtent.x, c.y + y, c.z + z, 0.0f));
					start->push_back((unsigned int)cor->size());
					cor->push_back(Vec4(face * L, 0.0f, 0.0f, 0.0f));                 // face normal
					if (ey){
						cor->push_back(Vec4(0.0f, ey * L, 0.0f, 0.0f));               // edge outward
						cor->push_back(Vec4(face * d, ey * d, 0.0f, 0.0f));           // diagonal
					}
					if (ez){
						cor->push_back(Vec4(0.0f, 0.0f, ez * L, 0.0f));
						cor->push_back(Vec4(face * d, 0.0f, ez * d, 0.0f));
					}
					end->push_back((unsigned int)cor->size());
				}
			}
		}
	}
}
