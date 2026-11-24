// Copyright (c) 2015-2026, Biagio Cosenza.
// Technische Universitaet Berlin (2015-2019). University of Salerno (2019-2026). All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _PLATELANES_H_
#define _PLATELANES_H_

#include "common.h"
#include "Renderable.h"
#include "Shader.h"
#include "vectorTypes.h"

/*
	The thin plates of SH accuracy scene 9: one axis-aligned plate per lane
	(AccuracyLanes.h), normal along x, centred on the lane centre. Renders them as
	grey boxes (column shader) and discretises them into obstacle points for the
	obstacle-SH engine.

	Each face of a plate carries a grid of obstacle points (PLATE_POINT_SPACING
	apart) on the face itself. An interior point has one repulsion vector, the face
	normal; a point on an edge adds the outward edge direction and the diagonal
	between the two, a corner both edges (as the column bundles). Points are
	written plate by plate, so lane L owns points [L * pointsPerPlate(), (L+1) * ...).
*/
class PlateLanes : public Renderable {
private:
	Shader* shader;
	unsigned int vbo[1];
	unsigned int vao[1];
	unsigned int vertexNum;
	bool visibility;

	std::vector<Vec4> centres;   // one per lane
	Vec4 halfExtent;             // x = half thickness, y = half height, z = half width

public:
	// worldX / worldZ: world extent, the lanes sit at its x/z centre
	PlateLanes(bool visible, float worldX, float worldZ);
	~PlateLanes();
	void render();
	Shader* getShader();
	void bindShader();
	void unbindShader();
	void setVisibility(bool visible);

	const std::vector<Vec4>& plateCentres() const { return centres; }
	Vec4 plateHalfExtent() const { return halfExtent; }

	// obstacle points of all plates (same layout as Column::getObstacleForce)
	void getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start,
	                      std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle) const;
	int pointsPerPlate() const;
};

#endif // _PLATELANES_H_
