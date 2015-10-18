#include "stdafx.h"
#include "worldBox.h"
#include "simParam.h"
#include "vectorTypes.h"

WorldBox::WorldBox(unsigned int factor, bool visible, unsigned int gridSizeX, unsigned int gridSizeY, unsigned int gridSizeZ){
	vertexNum = 2*((gridSizeX / factor + 1) * (gridSizeY / factor + 1) + (gridSizeX / factor + 1) * (gridSizeZ / factor + 1) + (gridSizeZ / factor + 1) * (gridSizeY / factor + 1));
	
	visibility = visible;

	std::vector<Vec4> newDataVertex(vertexNum);
	std::vector<Vec4> newDataColor(vertexNum);

	int i = 0;
	float posX, posY, posZ;
	float maxPosX = WORLD_ORIGIN_X + CELL_SIZE_X * gridSizeX;
	float maxPosY = WORLD_ORIGIN_Y + CELL_SIZE_Y * gridSizeY;
	float maxPosZ = WORLD_ORIGIN_Z + CELL_SIZE_Z * gridSizeZ;

	if (gridSizeX <= 1)
		maxPosX = WORLD_ORIGIN_X;

	if (gridSizeY <= 1)
		maxPosY = WORLD_ORIGIN_Y;

	if (gridSizeZ <= 1)
		maxPosZ = WORLD_ORIGIN_Z;

		// Render iso grid lines in each direction, that make up the visible box.

	for (posX = WORLD_ORIGIN_X; posX <= maxPosX; posX += CELL_SIZE_X * factor){
		for (posY = WORLD_ORIGIN_Y; posY <= maxPosY; posY += CELL_SIZE_Y * factor){
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(posX, posY, WORLD_ORIGIN_Z, 1.f);
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(posX, posY, maxPosZ, 1.f);
		}
	}


	for (posY = WORLD_ORIGIN_Y; posY <= maxPosY; posY += CELL_SIZE_Y * factor){
		for (posZ = WORLD_ORIGIN_Z; posZ <= maxPosZ; posZ += CELL_SIZE_Z * factor){
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(WORLD_ORIGIN_X, posY, posZ, 1.f);
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(maxPosX, posY, posZ, 1.f);
		}
	}

	for (posX = WORLD_ORIGIN_X; posX <= maxPosX; posX += CELL_SIZE_X * factor){
		for (posZ = WORLD_ORIGIN_Z; posZ <= maxPosZ; posZ += CELL_SIZE_Z * factor){
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(posX, WORLD_ORIGIN_Y, posZ, 1.f);
			newDataColor[i] = Vec4(0.796f, 0.659f, 0.0f, .8f);
			newDataVertex[i++] = Vec4(posX, maxPosY, posZ, 1.f);
		}
	}

	shader = new Shader("worldBox.v.glsl", "worldBox.f.glsl");

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetAttribLocation(shader->id(), "v_color");

	glGenVertexArrays(1, &worldBoxAttributeObject[0]); // Create our Vertex Array Object  
	glBindVertexArray(worldBoxAttributeObject[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &worldBoxBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, worldBoxBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* vertexNum, &newDataVertex[0], GL_STATIC_DRAW);
	
	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	glGenBuffers(1, &worldBoxBufferObject[1]);
	glBindBuffer(GL_ARRAY_BUFFER, worldBoxBufferObject[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* vertexNum, &newDataColor[0], GL_STATIC_DRAW);

	glVertexAttribPointer(colorLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
	glEnableVertexAttribArray(colorLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);
}

void WorldBox::render(){
	if (visibility){
		shader->bind();
		glBindVertexArray(worldBoxAttributeObject[0]);
		glDrawArrays(GL_LINES, 0, vertexNum);
		glBindVertexArray(0);
		shader->unbind();
	}
}

void WorldBox::bindShader(){
	shader->bind();
}

void WorldBox::unbindShader(){
	shader->unbind();
}

Shader* WorldBox::getShader(){
	return shader;
}

WorldBox::~WorldBox(){

}

void WorldBox::toggleVisibility(){
	visibility = !visibility;
}