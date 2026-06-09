#include "stdafx.h"
#include "column.h"
#include "simParam.h"
#include "vectorTypes.h"

Column::Column(bool visible, float cellSizeX, float cellSizeY, float cellSizeZ, int posX, int posY, int posZ, int hH){
	vertexNum = 6 * 4;

	visibility = visible;

	std::vector<Vec4> newDataVertex(vertexNum);
	GLfloat colorColumn[4] = {0.6f, 0.6f, 0.6f, 1.0f};

	h = hH;
	lHeight = cellSizeY;

	xMin = posX * cellSizeX;
	xMax = posX * cellSizeX + cellSizeX;

	yMin = posY * cellSizeY;
	yMax = posY * cellSizeY + h * cellSizeY;

	zMin = posZ * cellSizeZ;
	zMax = posZ * cellSizeZ + cellSizeZ;

	newDataVertex[0] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[1] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[2] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[3] = Vec4(xMin, yMax, zMin, 1.0f);

	newDataVertex[4] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[5] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[6] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[7] = Vec4(xMax, yMax, zMin, 1.0f);

	newDataVertex[8] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[9] = Vec4(xMin, yMin, zMax, 1.0f);
	newDataVertex[10] = Vec4(xMin, yMax, zMax, 1.0f);
	newDataVertex[11] = Vec4(xMax, yMax, zMax, 1.0f);

	newDataVertex[12] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[13] = Vec4(xMin, yMin, zMax, 1.0f);
	newDataVertex[14] = Vec4(xMin, yMax, zMax, 1.0f);
	newDataVertex[15] = Vec4(xMin, yMax, zMin, 1.0f);

	newDataVertex[16] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[17] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[18] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[19] = Vec4(xMin, yMin, zMax, 1.0f);

	newDataVertex[20] = Vec4(xMin, yMax, zMin, 1.0f);
	newDataVertex[21] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[22] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[23] = Vec4(xMin, yMax, zMax, 1.0f);

	shader = new Shader("column.v.glsl", "column.f.glsl");

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");
	GLint colorLoc = glGetUniformLocation(shader->id(), "color");

	glGenVertexArrays(1, &columnAttributeObject[0]); // Create our Vertex Array Object  
	glBindVertexArray(columnAttributeObject[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &columnBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, columnBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* vertexNum, &newDataVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	shader->bind();
	glUniform4fv(colorLoc, 1, colorColumn);
	shader->unbind();
}

void Column::render(){
	if (visibility){
		shader->bind();
		glBindVertexArray(columnAttributeObject[0]);
		glDrawArrays(GL_QUADS, 0, vertexNum);
		glBindVertexArray(0);
		shader->unbind();
	}
}

void Column::bindShader(){
	shader->bind();
}

void Column::unbindShader(){
	shader->unbind();
}

Shader* Column::getShader(){
	return shader;
}

Column::~Column(){

}

void Column::setVisibility(bool visible){
	visibility = visible;
}

/*
void Column::getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle){
	float x = 8.f;
	float y = 5.f;
	float z = 6.f;

	(*start)[0] = 0;
	(*end)[0] = 5;
	(*posObstacle)[0] = Vec4((xMin + xMax) / 2, yMin + lHeight / 2, (zMin + zMax) / 2, 0.0f);
	(*cor)[0] = Vec4(0.0f, -y, 0.0f, 0.0f);
	(*cor)[1] = Vec4(-x, -y, 0.0f, 0.0f);
	(*cor)[2] = Vec4(x, -y, 0.0f, 0.0f);
	(*cor)[3] = Vec4(0.0f, -y, z, 0.0f);
	(*cor)[4] = Vec4(0.0f, -y, -z, 0.0f);

	for (int i = 0; i < 2 * h; i+=2){
		unsigned int iP = i * 4 + 5;

		(*start)[i + 1 ] = (*end)[i];
		(*end)[i + 1 ] = (*end)[i] + 4;
		(*posObstacle)[i + 1] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP] = Vec4(x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 1] = Vec4(x, 0.0f, z, 0.0f);
		(*cor)[iP + 2] = Vec4(x, 0.0f, -z, 0.0f);
		(*cor)[iP + 3] = Vec4(0.0, 0.0f, z, 0.0f);

		(*start)[i + 2] = (*end)[i + 1];
		(*end)[i + 2] = (*end)[i + 1] + 4;
		(*posObstacle)[i + 2] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP + 4] = Vec4(-x, 0.0f, z, 0.0f);
		(*cor)[iP + 5] = Vec4(-x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 6] = Vec4(-x, 0.0f, -z, 0.0f);
		(*cor)[iP + 7] = Vec4(0.0f, 0.0f, -z, 0.0f);
	}

	(*start)[2 * h + 1] = (*end)[2 * h];
	(*end)[2 * h + 1] = (*end)[2 * h] + 5;
	(*posObstacle)[2 * h + 1] = Vec4((xMin + xMax) / 2, yMin + lHeight * (h - 1) + lHeight / 2, (zMin + zMax) / 2, 0.0f);

	unsigned int iP = (2 * h) * 4 + 5;
	(*cor)[iP] = Vec4(0.0f, y, 0.0f, 0.0f);
	(*cor)[iP + 1] = Vec4(-x, y, 0.0f, 0.0f);
	(*cor)[iP + 2] = Vec4(x, y, 0.0f, 0.0f);
	(*cor)[iP + 3] = Vec4(0.0f, y, z, 0.0f);
	(*cor)[iP + 4] = Vec4(0.0f, y, -z, 0.0f);
}*/

void Column::getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle, unsigned int offset){
	float x = 5.f;
	float y = 5.f;
	float z = 5.f;
	unsigned int iP;
	if (offset == 0){
		(*start)[0] = 0;
		(*end)[0] = 5;
		iP = 0;
	}
	else
	{
		(*start)[offset] = (*end)[offset - 1];
		(*end)[offset] = (*end)[offset - 1] + 5;
		iP = (*end)[offset -1];
	}
	(*posObstacle)[offset] = Vec4((xMin + xMax) / 2, yMin + lHeight / 2, (zMin + zMax) / 2, 0.0f);
	(*cor)[iP] = Vec4(0.0f, -y, 0.0f, 0.0f);
	(*cor)[iP + 1] = Vec4(-x, -y, 0.0f, 0.0f);
	(*cor)[iP + 2] = Vec4(x, -y, 0.0f, 0.0f);
	(*cor)[iP + 3] = Vec4(0.0f, -y, z, 0.0f);
	(*cor)[iP + 4] = Vec4(0.0f, -y, -z, 0.0f);

	for (int i = 0; i < h; i += 1){
		iP = (*end)[offset + 4 * i];

		(*start)[offset + 4 * i + 1] = (*end)[offset + 4 * i];
		(*end)[offset + 4 * i + 1] = (*end)[offset + 4 * i] + 3;
		(*posObstacle)[offset + 4 * i + 1] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP] = Vec4(x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 1] = Vec4(x, 0.0f, z, 0.0f);
		(*cor)[iP + 2] = Vec4(0.0, 0.0f, z, 0.0f);

		(*start)[offset + 4 * i + 2] = (*end)[offset + 4 * i + 1];
		(*end)[offset + 4 * i + 2] = (*end)[offset + 4 * i + 1] + 3;
		(*posObstacle)[offset + 4 * i + 2] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP + 3] = Vec4(x, 0.0f, -z, 0.0f);
		(*cor)[iP + 4] = Vec4(x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 5] = Vec4(0.0f, 0.0f, -z, 0.0f);

		(*start)[offset + 4 * i + 3] = (*end)[offset + 4 * i + 2];
		(*end)[offset + 4 * i + 3] = (*end)[offset + 4 * i + 2] + 3;
		(*posObstacle)[offset + 4 * i + 3] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP + 6] = Vec4(-x, 0.0f, -z, 0.0f);
		(*cor)[iP + 7] = Vec4(0.0, 0.0f, -z, 0.0f);
		(*cor)[iP + 8] = Vec4(-x, 0.0f, 0.0f, 0.0f);

		(*start)[offset + 4 * i + 4] = (*end)[offset + 4 * i + 3];
		(*end)[offset + 4 * i + 4] = (*end)[offset + 4 * i + 3] + 3;
		(*posObstacle)[offset + 4 * i + 4] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);

		(*cor)[iP + 9] = Vec4(-x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 10] = Vec4(-x, 0.0f, z, 0.0f);
		(*cor)[iP + 11] = Vec4(0.0f, 0.0f, z, 0.0f);
	}

	(*start)[offset + 4 * h + 1] = (*end)[offset + 4 * h];
	(*end)[offset + 4 * h + 1] = (*end)[offset + 4 * h] + 5;
	(*posObstacle)[offset + 4 * h + 1] = Vec4((xMin + xMax) / 2, yMin + lHeight * (h - 1) + lHeight / 2, (zMin + zMax) / 2, 0.0f);

	iP = (*end)[offset + 4 * h];
	(*cor)[iP] = Vec4(0.0f, y, 0.0f, 0.0f);
	(*cor)[iP + 1] = Vec4(-x, y, 0.0f, 0.0f);
	(*cor)[iP + 2] = Vec4(x, y, 0.0f, 0.0f);
	(*cor)[iP + 3] = Vec4(0.0f, y, z, 0.0f);
	(*cor)[iP + 4] = Vec4(0.0f, y, -z, 0.0f);
}

/*
void Column::getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle){
	float x = 10.f;
	float y = 10.f;
	float z = 10.f;

	(*start)[0] = 0;
	(*end)[0] = 13;
	(*posObstacle)[0] = Vec4((xMin + xMax) / 2, yMin + lHeight / 2, (zMin + zMax) / 2, 0.0f);
	(*cor)[0] = Vec4(x, 0.0f, 0.0f, 0.0f);
	(*cor)[1] = Vec4(x, 0.0f, z, 0.0f);
	(*cor)[2] = Vec4(0.0, 0.0f, z, 0.0f);
	(*cor)[3] = Vec4(-x, 0.0f, z, 0.0f);
	(*cor)[4] = Vec4(-x, 0.0f, 0.0f, 0.0f);
	(*cor)[5] = Vec4(-x, 0.0f, -z, 0.0f);
	(*cor)[6] = Vec4(0.0f, 0.0f, -z, 0.0f);
	(*cor)[7] = Vec4(x, 0.0f, -z, 0.0f);
	(*cor)[8] = Vec4(0.0f, -y, 0.0f, 0.0f);
	(*cor)[9] = Vec4(-x, -y, 0.0f, 0.0f);
	(*cor)[10] = Vec4(x, -y, 0.0f, 0.0f);
	(*cor)[11] = Vec4(0.0f, -y, z, 0.0f);
	(*cor)[12] = Vec4(0.0f, -y, -z, 0.0f);

	for (int i = 1; i < (h - 1); i++){
		(*start)[i] = (*end)[i - 1];
		(*end)[i] = (*end)[i - 1] + 8;
		(*posObstacle)[i] = Vec4((xMin + xMax) / 2, yMin + lHeight * i + lHeight / 2, (zMin + zMax) / 2, 0.0f);
		unsigned int iP = i * 8 + 5;
		(*cor)[iP] = Vec4(x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 1] = Vec4(x, 0.0f, z, 0.0f);
		(*cor)[iP + 2] = Vec4(0.0, 0.0f, z, 0.0f);
		(*cor)[iP + 3] = Vec4(-x, 0.0f, z, 0.0f);
		(*cor)[iP + 4] = Vec4(-x, 0.0f, 0.0f, 0.0f);
		(*cor)[iP + 5] = Vec4(-x, 0.0f, -z, 0.0f);
		(*cor)[iP + 6] = Vec4(0.0f, 0.0f, -z, 0.0f);
		(*cor)[iP + 7] = Vec4(x, 0.0f, -z, 0.0f);
	}

	(*start)[h - 1] = (*end)[h - 2];
	(*end)[h - 1] = (*end)[h - 2] + 13;
	(*posObstacle)[h - 1] = Vec4((xMin + xMax) / 2, yMin + lHeight * (h - 1) + lHeight / 2, (zMin + zMax) / 2, 0.0f);

	unsigned int iP = (h - 1) * 8 + 5;
	(*cor)[iP] = Vec4(x, 0.0f, 0.0f, 0.0f);
	(*cor)[iP + 1] = Vec4(x, 0.0f, z, 0.0f);
	(*cor)[iP + 2] = Vec4(0.0, 0.0f, z, 0.0f);
	(*cor)[iP + 3] = Vec4(-x, 0.0f, z, 0.0f);
	(*cor)[iP + 4] = Vec4(-x, 0.0f, 0.0f, 0.0f);
	(*cor)[iP + 5] = Vec4(-x, 0.0f, -z, 0.0f);
	(*cor)[iP + 6] = Vec4(0.0f, 0.0f, -z, 0.0f);
	(*cor)[iP + 7] = Vec4(x, 0.0f, -z, 0.0f);
	(*cor)[iP + 8] = Vec4(0.0f, y, 0.0f, 0.0f);
	(*cor)[iP + 9] = Vec4(-x, y, 0.0f, 0.0f);
	(*cor)[iP + 10] = Vec4(x, y, 0.0f, 0.0f);
	(*cor)[iP + 11] = Vec4(0.0f, y, z, 0.0f);
	(*cor)[iP + 12] = Vec4(0.0f, y, -z, 0.0f);
	}*/