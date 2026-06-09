#include "stdafx.h"
#include "tunnel.h"
#include "simParam.h"
#include "vectorTypes.h"

Tunnel::Tunnel(bool visible, float cellSizeX, float cellSizeY, float cellSizeZ, int posX, int posY, int posZ, int w){
	vertexNum = 88;	// 20 Front + 20 Back + 2 * 16 tunnels + 2 * 4 side + 2 * 4 top/bottom
	vertexLineNum = 48; //8 Front 8 Back 8 Middle + everything times 2

	visibility = visible;

	std::vector<Vec4> newDataVertex(vertexNum);

	colorTunnel[0] = 0.6f;
	colorTunnel[1] = 0.6f;
	colorTunnel[2] = 0.6f;
	colorTunnel[3] = 1.0f;

	colorLines[0] = 0.0f;
	colorLines[1] = 0.0f;
	colorLines[2] = 0.0f;
	colorLines[3] = 1.0f;

	xSize = cellSizeX;
	ySize = cellSizeY;
	zSize = cellSizeZ;

	width = w;
	lHeight = cellSizeY;

	xMin = posX * cellSizeX;
	xMax = posX * cellSizeX + 9 * cellSizeX;

	yMin = posY * cellSizeY;
	yMax = posY * cellSizeY + 9 * cellSizeY;

	zMin = posZ * cellSizeZ;
	zMax = posZ * cellSizeZ + width * cellSizeZ;

	xTunnel1 = xMin + 3 * cellSizeX;
	xTunnel2 = xMin + 4 * cellSizeX;
	xTunnel3 = xMin + 5 * cellSizeX;
	xTunnel4 = xMin + 6 * cellSizeX;

	yTunnel1 = yMin + 4 * cellSizeY;
	yTunnel2 = yMin + 5 * cellSizeY;

	//front
	newDataVertex[0] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[1] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[2] = Vec4(xMax, yTunnel1, zMin, 1.0f);
	newDataVertex[3] = Vec4(xMin, yTunnel1, zMin, 1.0f);

	newDataVertex[4] = Vec4(xMin, yTunnel2, zMin, 1.0f);
	newDataVertex[5] = Vec4(xMax, yTunnel2, zMin, 1.0f);
	newDataVertex[6] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[7] = Vec4(xMin, yMax, zMin, 1.0f);

	newDataVertex[8] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[9] = Vec4(xTunnel1, yMin, zMin, 1.0f);
	newDataVertex[10] = Vec4(xTunnel1, yMax, zMin, 1.0f);
	newDataVertex[11] = Vec4(xMin, yMax, zMin, 1.0f);

	newDataVertex[12] = Vec4(xTunnel2, yMin, zMin, 1.0f);
	newDataVertex[13] = Vec4(xTunnel3, yMin, zMin, 1.0f);
	newDataVertex[14] = Vec4(xTunnel3, yMax, zMin, 1.0f);
	newDataVertex[15] = Vec4(xTunnel2, yMax, zMin, 1.0f);

	newDataVertex[16] = Vec4(xTunnel4, yMin, zMin, 1.0f);
	newDataVertex[17] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[18] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[19] = Vec4(xTunnel4, yMax, zMin, 1.0f);

	//back
	newDataVertex[20] = Vec4(xMin, yMin, zMax, 1.0f);
	newDataVertex[21] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[22] = Vec4(xMax, yTunnel1, zMax, 1.0f);
	newDataVertex[23] = Vec4(xMin, yTunnel1, zMax, 1.0f);

	newDataVertex[24] = Vec4(xMin, yTunnel2, zMax, 1.0f);
	newDataVertex[25] = Vec4(xMax, yTunnel2, zMax, 1.0f);
	newDataVertex[26] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[27] = Vec4(xMin, yMax, zMax, 1.0f);

	newDataVertex[28] = Vec4(xMin, yMin, zMax, 1.0f);
	newDataVertex[29] = Vec4(xTunnel1, yMin, zMax, 1.0f);
	newDataVertex[30] = Vec4(xTunnel1, yMax, zMax, 1.0f);
	newDataVertex[31] = Vec4(xMin, yMax, zMax, 1.0f);

	newDataVertex[32] = Vec4(xTunnel2, yMin, zMax, 1.0f);
	newDataVertex[33] = Vec4(xTunnel3, yMin, zMax, 1.0f);
	newDataVertex[34] = Vec4(xTunnel3, yMax, zMax, 1.0f);
	newDataVertex[35] = Vec4(xTunnel2, yMax, zMax, 1.0f);

	newDataVertex[36] = Vec4(xTunnel4, yMin, zMax, 1.0f);
	newDataVertex[37] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[38] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[39] = Vec4(xTunnel4, yMax, zMax, 1.0f);

	//side1
	newDataVertex[40] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[41] = Vec4(xMin, yMax, zMin, 1.0f);
	newDataVertex[42] = Vec4(xMin, yMax, zMax, 1.0f);
	newDataVertex[43] = Vec4(xMin, yMin, zMax, 1.0f);

	//side2
	newDataVertex[44] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[45] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[46] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[47] = Vec4(xMax, yMin, zMax, 1.0f);

	//bottom
	newDataVertex[48] = Vec4(xMin, yMin, zMin, 1.0f);
	newDataVertex[49] = Vec4(xMax, yMin, zMin, 1.0f);
	newDataVertex[50] = Vec4(xMax, yMin, zMax, 1.0f);
	newDataVertex[51] = Vec4(xMin, yMin, zMax, 1.0f);

	//top
	newDataVertex[52] = Vec4(xMin, yMax, zMin, 1.0f);
	newDataVertex[53] = Vec4(xMax, yMax, zMin, 1.0f);
	newDataVertex[54] = Vec4(xMax, yMax, zMax, 1.0f);
	newDataVertex[55] = Vec4(xMin, yMax, zMax, 1.0f);

	//tunnel1
	newDataVertex[56] = Vec4(xTunnel1, yTunnel1, zMin, 1.0f);
	newDataVertex[57] = Vec4(xTunnel2, yTunnel1, zMin, 1.0f);
	newDataVertex[58] = Vec4(xTunnel2, yTunnel1, zMax, 1.0f);
	newDataVertex[59] = Vec4(xTunnel1, yTunnel1, zMax, 1.0f);

	newDataVertex[60] = Vec4(xTunnel1, yTunnel2, zMin, 1.0f);
	newDataVertex[61] = Vec4(xTunnel2, yTunnel2, zMin, 1.0f);
	newDataVertex[62] = Vec4(xTunnel2, yTunnel2, zMax, 1.0f);
	newDataVertex[63] = Vec4(xTunnel1, yTunnel2, zMax, 1.0f);

	newDataVertex[64] = Vec4(xTunnel1, yTunnel1, zMin, 1.0f);
	newDataVertex[65] = Vec4(xTunnel1, yTunnel2, zMin, 1.0f);
	newDataVertex[66] = Vec4(xTunnel1, yTunnel2, zMax, 1.0f);
	newDataVertex[67] = Vec4(xTunnel1, yTunnel1, zMax, 1.0f);

	newDataVertex[68] = Vec4(xTunnel2, yTunnel1, zMin, 1.0f);
	newDataVertex[69] = Vec4(xTunnel2, yTunnel2, zMin, 1.0f);
	newDataVertex[70] = Vec4(xTunnel2, yTunnel2, zMax, 1.0f);
	newDataVertex[71] = Vec4(xTunnel2, yTunnel1, zMax, 1.0f);

	//tunnel2
	newDataVertex[72] = Vec4(xTunnel3, yTunnel1, zMin, 1.0f);
	newDataVertex[73] = Vec4(xTunnel4, yTunnel1, zMin, 1.0f);
	newDataVertex[74] = Vec4(xTunnel4, yTunnel1, zMax, 1.0f);
	newDataVertex[75] = Vec4(xTunnel3, yTunnel1, zMax, 1.0f);

	newDataVertex[76] = Vec4(xTunnel3, yTunnel2, zMin, 1.0f);
	newDataVertex[77] = Vec4(xTunnel4, yTunnel2, zMin, 1.0f);
	newDataVertex[78] = Vec4(xTunnel4, yTunnel2, zMax, 1.0f);
	newDataVertex[79] = Vec4(xTunnel3, yTunnel2, zMax, 1.0f);

	newDataVertex[80] = Vec4(xTunnel3, yTunnel1, zMin, 1.0f);
	newDataVertex[81] = Vec4(xTunnel3, yTunnel2, zMin, 1.0f);
	newDataVertex[82] = Vec4(xTunnel3, yTunnel2, zMax, 1.0f);
	newDataVertex[83] = Vec4(xTunnel3, yTunnel1, zMax, 1.0f);

	newDataVertex[84] = Vec4(xTunnel4, yTunnel1, zMin, 1.0f);
	newDataVertex[85] = Vec4(xTunnel4, yTunnel2, zMin, 1.0f);
	newDataVertex[86] = Vec4(xTunnel4, yTunnel2, zMax, 1.0f);
	newDataVertex[87] = Vec4(xTunnel4, yTunnel1, zMax, 1.0f);

	//lines
	std::vector<Vec4> newDataVertexLine(vertexLineNum);
	//front
	newDataVertexLine[0] = Vec4(xTunnel1, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[1] = Vec4(xTunnel2, yTunnel1, zMin - 0.1f, 1.0f);

	newDataVertexLine[2] = Vec4(xTunnel1, yTunnel2, zMin - 0.1f, 1.0f);
	newDataVertexLine[3] = Vec4(xTunnel2, yTunnel2, zMin - 0.1f, 1.0f);

	newDataVertexLine[4] = Vec4(xTunnel1, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[5] = Vec4(xTunnel1, yTunnel2, zMin - 0.1f, 1.0f);

	newDataVertexLine[6] = Vec4(xTunnel2, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[7] = Vec4(xTunnel2, yTunnel2, zMin - 0.1f, 1.0f);

	newDataVertexLine[8] = Vec4(xTunnel3, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[9] = Vec4(xTunnel4, yTunnel1, zMin - 0.1f, 1.0f);

	newDataVertexLine[10] = Vec4(xTunnel3, yTunnel2, zMin - 0.1f, 1.0f);
	newDataVertexLine[11] = Vec4(xTunnel4, yTunnel2, zMin - 0.1f, 1.0f);

	newDataVertexLine[12] = Vec4(xTunnel3, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[13] = Vec4(xTunnel3, yTunnel2, zMin - 0.1f, 1.0f);

	newDataVertexLine[14] = Vec4(xTunnel4, yTunnel1, zMin - 0.1f, 1.0f);
	newDataVertexLine[15] = Vec4(xTunnel4, yTunnel2, zMin - 0.1f, 1.0f);

	//back
	newDataVertexLine[16] = Vec4(xTunnel1, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[17] = Vec4(xTunnel2, yTunnel1, zMax + 0.1f, 1.0f);

	newDataVertexLine[18] = Vec4(xTunnel1, yTunnel2, zMax + 0.1f, 1.0f);
	newDataVertexLine[19] = Vec4(xTunnel2, yTunnel2, zMax + 0.1f, 1.0f);

	newDataVertexLine[20] = Vec4(xTunnel1, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[21] = Vec4(xTunnel1, yTunnel2, zMax + 0.1f, 1.0f);

	newDataVertexLine[22] = Vec4(xTunnel2, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[23] = Vec4(xTunnel2, yTunnel2, zMax + 0.1f, 1.0f);

	newDataVertexLine[24] = Vec4(xTunnel3, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[25] = Vec4(xTunnel4, yTunnel1, zMax + 0.1f, 1.0f);

	newDataVertexLine[26] = Vec4(xTunnel3, yTunnel2, zMax + 0.1f, 1.0f);
	newDataVertexLine[27] = Vec4(xTunnel4, yTunnel2, zMax + 0.1f, 1.0f);

	newDataVertexLine[28] = Vec4(xTunnel3, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[29] = Vec4(xTunnel3, yTunnel2, zMax + 0.1f, 1.0f);

	newDataVertexLine[30] = Vec4(xTunnel4, yTunnel1, zMax + 0.1f, 1.0f);
	newDataVertexLine[31] = Vec4(xTunnel4, yTunnel2, zMax + 0.1f, 1.0f);

	//middle
	newDataVertexLine[32] = Vec4(xTunnel1 + 0.1f, yTunnel1 + 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[33] = Vec4(xTunnel1 + 0.1f, yTunnel1 + 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[34] = Vec4(xTunnel1 + 0.1f, yTunnel2 - 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[35] = Vec4(xTunnel1 + 0.1f, yTunnel2 - 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[36] = Vec4(xTunnel2 - 0.1f, yTunnel1 + 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[37] = Vec4(xTunnel2 - 0.1f, yTunnel1 + 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[38] = Vec4(xTunnel2 - 0.1f, yTunnel2 - 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[39] = Vec4(xTunnel2 - 0.1f, yTunnel2 - 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[40] = Vec4(xTunnel3 + 0.1f, yTunnel1 + 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[41] = Vec4(xTunnel3 + 0.1f, yTunnel1 + 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[42] = Vec4(xTunnel3 + 0.1f, yTunnel2 - 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[43] = Vec4(xTunnel3 + 0.1f, yTunnel2 - 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[44] = Vec4(xTunnel4 - 0.1f, yTunnel1 + 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[45] = Vec4(xTunnel4 - 0.1f, yTunnel1 + 0.1f, zMax + 0.1f, 1.0f);

	newDataVertexLine[46] = Vec4(xTunnel4 - 0.1f, yTunnel2 - 0.1f, zMin - 0.1f, 1.0f);
	newDataVertexLine[47] = Vec4(xTunnel4 - 0.1f, yTunnel2 - 0.1f, zMax + 0.1f, 1.0f);

	shader = new Shader("column.v.glsl", "column.f.glsl");

	GLint vertLoc = glGetAttribLocation(shader->id(), "coord3d");

	glGenVertexArrays(1, &tunnelAttributeObject[0]); // Create our Vertex Array Object  
	glBindVertexArray(tunnelAttributeObject[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &tunnelBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, tunnelBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* vertexNum, &newDataVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	glGenVertexArrays(1, &tunnelLineAttributeObject[0]); // Create our Vertex Array Object  
	glBindVertexArray(tunnelLineAttributeObject[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &tunnelLineBufferObject[0]);
	glBindBuffer(GL_ARRAY_BUFFER, tunnelLineBufferObject[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* vertexLineNum, &newDataVertexLine[0], GL_STATIC_DRAW);

	glVertexAttribPointer(vertLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(vertLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

}

void Tunnel::render(){
	if (visibility){
		shader->bind();
		GLint colorLoc = glGetUniformLocation(shader->id(), "color");
		glUniform4fv(colorLoc, 1, colorTunnel);

		glBindVertexArray(tunnelAttributeObject[0]);
		glDrawArrays(GL_QUADS, 0, vertexNum);
		//glDrawArrays(GL_LINES, 0, 2 * vertexNum);
		glBindVertexArray(0);

		glUniform4fv(colorLoc, 1, colorLines);
		
		glBindVertexArray(tunnelLineAttributeObject[0]);
		glDrawArrays(GL_LINES, 0, vertexLineNum);
		glBindVertexArray(0);

		shader->unbind();
	}
}

void Tunnel::bindShader(){
	shader->bind();
}

void Tunnel::unbindShader(){
	shader->unbind();
}

Shader* Tunnel::getShader(){
	return shader;
}

Tunnel::~Tunnel(){

}

void Tunnel::setVisibility(bool visible){
	visibility = visible;
}


void Tunnel::getObstacleForce(std::vector<Vec4>* cor, std::vector<unsigned int>* start, std::vector<unsigned int>* end, std::vector<Vec4>* posObstacle, unsigned int offset){
	float x = 5.f;
	float y = 5.f;
	float z = 5.f;
	unsigned int iP;
	unsigned int ip2;

	for (int i = 0; i < width; i++){
		x = 2.f;
		y = 2.f;
		iP = i * 10;
		ip2 = i * 18;
		(*start)[iP] = ip2;
		(*end)[iP] = ip2 + 1;
		(*posObstacle)[iP] = Vec4((xTunnel1 + xTunnel2) / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2] = Vec4(-x, 0.0f, z, 0.0f);

		(*start)[iP + 1] = (*end)[iP];
		(*end)[iP + 1] = (*end)[iP]  + 2;
		(*posObstacle)[iP + 1] = Vec4((xTunnel2 + xTunnel3) / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 1] = Vec4(-x, 0.0f, 0.0f, 0.0f);
		(*cor)[ip2 + 2] = Vec4(-x, 0.0f, z, 0.0f);

		(*start)[iP + 2] = (*end)[iP + 1];
		(*end)[iP + 2] = (*end)[iP + 1] + 2;
		(*posObstacle)[iP + 2] = Vec4((xTunnel1 + xTunnel2) / 2, yTunnel2 + ySize / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 3] = Vec4(0.0f, -y, 0.0f, 0.0f);
		(*cor)[ip2 + 4] = Vec4(0.0f, -y, z, 0.0f);

		(*start)[iP + 3] = (*end)[iP + 2];
		(*end)[iP + 3] = (*end)[iP + 2] + 2;
		(*posObstacle)[iP + 3] = Vec4((xTunnel1 + xTunnel2) / 2, yTunnel1 - ySize / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 5] = Vec4(0.0f, y, 0.0f, 0.0f);
		(*cor)[ip2 + 6] = Vec4(0.0f, y, z, 0.0f);

		(*start)[iP + 4] = (*end)[iP + 3];
		(*end)[iP + 4] = (*end)[iP + 3] + 2;
		(*posObstacle)[iP + 4] = Vec4(xTunnel1 - xSize / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 7] = Vec4(x, 0.0, 0.0f, 0.0f);
		(*cor)[ip2 + 8] = Vec4(x, 0.0,  z, 0.0f);

		(*start)[iP + 5] = (*end)[iP + 4];
		(*end)[iP + 5] = (*end)[iP + 4] + 1;
		(*posObstacle)[iP + 5] = Vec4((xTunnel3 + xTunnel4) / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 9] = Vec4(0.0f, 0.0f, -z, 0.0f);

		(*start)[iP + 6] = (*end)[iP + 5];
		(*end)[iP + 6] = (*end)[iP + 5] + 2;
		(*posObstacle)[iP + 6] = Vec4((xTunnel2 + xTunnel3) / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 10] = Vec4(x, 0.0f, 0.0f, 0.0f);
		(*cor)[ip2 + 11] = Vec4(x, 0.0f, -z, 0.0f);

		(*start)[iP + 7] = (*end)[iP + 6];
		(*end)[iP + 7] = (*end)[iP + 6] + 2;
		(*posObstacle)[iP + 7] = Vec4((xTunnel3 + xTunnel4) / 2, yTunnel2 + ySize / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 12] = Vec4(0.0f, -y, 0.0f, 0.0f);
		(*cor)[ip2 + 13] = Vec4(0.0f, -y, -z, 0.0f);

		(*start)[iP + 8] = (*end)[iP + 7];
		(*end)[iP + 8] = (*end)[iP + 7] + 2;
		(*posObstacle)[iP + 8] = Vec4((xTunnel3 + xTunnel4) / 2, yTunnel1 - ySize / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 14] = Vec4(0.0f, y, 0.0f, 0.0f);
		(*cor)[ip2 + 15] = Vec4(0.0f, y, -z, 0.0f);

		(*start)[iP + 9] = (*end)[iP + 8];
		(*end)[iP + 9] = (*end)[iP + 8] + 2;
		(*posObstacle)[iP + 9] = Vec4(xTunnel4 + xSize / 2, (yTunnel1 + yTunnel2) / 2, zMin + i * zSize + zSize / 2, 0.0f);
		(*cor)[ip2 + 16] = Vec4(-x, 0.0, 0.0f, 0.0f);
		(*cor)[ip2 + 17] = Vec4(-x, 0.0, -z, 0.0f);
	}

	x = 5.f;
	y = 5.f;
	z = z / 5;
	x = x * 8;
	y = y * 7;
	iP = width * 10;
	ip2 = width * 18;
	for (int i = 0; i < 9; i++){
		for (int j = 0; j < 9; j++){
			if (i == 3 && j == 4){

			}
			else if (i == 5 && j == 4){

			}
			else {
				(*start)[iP] = (*end)[iP - 1];
				(*end)[iP] = (*end)[iP - 1] + 2;
				(*posObstacle)[iP] = Vec4(xMin + xSize * i + xSize / 2, yMin + j * ySize + ySize / 2, zMin + zSize / 2, 0.0f);
				(*cor)[ip2] = Vec4(0.0f, 0.0, -z, 0.0f);

				(*start)[iP + 1] = (*end)[iP];
				(*end)[iP + 1] = (*end)[iP] + 2;
				(*posObstacle)[iP + 1] = Vec4(xMin + xSize * i + xSize / 2, yMin + j * ySize + ySize / 2, zMax - zSize / 2, 0.0f);
				(*cor)[ip2 + 2] = Vec4(0.0f, 0.0, z, 0.0f);


				if (j < 4){
					(*cor)[ip2 + 1] = Vec4(0.0f, y, -z, 0.0f);
					(*cor)[ip2 + 3] = Vec4(0.0f, y, z, 0.0f);
				}
				else if (j > 4){
					(*cor)[ip2 + 1] = Vec4(0.0f, -y, -z, 0.0f);
					(*cor)[ip2 + 3] = Vec4(0.0f, -y, z, 0.0f);
				}
				else {
					if (i > 5){
						(*cor)[ip2 + 1] = Vec4(-x, 0.0f, -z, 0.0f);
						(*cor)[ip2 + 3] = Vec4(-x, 0.0f, z, 0.0f);
					}
					 else if (i < 3){
						 (*cor)[ip2 + 1] = Vec4(x, 0.0f, -z, 0.0f);
						 (*cor)[ip2 + 3] = Vec4(x, 0.0f, z, 0.0f);
					 }
					 else {
						 (*cor)[ip2 + 1] = Vec4(-x, 0.0f, -z, 0.0f);
						 (*cor)[ip2 + 3] = Vec4(x, 0.0f, z, 0.0f);
					 }
				}

				iP += 2;
				ip2 += 4;
			}
		}
	}
}
