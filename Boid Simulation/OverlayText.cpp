#include "stdafx.h"
#include "overlayText.h"
#include "gfx.h"

OverlayText::OverlayText(){
	shaderText = new Shader("text.v.glsl", "text.f.glsl");
	shaderBox = new Shader("box.v.glsl", "box.f.glsl");

	loadFont();
	createText();

	glGenBuffers(1, &vbo);

	colorTextLoc = glGetUniformLocation(shaderText->id(), "color");
	texTextLoc = glGetUniformLocation(shaderText->id(), "tex");
	coordTextLoc = glGetAttribLocation(shaderText->id(), "coord");

	colorBoxLoc = glGetUniformLocation(shaderBox->id(), "color");
	coordBoxLoc = glGetAttribLocation(shaderBox->id(), "coord");
	viewBoxLoc = glGetUniformLocation(shaderBox->id(), "m_transform");

	createBox();
}

OverlayText::~OverlayText(){

}

int OverlayText::loadFont(){
	
	if (FT_Init_FreeType(&ft)) {
		fprintf(stderr, "Could not init freetype library\n");
		return 0;
	}
	
	if (FT_New_Face(ft, fontFilename, 0, &face)) {
		fprintf(stderr, "Could not open font %s\n", fontFilename);
		return 0;
	}

	return 1;
}

bool OverlayText::createBox(){
	boxColor[0] = 0.6f;
	boxColor[1] = 0.6f;
	boxColor[2] = 0.6f;
	boxColor[3] = 0.8f;

	float windowWidth = (float)GFX::getInstance().getWindowWidth();
	float windowHeight = (float)GFX::getInstance().getWindowHeight();
	float sx = 2.0 / windowWidth;
	float sy = 2.0 / windowHeight;

	int size = textExtended.size();
	float top = 1 - windowHeight * sy / 4;
	float bottom = 1 - (windowHeight / 4 + 24.5 * size) * sy;
	float left = -1 + windowWidth  * sx / 4;
	float right = -1 + windowWidth * sx * 3 / 4;

	boxVertex = std::vector<Vec4>(4);
	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	//text box

	glGenVertexArrays(1, &boxVAO[0]); // Create our Vertex Array Object  
	glBindVertexArray(boxVAO[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &boxVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, boxVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(coordBoxLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(coordBoxLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	glm::mat4 projection = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);

	shaderBox->bind();
	glUniform4fv(colorBoxLoc, 1, boxColor);
	glUniformMatrix4fv(viewBoxLoc, 1, GL_FALSE, glm::value_ptr(projection));
	shaderBox->unbind();

	//bottom box for imidiate simulation Information

	top = 1 - (windowHeight - 48) * sy;
	bottom = 1 - windowHeight * sy;
	left = 1 - (windowWidth - 5) * sx;
	right = 1 - (windowWidth - 120) * sx;

	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	glGenVertexArrays(1, &boxBottomVAO[0]); // Create our Vertex Array Object  
	glBindVertexArray(boxBottomVAO[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &boxBottomVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, boxBottomVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(coordBoxLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(coordBoxLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	shaderBox->bind();
	glUniform4fv(colorBoxLoc, 1, boxColor);
	glUniformMatrix4fv(viewBoxLoc, 1, GL_FALSE, glm::value_ptr(projection));
	shaderBox->unbind();

	top = windowHeight * sy - 1;
	bottom = (windowHeight - 48) * sy - 1;
	left = 1 - (windowWidth - 5) * sx;
	right = 1 - (windowWidth - 200) * sx;

	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	glGenVertexArrays(1, &boxTopVAO[0]); // Create our Vertex Array Object  
	glBindVertexArray(boxTopVAO[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &boxTopVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, boxTopVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(coordBoxLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(coordBoxLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	shaderBox->bind();
	glUniform4fv(colorBoxLoc, 1, boxColor);
	glUniformMatrix4fv(viewBoxLoc, 1, GL_FALSE, glm::value_ptr(projection));
	shaderBox->unbind();

	top = 1 - windowHeight * sy / 4;
	bottom = 1 - (windowHeight / 4 + 24.5 * size) * sy;
	left = -1 + windowWidth  * sx / 6;
	right = -1 + windowWidth * sx * 5 / 6;

	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	glGenVertexArrays(1, &boxExtVAO[0]); // Create our Vertex Array Object  
	glBindVertexArray(boxExtVAO[0]); // Bind our Vertex Array Object so we can use it  

	glGenBuffers(1, &boxExtVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, boxExtVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);

	glVertexAttribPointer(coordBoxLoc, 4, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer
	glEnableVertexAttribArray(coordBoxLoc);

	glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
	glBindVertexArray(0);

	shaderBox->bind();
	glUniform4fv(colorBoxLoc, 1, boxColor);
	glUniformMatrix4fv(viewBoxLoc, 1, GL_FALSE, glm::value_ptr(projection));
	shaderBox->unbind();

	return true;
}

void OverlayText::changeBox(int size, float sy, float sx, int width){
	float windowWidth = (float)GFX::getInstance().getWindowWidth();
	float windowHeight = (float)GFX::getInstance().getWindowHeight();

	float top = 1 - windowHeight * sy / 4;
	float bottom = 1 - (windowHeight / 4 + 24.5 * size) * sy;
	float left = -1 + windowWidth  * sx / 4;
	float right = -1 + (windowWidth + width) * sx / 4;

	boxVertex = std::vector<Vec4>(4);
	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	shaderBox->bind();
	glBindBuffer(GL_ARRAY_BUFFER, boxVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	shaderBox->unbind();

	top = 1 - windowHeight * sy / 4;
	bottom = 1 - (windowHeight / 4 + 24.5 * size) * sy;
	left = -1 + windowWidth  * sx / 6;
	right = -1 + (windowWidth + width) * sx / 6;

	boxVertex = std::vector<Vec4>(4);
	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	shaderBox->bind();
	glBindBuffer(GL_ARRAY_BUFFER, boxExtVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	shaderBox->unbind();
}

void OverlayText::changeBoxTopBottom(float sy, float sx){
	float windowHeight = (float)GFX::getInstance().getWindowHeight();
	float windowWidth = (float)GFX::getInstance().getWindowWidth();

	float top = 1 - (windowHeight - 48) * sy;
	float bottom = 1 - windowHeight * sy;
	float left = 1 - (windowWidth - 5) * sx;
	float right = 1 - (windowWidth - 145) * sx;

	boxVertex = std::vector<Vec4>(4);
	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	shaderBox->bind();
	glBindBuffer(GL_ARRAY_BUFFER, boxBottomVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	shaderBox->unbind();

	top = windowHeight * sy - 1;
	bottom = (windowHeight - 48) * sy - 1;
	left = 1 - (windowWidth - 5) * sx;
	right = 1 - (windowWidth - 200) * sx;

	boxVertex[0] = Vec4(left, top, 0.01f, 1.0f);
	boxVertex[1] = Vec4(left, bottom, 0.01f, 1.0f);
	boxVertex[2] = Vec4(right, bottom, 0.01f, 1.0f);
	boxVertex[3] = Vec4(right, top, 0.01f, 1.0f);

	shaderBox->bind();
	glBindBuffer(GL_ARRAY_BUFFER, boxTopVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4)* 4, &boxVertex[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	shaderBox->unbind();
}

void OverlayText::drawBox(GLuint vao){
	shaderBox->bind();
	glBindVertexArray(vao);
	glDrawArrays(GL_QUADS, 0, 4);
	glBindVertexArray(0);
	shaderBox->unbind();
}



void OverlayText::createText(){
	textSimple = std::vector<const char*>(2);
	textSimple[0] = "F1 to toggle help";
	textSimple[1] = "F2 to toggle measurements";

	textExtended = std::vector<const char*>(13);
	textExtended[0] = "Switch Boid Model:       ";
	textExtended[1] = "      Boid Model Simple     [1]";
	textExtended[2] = "      Boid Model Grid          [2]";
	textExtended[3] = "      Boid Model SH            [3]";
	textExtended[4] = "      Boid Model Grid 2D    [4]";
	textExtended[5] = "      Boid Model SH 2D      [5]";
	textExtended[6] = "----------------------------------------------";
	textExtended[7] = "Restart Simulation               [R]";
	textExtended[8] = "Increase/Decrease Boids  [+/-]";
	textExtended[9] = "Change initial setup             [T]";
	textExtended[10] = "Follow random boid             [F]";
	textExtended[11] = "Switch camera                 [TAB]";
	textExtended[12] = "Reset camera                       [C]";

	textExtended2 = std::vector<const char*>(11);
	textExtended2[0] = "";
	textExtended2[1] = "Boid Model Way1            [6]";
	textExtended2[2] = "Boid Model Way2            [7]";
	textExtended2[3] = "Boid Model Obstacle       [8]";
	textExtended2[4] = "Boid Model Combined    [9]";
	textExtended2[5] = "Boid Model Tunnel          [0]";
	textExtended2[6] = "----------------------------------------";
	textExtended2[7] = "World ground visibility [G]";
	textExtended2[8] = "World box visibility       [V]";
	textExtended2[9] = "Sky box visibility           [S]";
	textExtended2[10] = "Quit                       [ESC/Q]";
}

void OverlayText::renderText(std::vector<const char*> textVector, float xBegin, float yBegin, float sx, float sy){
	const char *p;
	FT_GlyphSlot g = face->glyph;
	/* Create a texture that will be used to hold one "glyph" */
	GLuint tex;
	GLfloat black[4] = { .0f, .0f, .0f, .8f };

	shaderText->bind();

	FT_Set_Pixel_Sizes(face, 0, fontSize);
	glUniform4fv(colorTextLoc, 1, black);

	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glUniform1i(texTextLoc, 0);
	/* We require 1 byte alignment when uploading texture data */
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	/* Clamping to edges is important to prevent artifacts when scaling */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	/* Linear filtering usually looks best for text */
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	/* Set up the VBO for our vertex data */
	glEnableVertexAttribArray(coordTextLoc);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(coordTextLoc, 4, GL_FLOAT, GL_FALSE, 0, 0);

	for (int i = 0; i < textVector.size(); i++){
		float y = yBegin - i * fontSize * lineSpacing * sy;
		float x = xBegin;
		const char* text = textVector[i];
		/* Loop through all characters */
		for (p = text; *p; p++) {
			/* Try to load and render the character */
			if (FT_Load_Char(face, *p, FT_LOAD_RENDER))
				continue;
			/* Upload the "bitmap", which contains an 8-bit grayscale image, as an alpha texture */
			glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, g->bitmap.width, g->bitmap.rows, 0, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);
			/* Calculate the vertex and texture coordinates */
			float x2 = x + g->bitmap_left * sx;
			float y2 = -y - g->bitmap_top * sy;
			float w = g->bitmap.width * sx;
			float h = g->bitmap.rows * sy;
			point box[4] = {
				{ x2, -y2, 0, 0 },
				{ x2 + w, -y2, 1, 0 },
				{ x2, -y2 - h, 0, 1 },
				{ x2 + w, -y2 - h, 1, 1 },
			};
			/* Draw the character on the screen */
			glBufferData(GL_ARRAY_BUFFER, sizeof box, box, GL_DYNAMIC_DRAW);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			/* Advance the cursor to the start of the next character */
			x += (g->advance.x >> 6) * sx;
			y += (g->advance.y >> 6) * sy;
		}
	}

	glDisableVertexAttribArray(coordTextLoc);
	glDeleteTextures(1, &tex);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	shaderText->unbind();
}

void OverlayText::render(){
	float windowWidth = (float)GFX::getInstance().getWindowWidth();
	float windowHeight = (float)GFX::getInstance().getWindowHeight();
	float sx = 2.0f / windowWidth;
	float sy = 2.0f / windowHeight;

	std::stringstream strstream;
	strstream << "FPS: " << (int)(1000.f / Simulation::getInstance().getBoidModelSimulationTime());
	const std::string fpsString = strstream.str();
	textSimple[0] = fpsString.c_str();

	strstream.str(std::string());
	strstream << "#Boids: " << Simulation::getInstance().getBoidModelNumberOfBoids();
	const std::string numBoids = strstream.str();
	textSimple[1] = numBoids.c_str();

	changeBoxTopBottom(sy, sx);
	drawBox(boxBottomVAO[0]);
	renderText(textSimple, -1 + 8 * sx, 1 - windowHeight * sy + 28 * sy, sx, sy);

	switch (GFX::getInstance().getDisplayHelp()){
	case DISPLAY_NONE:
		textSimple[0] = "F1: toggle controls";
		textSimple[1] = "F2: toggle measurements";
		drawBox(boxTopVAO[0]);
		renderText(textSimple, -1 + 8 * sx, 1 - 17 * sy, sx, sy);
		break;
	case DISPLAY_HELP:
		changeBox(textExtended.size(), sy, sx, 2600);
		drawBox(boxExtVAO[0]);
		renderText(textExtended, -1 + (windowWidth + 20)  * sx / 6, 1 - (windowHeight + 75) * sy / 4, sx, sy);
		renderText(textExtended2, -1 + (windowWidth + 1400)  * sx / 6, 1 - (windowHeight + 75) * sy / 4, sx, sy);
		break;
	case DISPLAY_TIME:
		changeBox(Simulation::getInstance().getSimTimeDescriptions().size(), sy, sx, 940);
		drawBox(boxVAO[0]);
		renderText(Simulation::getInstance().getSimTimeDescriptions(), -1 + (windowWidth + 15)  * sx / 4, 1 - (windowHeight + 75) * sy / 4, sx, sy);
		break;
	}
}

void OverlayText::bindShader(){
	shaderText->bind();
}

void OverlayText::unbindShader(){
	shaderText->unbind();
}

Shader* OverlayText::getShader(){
	return shaderText;
}

