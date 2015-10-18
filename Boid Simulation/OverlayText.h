// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

#ifndef _OVERLAYTEXT_H_
#define _OVERLAYTEXT_H_

#include "stdafx.h"
#include "shader.h"
#include "renderable.h"
#include "vectorTypes.h"

#include <ft2build.h>
#include FT_FREETYPE_H
//#define FT2_BUILD_LIBRARY



/*
	Renderable text overlaying on the screen.
*/
class OverlayText : public Renderable {
	private:
		const char* fontFilename = "FreeSans.otf";
		//load the bitmap of font
		int loadFont();
		//Render every line of text starting at position x, y. sx and sy are variables that scale the position to the current window size
		void renderText(std::vector<const char*> textVector, float x, float y, float sx, float sy);
		//create text for overlay
		void createText();
		//create a box for better visibility of text
		bool createBox();
		//draw the box with specific VAO
		void drawBox(GLuint vao);
		//change the size of the box to new window size
		void changeBox(int size, float sy, float sx, int width);


		//change coordinates and size of top and bottom box if window resize happens
		void changeBoxTopBottom(float sy, float sx);

		//shader attributes for text
		std::vector<std::string> attribTextName;
		//shader attributes for box
		std::vector<std::string> attribBoxName;
		Shader* shaderText;
		Shader* shaderBox;

		//vertex positions for text
		std::vector<Vec4> boxVertex;
		//vertex color for box
		GLfloat boxColor[4];

		//struct which holds points for text rendering
		struct point {
			GLfloat x;
			GLfloat y;
			GLfloat s;
			GLfloat t;
		};

		GLuint vbo;
		GLuint boxVBO[1], boxVAO[1];
		GLuint boxExtVBO[1], boxExtVAO[1];
		GLuint boxTopVBO[1], boxTopVAO[1];
		GLuint boxBottomVBO[1], boxBottomVAO[1];
		GLuint colorTextLoc, texTextLoc, coordTextLoc;
		GLuint colorBoxLoc, coordBoxLoc, viewBoxLoc;
		GLuint fontSize = 16;
		float lineSpacing = 1.5;

		//textExtended holds information regarding keys
		std::vector<const char*> textExtended;
		std::vector<const char*> textExtended2;
		//textSimple holds information which is displayed when no overlay is active
		std::vector<const char*> textSimple;
		//text measure holds fps information
		std::vector<const char*> textMeasure;

		FT_Library ft;
		FT_Face face;

	public:
		OverlayText();
		~OverlayText();
		void render();
		Shader* getShader();
		void bindShader();
		void unbindShader();
};

#endif