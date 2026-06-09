#ifndef _SHADER_H_
#define _SHADER_H_

#include "stdafx.h"

class Shader {
public:
	Shader(); // Default constructor
	Shader(const char *vsFile, const char *fsFile); // Constructor for creating a shader from two shader filenames
	Shader(const char *vsFile, const char *fsFile, const char *gsFile); //Constructor with additional geometry shader
	~Shader(); // Deconstructor for cleaning up

	void init(const char *vsFile, const char *fsFile); // Initialize our shader file if we have to
	void init_withGeo(const char *vsFile, const char *fsFile, const char *gsFile);

	void bind(); // Bind our GLSL shader program
	void unbind(); // Unbind our GLSL shader program

	unsigned int id(); // Get the identifier for our program

private:
	unsigned int shader_id; // The shader program identifier
	unsigned int shader_vp; // The vertex shader identifier
	unsigned int shader_fp; // The fragment shader identifier
	unsigned int shader_gp;

	bool hasGeo;
	bool inited; // Whether or not we have initialized the shader
};
#endif