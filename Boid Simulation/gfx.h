#ifndef _GFX_H_
#define _GFX_H_

#include "stdafx.h"
#include "shader.h"
#include "worldBox.h"
#include "simulation.h"
#include "overlayText.h"

/* 
  Display class for the simulation (singleton usage).
*/
class GFX {
	private:
		int windowHeight = 480;
		int windowWidth = 640;

		// stores which overlay is displayed
		int displayHelp = DISPLAY_NONE;
		
		// aspect ratio of windowWidth to Height
		float aspectRatio;
		float fov = 60.0f;	//field of view
		float nearClip = 0.1f;	//clipping distance near
		float farClip = 3000.0f;	//clipping distance far

		// if a specific boid should be follow
		bool follow;
		unsigned int boidFollowed;

		// view matrix
		glm::mat4 view;
		// projection matrix
		glm::mat4 projection;
		// model matrix
		glm::mat4 model;

		// position vector of the camera
		glm::vec3 eye;
		// position where the camera is pointed to
		glm::vec3 camCenter;
		glm::vec3 camPitch;
		// directional vector of the camera
		glm::vec3 eyeToCam;
		// perpendicular vector from eyeToCam, used for sideways navigation
		glm::vec3 eyeOrtho;

		// holds the current camera preset position
		unsigned int currentCamPos;

		// angle for the camera, calculated by mouse movement
		float angleX;
		float angleY;

		float deltaX;
		float deltaY;

		// positions where mouse movement was started
		int mouseOriginX;
		int mouseOriginY;

		// check if camera is inside the Skybox and correct if neccessary.
		void checkCamera();

		// freeglut callbacks
		void setupRenderCallback();
		void setupKeyboardCallback();
		void setupMouseCallback();
		void setupMotionCallback();
		void setupSpecialKeyCallback();
		void setupWindowResizeCallback();
		void setupAppDestroyCallback();
		void setupTimerCallback();
		void setupIdleCallback();

		GFX();
		~GFX();

		static GFX* pInstance;
	public:

		static inline GFX &getInstance() {
			if (NULL == pInstance) { pInstance = new GFX(); }
			return *pInstance;
		}

		bool initOpenGL();
		void startRendering();

		//set camera to preset camera positions
		void setCam(unsigned int camPos);
		//draw output
		void render();
		//keyboard handler for ascii chars
		void keyboardHandler(unsigned char key, int x, int y);
		//mouse button handler
		void mouseHandler(int button, int state, int x, int y);
		//mouse motion handler
		void motionHandler(int x, int y);
		//handler for special keyboard keys like arrow keys
		void specialKeyHandler(int key, int x, int y);
		//handler for window size changes
		void windowResizeHandler(int w, int h);
		//handler for closing window
		void appDestroyHandler();
		//handler which is active every x milli seconds
		void timerHandler(int ms);
		//handler which is active when nothing happens
		void idleHandler();
		//return current overlay mode
		int getDisplayHelp();
		int getWindowWidth();
		int getWindowHeight();
};
#endif