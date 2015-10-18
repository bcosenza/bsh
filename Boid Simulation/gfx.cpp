#include "stdafx.h"
#include "gfx.h"

#define GL_PROGRAM_POINT_SIZE 0x8642

GFX* GFX::pInstance = NULL;

GFX::GFX(){
	aspectRatio = (float)windowWidth / (float)windowHeight;

	projection = glm::perspective(fov, aspectRatio, nearClip, farClip);
	model =  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0));

	setCam(CAMERA_PRESET_STANDARD);
	currentCamPos = CAMERA_PRESET_STANDARD;

	//define Y as up axis
	camPitch = glm::vec3(0.0f, 1.0f, 0.0f);;

	mouseOriginX = 0;
	mouseOriginY = 0;

	angleX = 0.f;
	angleY = 0.f;
	deltaX = 0.f;
	deltaY = 0.f;

	follow = false;
}

GFX::~GFX(){}

bool GFX::initOpenGL(){
	int argc = 1;
	char* argv[] = { "something" };
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Behavioural Spherical Harmonic - Agent-based Simulation");
	
	GLenum glew_status = glewInit();
	if (glew_status != GLEW_OK) {
		fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
		return FALSE;
	}

	if (!GLEW_VERSION_2_0) {
		fprintf(stderr, "Error: your graphic card does not support OpenGL 2.0\n");
		return FALSE;
	}

	//setup callbacks
	setupRenderCallback();
	setupIdleCallback();
	setupWindowResizeCallback();
	setupKeyboardCallback();
	setupSpecialKeyCallback();
	setupMotionCallback();
	setupMouseCallback();

	//enable point sizes for boids
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	

	return TRUE;
}

void GFX::startRendering(){
	glutMainLoop();
}

void GFX::render(){
	//make a simulation step
	Simulation::getInstance().simulationStep();

	glViewport(0, 0, windowWidth, windowHeight);
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//render everything in renderList
	std::vector<Renderable*> renderList = Simulation::getInstance().getRenderList();
	for (int i = 0; i < renderList.size(); i++){
		renderList[i]->render();
	}

	//worldBox->render();

	glutSwapBuffers();
}

void GFX::setCam(unsigned int camPos){
	float camPosX, camPosY, camPosZ;
	float camCenterX, camCenterY, camCenterZ;

	currentCamPos = camPos;

	switch (camPos){
	case CAMERA_PRESET_STANDARD:
		camPosX = CELL_SIZE_X * GRID_SIZE_X / 8;
		camPosY = CELL_SIZE_Y * GRID_SIZE_Y / 2;
		camPosZ = -CELL_SIZE_Z * GRID_SIZE_Z / 2;

		camCenterX = CELL_SIZE_X * GRID_SIZE_X / 2;
		camCenterY = CELL_SIZE_Y * GRID_SIZE_Y / 2;
		camCenterZ = CELL_SIZE_Z * GRID_SIZE_Z / 2;

		eye = glm::vec3(camPosX, camPosY, camPosZ);
		camCenter = glm::vec3(camCenterX, camCenterY, camCenterZ);
		break;

	case CAMERA_PRESET_SH:
		camPosX = CELL_SIZE_X * GRID_SIZE_X_SH / 8;
		camPosY = CELL_SIZE_Y * GRID_SIZE_Y_SH / 2;
		camPosZ = -CELL_SIZE_Z * GRID_SIZE_Z_SH / 2;

		camCenterX = CELL_SIZE_X * GRID_SIZE_X_SH / 2;
		camCenterY = CELL_SIZE_Y * GRID_SIZE_Y_SH / 2;
		camCenterZ = CELL_SIZE_Z * GRID_SIZE_Z_SH / 2;

		eye = glm::vec3(camPosX, camPosY, camPosZ);
		camCenter = glm::vec3(camCenterX, camCenterY, camCenterZ);
		break;

	case CAMERA_PRESET_2D:
		camPosX = CELL_SIZE_X * GRID_SIZE_X_SH_2D / 2;
		camPosY = CELL_SIZE_Y * GRID_SIZE_Y_SH_2D * 100;
		camPosZ = CELL_SIZE_Z * GRID_SIZE_Z_SH_2D / 2;

		camCenterX = CELL_SIZE_X * GRID_SIZE_X_SH_2D / 2;
		camCenterY = CELL_SIZE_Y * GRID_SIZE_Y_SH_2D / 2;
		camCenterZ = CELL_SIZE_Z * GRID_SIZE_Z_SH_2D / 2;

		eye = glm::vec3(camPosX, camPosY, camPosZ);
		camCenter = glm::vec3(camCenterX, camCenterY, camCenterZ);
		motionHandler(0, 5);	//dirty fix for white screen problem
		break;

	case CAMERA_PRESET_2D_FAR:
		camPosX = CELL_SIZE_X * GRID_SIZE_X_GRID_2D / 2;
		camPosY = CELL_SIZE_Y * GRID_SIZE_Y_GRID_2D * 150;
		camPosZ = CELL_SIZE_Z * GRID_SIZE_Z_GRID_2D / 2;

		camCenterX = CELL_SIZE_X * GRID_SIZE_X_GRID_2D / 2;
		camCenterY = CELL_SIZE_Y * GRID_SIZE_Y_GRID_2D / 2;
		camCenterZ = CELL_SIZE_Z * GRID_SIZE_Z_GRID_2D / 2;

		eye = glm::vec3(camPosX, camPosY, camPosZ);
		camCenter = glm::vec3(camCenterX, camCenterY, camCenterZ);
		motionHandler(0, 5);	//dirty fix for white screen problem
		break;
	}

	eyeToCam = camCenter - eye;
	eyeOrtho = glm::vec3(eyeToCam.z, 0.0, -eyeToCam.x);
}

void GFX::checkCamera(){
	float x = eye.x;
	float y = eye.y;
	float z = eye.z;
	float max = SKYBOX_SIZE - 20.f;

	if (x > max)
		x = max;
	else if (x < -max)
		x = -max;

	if (y > max)
		y = max;
	else if (y < -max)
		y = -max;

	if (z > max)
		z = max;
	else if (z < -max)
		z = -max;

	eye = glm::vec3(x, y, z);
	camCenter = eye + eyeToCam;
}

void GFX::keyboardHandler(unsigned char key, int x, int y){
	//this way we can exit the program cleanly
	switch (key)
	{
	case '1':	//switch model to boid simple
	case '2':	//switch model to boid grid
	case '3':	//switch model to boid SH
	case '4':	//switch model to boid SH 2D
	case '5':   //switch model to boid grid 2D
	case '6':   //switch model to boid sh with way finding
	case '7':	//switch model to boid sh with way finding version 2
	case '8':	//switch model to boid sh with obstacle avoidance
	case '9':   //switch model to boid sh with obstacle avoidance and sh group avoidance
	case '0':   //switch model to boid sh tunnel example
	case 'r':   //restart model
	case 'R':	
	case 'v':	//make worldbox visible/invisible
	case 'V':
	case 't':	//switch current initial boid setup
	case 'T':
	case 'g':	//
	case 'G':
	case 's':	//make skybox invisible/visible
	case 'S':	
		Simulation::getInstance().keyPress(key); //handled by controller
		break;
	case '+':	//increase boids
	case '-':	//decrease boids
		follow = false;
		setCam(currentCamPos);
		Simulation::getInstance().keyPress(key); //handled by controller
		break;
	case '\t':	//switch to next camera position
		currentCamPos = (currentCamPos + 1) % CAMERA_PRESET_SIZE;
		setCam(currentCamPos);
		break;
	case 'f':	//follow boid
	case 'F':
		if (follow){
			follow = false;
			setCam(currentCamPos);
		}
		else
		{
			follow = true;
			boidFollowed = rand() % Simulation::getInstance().getBoidModelNumberOfBoids();
		}
		break;
	case 'c':
	case 'C':	//reset cam
		setCam(currentCamPos);
		break;
	case '\033': // escape quits
	case '\015': // Enter quits
	case 'Q': // Q quits
	case 'q': // q (or escape) 
		// Cleanup up and quit
		appDestroyHandler();
		break;
	}
}

void GFX::mouseHandler(int button, int state, int x, int y){
	if (button == GLUT_LEFT_BUTTON) {

		// when the button is released
		if (state == GLUT_UP) {
			mouseOriginX = -1;
			mouseOriginY = -1;
		}
		else  {// state = GLUT_DOWN
			mouseOriginX = x;
			mouseOriginY = y;
		}
	}
}

void GFX::motionHandler(int x, int y){
	if (mouseOriginX >= 0) {
		deltaX = (x - mouseOriginX) * 0.01f;
	}

	if (mouseOriginY >= 0){
		deltaY = (y - mouseOriginY) * 0.01f;
	}

	angleX = -deltaX;
	angleY = deltaY;

	glm::vec3 checkAxis = glm::normalize(eyeToCam);

	glm::vec3 rot;
	
	if(abs(checkAxis.x) > abs(checkAxis.z)){
		if (checkAxis.x > 0.f)
			rot = glm::rotateZ(eyeToCam, -angleY);
		else
			rot = glm::rotateZ(eyeToCam, angleY);
	}
	else
	{
		if (checkAxis.z > 0.f)
			rot = glm::rotateX(eyeToCam, angleY);
		else
			rot = glm::rotateX(eyeToCam, -angleY);
	}

	rot = glm::rotateY(rot, angleX);

	//not needed anymore better calculation available below
	//eyeOrtho = glm::rotateY(eyeOrtho, angleX);
	//eyeOrtho = glm::rotateX(eyeOrtho, angleY);

	glm::vec3 diff = (eyeToCam - rot);
	diff /= 2;

	eye = eye + diff;
	camCenter = camCenter - diff;
	eyeToCam = camCenter - eye;

	//easier than above
	eyeOrtho.x = +eyeToCam.z;
	eyeOrtho.z = -eyeToCam.x;

	deltaX = 0;
	deltaY = 0;
}

	void GFX::specialKeyHandler(int key, int x, int y){
		switch (key){
		case GLUT_KEY_F1:
			if (displayHelp == DISPLAY_HELP)
				displayHelp = DISPLAY_NONE;
			else
				displayHelp = DISPLAY_HELP;
			break;
		case GLUT_KEY_F2:
			if (displayHelp == DISPLAY_TIME)
				displayHelp = DISPLAY_NONE;
			else
				displayHelp = DISPLAY_TIME;
			break;
		case GLUT_KEY_UP:
			eye = eye + eyeToCam * CAM_MOVE_FACTOR_FWD;
			camCenter = camCenter + eyeToCam * CAM_MOVE_FACTOR_FWD;
			break;
		case GLUT_KEY_DOWN:
			eye = eye - eyeToCam * CAM_MOVE_FACTOR_FWD;
			camCenter = camCenter - eyeToCam * CAM_MOVE_FACTOR_FWD;
			break;
		case GLUT_KEY_LEFT:
			eye = eye + eyeOrtho * CAM_MOVE_FACTOR_SIDE;
			camCenter = camCenter + eyeOrtho * CAM_MOVE_FACTOR_SIDE;
			break;
		case GLUT_KEY_RIGHT:
			eye = eye - eyeOrtho * CAM_MOVE_FACTOR_SIDE;
			camCenter = camCenter - eyeOrtho * CAM_MOVE_FACTOR_SIDE;
			break;
		}

		checkCamera();
	}

void GFX::windowResizeHandler(int w, int h){
	if (h == 0)
		h = 1;

	windowWidth = w;
	windowHeight = h;
	
	aspectRatio = (float)windowWidth / (float)windowHeight;

	projection = glm::perspective(fov, aspectRatio, nearClip, farClip);
	glViewport(0, 0, windowWidth, windowHeight);
}

void GFX::appDestroyHandler(){
	glutLeaveMainLoop();
}

void GFX::timerHandler(int ms){
	setupTimerCallback();
	glutPostRedisplay();
}

void GFX::idleHandler(){

	if (follow){
		float posX, posY, posZ, velX, velY, velZ;
		Simulation::getInstance().getPosVelOfBoid(&boidFollowed, &posX, &posY, &posZ, &velX, &velY, &velZ);
		camCenter = glm::vec3(posX, posY, posZ);
		glm::vec3 vel = glm::vec3(velX, velY, velZ);
		vel = glm::normalize(vel);
		eye = camCenter - CAM_FOLLOW_DISTANCE_FACTOR * vel + 1.5f * glm::vec3(-vel.z, 0.0f, vel.x);
		eyeToCam = camCenter - eye;
		eyeOrtho = glm::vec3(eyeToCam.z, 0.0, -eyeToCam.x);
	}

	glm::mat4 view = glm::lookAt(eye, camCenter, camPitch);
	glm::mat4 mvp = projection * view * model;
	

	std::vector<Renderable*> renderList = Simulation::getInstance().getRenderList();

	for (int i = 0; i < renderList.size(); i++){
		renderList[i]->bindShader();
		GLint loc = glGetUniformLocation(renderList[i]->getShader()->id(), "m_transform");
		if (loc >= 0)
			glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(mvp));
		renderList[i]->unbindShader();
	}

	glutPostRedisplay();
}


int GFX::getDisplayHelp(){
	return displayHelp;
}

int GFX::getWindowHeight(){
	return windowHeight;
}

int GFX::getWindowWidth(){
	return windowWidth;
}


GFX* gfxCurrentInstance;

extern "C"
void renderCallback()
{
	gfxCurrentInstance->render();
}

extern "C"
void keyboardCallback(unsigned char key, int x, int y)
{
	gfxCurrentInstance->keyboardHandler(key, x, y);
}

extern "C"
void mouseCallback(int button, int state, int x, int y)
{
	gfxCurrentInstance->mouseHandler(button, state, x, y);
}

extern "C"
void motionCallback(int x, int y)
{
	gfxCurrentInstance->motionHandler(x, y);
}

extern "C"
void specialKeyCallback(int key, int x, int y)
{
	gfxCurrentInstance->specialKeyHandler(key, x, y);
}

extern "C"
void windowResizeCallback(int w, int h)
{
	gfxCurrentInstance->windowResizeHandler(w, h);
}

extern "C"
void appDestroyCallback()
{
	gfxCurrentInstance->appDestroyHandler();
}

extern "C"
void timerCallback(int ms)
{
	gfxCurrentInstance->timerHandler(ms);
}

extern "C"
void idleCallback()
{
	gfxCurrentInstance->idleHandler();
}

void GFX::setupRenderCallback()
{
	::gfxCurrentInstance = this;
	::glutDisplayFunc(::renderCallback);
}

void GFX::setupKeyboardCallback()
{
	::gfxCurrentInstance = this;
	::glutKeyboardFunc(::keyboardCallback);
}

void GFX::setupMouseCallback()
{
	::gfxCurrentInstance = this;
	::glutMouseFunc(::mouseCallback);
}

void GFX::setupMotionCallback()
{
	::gfxCurrentInstance = this;
	::glutMotionFunc(::motionCallback);
}

void GFX::setupSpecialKeyCallback()
{
	::gfxCurrentInstance = this;
	::glutSpecialFunc(::specialKeyCallback);
}

void GFX::setupWindowResizeCallback()
{
	::gfxCurrentInstance = this;
	::glutReshapeFunc(::windowResizeCallback);
}

void GFX::setupAppDestroyCallback()
{
	::gfxCurrentInstance = this;
	::glutDisplayFunc(::appDestroyCallback);
}

void GFX::setupTimerCallback()
{
	::gfxCurrentInstance = this;
	::glutTimerFunc(30, ::timerCallback, 30);
}

void GFX::setupIdleCallback()
{
	::gfxCurrentInstance = this;
	::glutIdleFunc(::idleCallback);
}
