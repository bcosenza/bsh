#ifndef _SIMPARAM_H_
#define _SIMPARAM_H_

/*
	Header file to keep simulation parameters in one place
*/

//debug mode
#define DEBUG TRUE

//use SH for wayfinding kernel
#define USE_SH_FOR_PATH TRUE
#define USE_LOOKAHEAD FALSE

//draw triangles instead of points
#define TRIANGLE FALSE

//path for the folder where log files are stored
#define LOG_PATH_WIN ".\\logs"

//edge size of skybox
#define SKYBOX_SIZE 1200.f

//point size of boids
#define BOID_POINT_SIZE 1.4f

//color for boids
#define BOID_COLOR Vec4(.0, .0, 0.0, 1.0)

//color for world box
#define WORLD_BOX_COLOR Vec4(1.0, 1.0, 1.0, 1.0)

//camera movement factor
#define CAM_MOVE_FACTOR_SIDE 0.3f
#define CAM_MOVE_FACTOR_FWD 0.05f

//factor of distance between camera and followed boid
#define CAM_FOLLOW_DISTANCE_FACTOR 6.f

//factor of radius for initial setup of test case
#define TEST_SETUP_RADIUS 2.f

//origin of the world
#define WORLD_ORIGIN_X 0.0f
#define WORLD_ORIGIN_Y 0.0f
#define WORLD_ORIGIN_Z 0.0f

//size of a cell on axis xyz
#define CELL_SIZE_X 15.0f
#define CELL_SIZE_Y 15.0f
#define CELL_SIZE_Z 15.0f

//number of cells per axis of the SH model
#define GRID_SIZE_X_SH 20
#define GRID_SIZE_Y_SH 20
#define GRID_SIZE_Z_SH 20

//number of cells per axis of the SH model with goal seeking
#define GRID_SIZE_X_SH_WAY1 16
#define GRID_SIZE_Y_SH_WAY1 16
#define GRID_SIZE_Z_SH_WAY1 16

//number of cells per axis of the SH model with obstacle avoidance
#define GRID_SIZE_X_SH_OBSTACLE 16
#define GRID_SIZE_Y_SH_OBSTACLE 16
#define GRID_SIZE_Z_SH_OBSTACLE 16

//number of cells per axis of the 2D SH model
#define GRID_SIZE_X_SH_2D 100
#define GRID_SIZE_Y_SH_2D 1
#define GRID_SIZE_Z_SH_2D 100

//number of cells per axis of the 2D Grid model
#define GRID_SIZE_X_GRID_2D 160
#define GRID_SIZE_Y_GRID_2D 1
#define GRID_SIZE_Z_GRID_2D 160

//number of cells per axis of the simple and GPU optimized model
#define GRID_SIZE_X 80
#define GRID_SIZE_Y 80
#define GRID_SIZE_Z 80

//number of initial boids per model 
// do not set too high otherwise 
//M$ watchdog timer will terminate OCL kernel
#define NUM_BOIDS 8192//32768//65536//*/131072
#define NUM_BOIDS_SIMPLE 8192
#define NUM_BOIDS_GRID 8192
#define NUM_BOIDS_SH 8192
#define NUM_BOIDS_SH_2D 65536
#define NUM_BOIDS_GRID_2D 65536

//minimum number of boids user can set the simulation too
#define NUM_BOIDS_MIN 2048
#define NUM_BOIDS_MAX 524288

//OCL local memory usage sizes

//!!!needs to be the same as in bitonic_sort.cl!!!
#define LOCAL_SIZE_LIMIT 2048	//prefered local memory size for openCL bitonic sort 
//!!!needs to be the same as in bitonic_sort.cl!!!

#define LOCAL_SIZE_VEC4 256  
#define LOCAL_PREF 256		//prefered size of local memory for openCL kernels

//weight for the coefficients for the SH boid model
#define WEIGHT_ALIGNMENT_SH .2f				//0.2	||
#define WEIGHT_SEPARATION_SH 0.01f			//0.01	||
#define WEIGHT_COHESION_SH 0.002f			//0.002	||
#define WEIGHT_OWN_SH 1.1f					//1.1	||

//weight for the coefficients for the SH boid model with way finding
#if USE_LOOKAHEAD
	#define WEIGHT_ALIGNMENT_SH_WAY1 .20f				//0.2	||
	#define WEIGHT_SEPARATION_SH_WAY1 0.01f			//0.01	||
	#define WEIGHT_COHESION_SH_WAY1 0.02f			//0.002	||
	#define WEIGHT_OWN_SH_WAY1 0.f					//1.1	||
	#define WEIGHT_GOAL_SH_WAY1 4.5f
#else
	#define WEIGHT_ALIGNMENT_SH_WAY1 .02f				//0.2	||
	#define WEIGHT_SEPARATION_SH_WAY1 0.01f			//0.01	||
	#define WEIGHT_COHESION_SH_WAY1 0.02f			//0.002	||
	#define WEIGHT_OWN_SH_WAY1 1.0f					//1.1	||
	#define WEIGHT_GOAL_SH_WAY1 5.5f
#endif

//weight for the coefficients for the SH boid model
#define WEIGHT_ALIGNMENT_SH_OBSTACLE .0f				//0.2	||
#define WEIGHT_SEPARATION_SH_OBSTACLE 0.001f			//0.01	||
#define WEIGHT_COHESION_SH_OBSTACLE 0.02f			//0.002	||
#define WEIGHT_OWN_SH_OBSTACLE .8f					//1.1	||
#define WEIGHT_GOAL_SH_OBSTACLE 2.0f

//weights for the coefficienst for the 2D SH boid model
#define WEIGHT_ALIGNMENT_SH_2D .1f
#define WEIGHT_SEPARATION_SH_2D 0.004f
#define WEIGHT_COHESION_SH_2D 0.04f
#define WEIGHT_OWN_SH_2D 1.0f

//weights for the coefficients for the GPU optimized boid model
#define WEIGHT_ALIGNMENT_GRID .2f			//0.2	||
#define WEIGHT_SEPARATION_GRID 0.01f		//0.01	||
#define WEIGHT_COHESION_GRID 0.002f			//0.002 ||
#define WEIGHT_OWN_GRID 1.05f				//1.f	||

//weights for the coefficients for the GPU optimized 2D boid model
#define WEIGHT_ALIGNMENT_GRID_2D .4f
#define WEIGHT_SEPARATION_GRID_2D 0.01f
#define WEIGHT_COHESION_GRID_2D 0.05f
#define WEIGHT_OWN_GRID_2D 1.1f

//coefficients for the simple 
//non gpu optimized boid model
#define WEIGHT_ALIGNMENT .2f 
#define WEIGHT_SEPARATION .01f
#define WEIGHT_COHESION 0.002f
#define WEIGHT_OWN 1.05f

#define MAX_VEL_GRID 9.5f //not allowed to be larger than CELL_SIZE
#define MAX_VEL_COR_GRID 10.0f //dependent on MAX_VEL

//maximum velocity for the simple model
//maximum of the correction for the simple model
#define MAX_VEL_SIMPLE 10.f
#define MAX_VEL_COR_SIMPLE 10.f

//integer values of the different models
#define BOID_SIMPLE 1
#define BOID_GRID 2
#define BOID_SH 3
#define BOID_GRID_2D 4
#define BOID_SH_2D 5
#define BOID_SH_WAY1 6
#define BOID_SH_WAY2 7
#define BOID_SH_OBSTACLE 8
#define BOID_SH_OBSTACLE_COMBINED 9
#define BOID_SH_OBSTACLE_TUNNEL 0
// Copyright (c) 2015, Biagio Cosenza.
// Technische Universitaet Berlin. All rights reserved.
//
// This program is provided under a BSD Simplified license. For full
// license terms please see the LICENSE file distributed with this
// source code.

//Model initial placement of boids
//0 - random placement inside the cube
//1 - Test case 2 Groups moving towards each other
//2 - small group and large group moving towards each other
//3 - 4 groups moving towards each other
//4 - 2 groups moving and crossing at center
#define MODEL_INIT_PLACEMENT 0
#define MODEL_INIT_PLACEMENT_MAX 5

//GFX Camera preset positions
//0 - standard Camera Position
//1 - Camera Position for SH
//2 - Camera Position for 2D
//3 - Camera Position for grid 2D
#define CAMERA_PRESET_STANDARD 0
#define CAMERA_PRESET_SH 1
#define CAMERA_PRESET_2D 2
#define CAMERA_PRESET_2D_FAR 3

//used to iterate over the cam positions - must be size of available camera presets
#define CAMERA_PRESET_SIZE 4

//integer values for the overlay display 
#define DISPLAY_NONE 0
#define DISPLAY_HELP 1
#define DISPLAY_TIME 2

#endif // _SIMPARAM_H_