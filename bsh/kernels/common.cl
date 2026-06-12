/*
	Declarations and kernels shared by all grid based boid models. This file
	is prepended to every boidModel*_kernel_*.cl at program creation (see
	BoidModelGridBase::loadBoidProgram), so the simParams_t layout and the
	hash/memSet kernels exist only once.

	simParams_t must match the struct in BoidModel.h.
*/

#define boundingBoxFactor 2

typedef struct{
    float x;
    float y;
    float z;
} Float3;

typedef struct{
    uint x;
    uint y;
    uint z;
}Uint3;

typedef struct{
    int x;
    int y;
    int z;
}Int3;

typedef struct{
    Uint3 gridSize;
    uint numCells;
    Float3 worldOrigin;
    Float3 cellSize;

    uint numBodies;
	uint localSize;

	float wSeparation;
	float wAlignment;
	float wCohesion;
	float wOwn;
	float wPath;

	float maxVel;
	float maxVelCor
} simParams_t;

__kernel void memSet(
    __global uint *d_Data,
    uint val,
    uint N
){
    if(get_global_id(0) < N)
        d_Data[get_global_id(0)] = val;
}

int4 getGridPos(
	float4 pos, 
	__constant simParams_t* params)
{
	 int4 gridPos;
	 gridPos.x = (int) floor((pos.x - params->worldOrigin.x)/params->cellSize.x);
	 gridPos.y = (int) floor((pos.y - params->worldOrigin.y)/params->cellSize.y);
	 gridPos.z = (int) floor((pos.z - params->worldOrigin.z)/params->cellSize.z);
	 gridPos.x = clamp(gridPos.x, 0, (int)params->gridSize.x - 1);
	 gridPos.y = clamp(gridPos.y, 0, (int)params->gridSize.y - 1);
	 gridPos.z = clamp(gridPos.z, 0, (int)params->gridSize.z - 1);
	 return gridPos;
}

__kernel void getGridHash(
	__global float4* posUnsorted, 
	__global int* gridHashUnsorted, 
	__global int* gridIndexUnsorted, 
	__constant simParams_t* params)
{
	int id = get_global_id(0);
	float4 pos = posUnsorted[id];
	int4 gridPos = getGridPos(pos, params);

	gridHashUnsorted[id] = gridPos.x + (params->gridSize.x) * gridPos.z + (params->gridSize.z) * (params->gridSize.x) * gridPos.y;
	gridIndexUnsorted[id] = id;
}
