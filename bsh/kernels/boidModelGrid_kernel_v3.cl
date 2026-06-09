// using fast_fast_length instead of fast_length.

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


//same as in boidModel.h
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

//set memory to val
__kernel void memSet(
	__global uint *d_Data,
	uint val,
	uint N
	){
	if (get_global_id(0) < N)
		d_Data[get_global_id(0)] = val;
}

/*check if boid is in border cell and apply force*/
float4 checkAndCorrectBoundaries(uint cell, __constant simParams_t* params)
{
	uint sizePlane = params->gridSize.x * params->gridSize.z;
	float4 cor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

	if (cell >= (params->numCells - sizePlane * boundingBoxFactor))
		cor.y = -params->maxVelCor;
	else if (cell < sizePlane * boundingBoxFactor)
		cor.y = params->maxVelCor;

	cell = cell % sizePlane;

	if (cell >= (sizePlane - params->gridSize.x * boundingBoxFactor))
		cor.z = -params->maxVelCor;
	else if (cell < params->gridSize.x * boundingBoxFactor)
		cor.z = params->maxVelCor;

	cell = cell % params->gridSize.x;

	if (cell >= (params->gridSize.x - boundingBoxFactor))
		cor.x = -params->maxVelCor;

	if (cell < boundingBoxFactor)
		cor.x = params->maxVelCor;

	return cor;
}

//check and correct boundary force with grid position
float4 checkAndCorrectBoundariesWithPos(int4 gridPos, __constant simParams_t* simParams){
	float4 velCor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

	if (gridPos.x < boundingBoxFactor)
		velCor.x = simParams->maxVelCor;

	if (gridPos.x >= (simParams->gridSize.x - boundingBoxFactor))
		velCor.x = -simParams->maxVelCor;

	if (gridPos.y < boundingBoxFactor)
		velCor.y = simParams->maxVelCor;

	if (gridPos.y >= (simParams->gridSize.y - boundingBoxFactor))
		velCor.y = -simParams->maxVelCor;

	if (gridPos.z < boundingBoxFactor)
		velCor.z = simParams->maxVelCor;

	if (gridPos.z >= (simParams->gridSize.z - boundingBoxFactor))
		velCor.z = -simParams->maxVelCor;

	return velCor;
}

/*return cell position of boid*/
int4 getGridPos(
	float4 pos,
	__constant simParams_t* params)
{
	int4 gridPos;
	gridPos.x = (int)floor((pos.x - params->worldOrigin.x) / params->cellSize.x);
	gridPos.y = (int)floor((pos.y - params->worldOrigin.y) / params->cellSize.y);
	gridPos.z = (int)floor((pos.z - params->worldOrigin.z) / params->cellSize.z);
	return gridPos;
}

/*calculate grid hash value for sorting from boid position*/
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

/*find edges in sorted hash array and save this index as start/end index of cell. Also reorder position and velocity array (implementation from nvidia nbody paper)*/
__kernel void findGridEdgeAndReorder(
	__global uint   *cellStart,				//output: cell start index
	__global uint   *cellEnd,				//output: cell end index
	__global float4 *reorderedPos,			//output: reordered by cell hash positions
	__global float4 *reorderedVel,			//output: reordered by cell hash velocities
	__global const uint   *gridHash,		//input: sorted grid hashes
	__global const uint   *gridIndex,		//input: particle indices sorted by hash
	__global const float4 *unsortedPos,     //input: positions array sorted by hash
	__global const float4 *unsortedVel,     //input: velocity array sorted by hash
	__local uint *localHash,				//get_group_size(0) + 1 elements
	uint    numParticles
	){
	uint hash;
	const uint index = get_global_id(0);

	//Handle case when no. of boids not multiple of block size
	if (index < numParticles){
		hash = gridHash[index];

		//Load hash data into local memory so that we can look 
		//at neighboring boids's hash value without loading
		//two hash values per thread
		localHash[get_local_id(0) + 1] = hash;

		//First thread in block must load neighbor boids hash
		if (index > 0 && get_local_id(0) == 0)
			localHash[0] = gridHash[index - 1];
	}

	barrier(CLK_LOCAL_MEM_FENCE);

	if (index < numParticles){
		//Border case
		if (index == 0)
			cellStart[hash] = 0;

		//Main case
		else{
			if (hash != localHash[get_local_id(0)])
				cellEnd[localHash[get_local_id(0)]] = cellStart[hash] = index;
		};

		//Another border case
		if (index == numParticles - 1)
			cellEnd[hash] = numParticles;


		//Now use the sorted index to reorder the pos and vel arrays
		uint sortedIndex = gridIndex[index];
		float4 pos = unsortedPos[sortedIndex];
		float4 vel = unsortedVel[sortedIndex];

		reorderedPos[index] = pos;
		reorderedVel[index] = vel;
	}
}



/*simulation step*/
__kernel void simulate(__global float4* pos,
	__global float4* pos_out,
	__global float4* vel,
	__global float4* vel_out,
	__global uint *cellStart,
	__global uint *cellEnd,
	__local float4 *localPos,
	__local float4 *localVel,
	__constant simParams_t* simParams,
	__global uint *range_out,
	float dt)
{

	uint id = get_global_id(0);

	float4 perceivedPos = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 perceivedVel = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 separation = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 distance = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);	//maximum velocity

	int flockMatesVisible = 0;

	float4 velOwn = vel[id];
	velOwn.w = 0.0f;
	float4 posOwn = pos[id];
	int4 gridPos = getGridPos(posOwn, simParams);

	float4 velCor = checkAndCorrectBoundariesWithPos(gridPos, simParams);

	//accumulate data from sourrounding cells
	for (int z = -1; z <= 1; z++){
		for (int y = -1; y <= 1; y++){
			for (int x = -1; x <= 1; x++){
				int4 gridPos2 = gridPos + (int4)(x, y, z, 0);

				//skip out of bound cells
				if (gridPos2.x < 0 || gridPos2.x >= simParams->gridSize.x)
					continue;

				if (gridPos2.y < 0 || gridPos2.y >= simParams->gridSize.y)
					continue;

				if (gridPos2.z < 0 || gridPos2.z >= simParams->gridSize.z)
					continue;

				//calculate grid hash
				uint hash = gridPos2.x + (simParams->gridSize.x) * gridPos2.z + (simParams->gridSize.z) * (simParams->gridSize.x) * gridPos2.y;
				uint end = cellEnd[hash];
				uint start = cellStart[hash];
				uint range = end - start;
				//Skip empty cell
				if (range == 0)
					continue;

				//Iterate over boids in this cell
				for (uint j = start; j < end; j++){
					if (j == id)
						continue;

					float4 p = pos[j];
					float4 v = vel[j];

					float4 distance = p - posOwn;		//distance vector to other boid
					distance.w = 0.0f;

					//check if in range for pair-wise interaction
					if (fast_length(distance) < 5.0f){
						float dotP = dot(-velOwn, distance);
						float angle = dotP / (fast_length(velOwn) * fast_length(distance));		//calculate acute angle between self and other boid

						if (dotP < 0.f || fabs(degrees(acos(angle))) > 45){	//check if other boid is visible, dot product indicates that angle is > 90°

							flockMatesVisible++;							//increase counter how many boids are visible
							perceivedPos += p;				//add other boids position to perceived center of mass
							perceivedVel += v;				//add other boids velocity to perceived aligned velocity

							if (fast_length(distance) < 2.5f)						//check if other boid is near enough to separate
								separation -= distance;

						}
					}
				}
			}
		}
	}

	if (flockMatesVisible >= 1){
		perceivedPos = (perceivedPos / flockMatesVisible) - posOwn;
		perceivedVel = (perceivedVel / flockMatesVisible) - velOwn;
	}


	//calculate new velocities 
	velOwn = velOwn * simParams->wOwn + perceivedPos * simParams->wCohesion + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
	velOwn.w = 0.0;

	//truncate velocity to max velocity
	//velOwn = clamp(velOwn, -mVel, mVel);
	//using the magnitude of the velocity is nicer than clamping it

	float len = fast_length(velOwn);

	if (len > simParams->maxVel){
		velOwn.x = (velOwn.x / len) * simParams->maxVel;
		velOwn.z = (velOwn.z / len) * simParams->maxVel;
		velOwn.y = (velOwn.y / len) * simParams->maxVel;
	}


	//add correction velocity if boid in border cell
	velOwn += velCor;

	//write back velocity and new position
	vel_out[id] = velOwn;
	pos_out[id] = posOwn + velOwn * dt;

}

