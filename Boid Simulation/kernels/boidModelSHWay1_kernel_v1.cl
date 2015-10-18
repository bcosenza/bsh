/*
	As boidModelGrid_kernel except it uses localMem bigger than the local group size.
	Using fast methods instead of normal ones.
*/
#define FACTOR .600f
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

/*check if boid is in border cell and apply force*/
float4 checkAndCorrectBoundaries(   uint cell, __constant simParams_t* params)
{
	uint sizePlane = params->gridSize.x * params->gridSize.z;
	float4 cor = (float4)(0.0f,0.0f,0.0f,0.0f);

	if(cell >= (params->numCells - sizePlane * boundingBoxFactor))
		cor.y = -params->maxVelCor;
	else if(cell < sizePlane * boundingBoxFactor)
		cor.y = params->maxVelCor;

	cell = cell % sizePlane;

	if(cell >= (sizePlane - params->gridSize.x * boundingBoxFactor))
		cor.z = -params->maxVelCor;
	else if(cell < params->gridSize.x * boundingBoxFactor)
		cor.z = params->maxVelCor;

	cell = cell % params->gridSize.x;

	if(cell >= (params->gridSize.x - boundingBoxFactor))
		cor.x = -params->maxVelCor;

	if(cell < boundingBoxFactor)
		cor.x = params->maxVelCor;

	return cor;
}

int4 getGridPos(
	float4 pos, 
	__constant simParams_t* params)
{
	 int4 gridPos;
	 gridPos.x = (int) floor((pos.x - params->worldOrigin.x)/params->cellSize.x);
	 gridPos.y = (int) floor((pos.y - params->worldOrigin.y)/params->cellSize.y);
	 gridPos.z = (int) floor((pos.z - params->worldOrigin.z)/params->cellSize.z);
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

__kernel void findGridEdgeAndReorder(
	__global uint   *cellStart,     //output: cell start index
	__global uint   *cellEnd,       //output: cell end index
	__global float4 *reorderedPos,  //output: reordered by cell hash positions
	__global float4 *reorderedVel,  //output: reordered by cell hash velocities

	__global const uint   *gridHash,    //input: sorted grid hashes
	__global const uint   *gridIndex,   //input: particle indices sorted by hash
	__global const float4 *unsortedPos,     //input: positions array sorted by hash
	__global const float4 *unsortedVel,     //input: velocity array sorted by hash
	__global const float4 *unsortedGoal,
	__global float4 *reorderedGoal,
	__global const float4 *unsortedColor,
	__global float4 *reorderedColor,
	__local uint *localHash,          //get_group_size(0) + 1 elements
	uint    numParticles
	){
	uint hash;
	const uint index = get_global_id(0);

	//Handle case when no. of particles not multiple of block size
	if (index < numParticles){
		hash = gridHash[index];

		//Load hash data into local memory so that we can look 
		//at neighboring particle's hash value without loading
		//two hash values per thread
		localHash[get_local_id(0) + 1] = hash;

		//First thread in block must load neighbor particle hash
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
		float4 goal = unsortedGoal[sortedIndex];
		float4 color = unsortedColor[sortedIndex];

		reorderedPos[index] = pos;
		reorderedVel[index] = vel;
		reorderedGoal[index] = goal;
		reorderedColor[index] = color;
	}
}

__kernel void simulate(
	__global const float4* pos,
	__global float4* pos_out,
	__global const float4* vel,
	__global float4* vel_out,
	__global const uint *cellStart,
	__global const uint *cellEnd,
	__global const float4 *goal,
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

	//calculate straight path to goal
	float4 g = goal[id];
	g.w = 0.0f;

	float4 path = g - posOwn;
	path.w = 0.0f;
	path = fast_normalize(path);

	//calculate new velocities 
	velOwn = velOwn * simParams->wOwn + path * simParams->wPath + perceivedPos * simParams->wCohesion + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
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
	pos_out[id] = posOwn;
}



/*Evaluete 3. order SH. Numerical implementation from P.P. Sloan
  float8 is used to store the 9 coefficients because coefficient 0 is
  constant it is just added at the reconstruction.
*/
float8 SHEval3(float4 vel_in)
{
   float4 vel = vel_in;
   float8 pSH;
   float fC0,fC1,fS0,fS1,fTmpA,fTmpB,fTmpC;
   float fZ2 = vel.y*vel.y;

   //pSH.0 = 0.2820947917738781f;
   pSH.s1 = 0.4886025119029199f*vel.y;
   pSH.s5 = 0.9461746957575601f*fZ2 + -0.3153915652525201f;
   fC0 = vel.x;
   fS0 = vel.z;

   fTmpA = -0.48860251190292f;
   pSH.s2 = fTmpA*fC0;
   pSH.s0 = fTmpA*fS0;
   fTmpB = -1.092548430592079f*vel.y;
   pSH.s6 = fTmpB*fC0;
   pSH.s4 = fTmpB*fS0;
   fC1 = vel.x*fC0 - vel.z*fS0;
   fS1 = vel.x*fS0 + vel.z*fC0;

   fTmpC = 0.5462742152960395f;
   pSH.s7 = fTmpC*fC1;
   pSH.s3 = fTmpC*fS1;

   return pSH;
}

__kernel void evalSH(
	__global float4* vel,
	__global float8* sh_evalX,
	__global float8* sh_evalY,
	__global float8* sh_evalZ,
	__global float* coef0X,
	__global float* coef0Y,
	__global float* coef0Z,
	__local float* sh_c0_localX,
	__local float* sh_c0_localY,
	__local float* sh_c0_localZ, 
	__local float8* sh_eval_localX,
	__local float8* sh_eval_localY,
	__local float8* sh_eval_localZ, 
	__global uint* startIndex,
	__global uint* endIndex
){

	uint id = get_local_id(0);
	uint cell = get_group_id(0);
	uint lSize = get_local_size(0);

	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint index = start + id;
	uint range = end - start;

	sh_eval_localX[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[id] = 0.0f;
	sh_c0_localY[id] = 0.0f;
	sh_c0_localZ[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	while (index < end){
		float4 v = vel[index];
		v.w = 0.0f;

		float8 sh = SHEval3(fast_normalize(v));
		//sh.s0 *= v.y;
		//sh.s1 *= v.z;
		//sh.s2 *= v.x;
		//sh.s3 *= v.y;
		//sh.s4 *= v.z;
		//sh.s5 *= v.x;
		//sh.s6 *= v.y;
		//sh.s7 *= v.z;
		sh_eval_localX[id] += sh * v.x;
		sh_eval_localY[id] += sh * v.y;
		sh_eval_localZ[id] += sh * v.z;

		sh_c0_localX[id] += 0.2820947917738781f * v.x;
		sh_c0_localY[id] += 0.2820947917738781f * v.y;
		sh_c0_localZ[id] += 0.2820947917738781f * v.z;

		index += lSize;
	}

	uint k = lSize / 2;
	while (id < k){
		sh_eval_localX[id] += sh_eval_localX[id + k];
		sh_eval_localY[id] += sh_eval_localY[id + k];
		sh_eval_localZ[id] += sh_eval_localZ[id + k];
		sh_c0_localX[id] += sh_c0_localX[id + k];
		sh_c0_localY[id] += sh_c0_localY[id + k];
		sh_c0_localZ[id] += sh_c0_localZ[id + k];
		k = k / 2;

		barrier(CLK_LOCAL_MEM_FENCE);
	}

	
	if (range > 0 && id == 0){
		sh_eval_localX[id] = sh_eval_localX[id] / range;
		sh_eval_localY[id] = sh_eval_localY[id] / range;
		sh_eval_localZ[id] = sh_eval_localZ[id] / range;
		sh_c0_localX[id] = sh_c0_localX[id] / range;
		sh_c0_localY[id] = sh_c0_localY[id] / range;
		sh_c0_localZ[id] = sh_c0_localZ[id] / range;
	}

	if (id == 0){
		sh_evalX[cell] = sh_eval_localX[id];
		sh_evalY[cell] = sh_eval_localY[id];
		sh_evalZ[cell] = sh_eval_localZ[id];
		coef0X[cell] = sh_c0_localX[id];
		coef0Y[cell] = sh_c0_localY[id];
		coef0Z[cell] = sh_c0_localZ[id];
	}
}

/*kernel to use the SH calculations on boids*/
__kernel void useSH(
	__global const float4* vel,
	__global float4* vel_out,
	__global const uint* startIndex,
	__global const uint* endIndex,
	__global const float8* sh_evalX,
	__global const float8* sh_evalY,
	__global const float8* sh_evalZ,
	__constant simParams_t* simParams,
	__global const float4* pos,
	__global float4* pos_out,
	__local float8* sh_eval_localX,
	__local float8* sh_eval_localY,
	__local float8* sh_eval_localZ,
	__global const float* coef0X,
	__global const float* coef0Y,
	__global const float* coef0Z,
	__local float* sh_c0_localX,
	__local float* sh_c0_localY,
	__local float* sh_c0_localZ,
	const float dt)
{
	uint id = get_global_id(0);
	uint lSize = get_local_size(0);
	uint lId = get_local_id(0);

	float8 sumSHX;
	float8 sumSHY;
	float8 sumSHZ;

	__local float4 posCell[256];
	float4 shCor = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 velOwn = vel[id];
	float4 posOwn = pos[id];
	posOwn.w = 0.0f;
	velOwn.w = 0.0f;

	float8 SHSelf = SHEval3(normalize(velOwn));

	int4 gridPos = getGridPos(posOwn, simParams);
	int cell = gridPos.x + (simParams->gridSize.x) * gridPos.z + (simParams->gridSize.z) * (simParams->gridSize.x) * gridPos.y;
	uint plane = simParams->gridSize.x * simParams->gridSize.z;
	float4 velCor = checkAndCorrectBoundariesWithPos(gridPos, simParams);

	sh_eval_localX[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[lId] = 0.0f;
	sh_c0_localY[lId] = 0.0f;
	sh_c0_localZ[lId] = 0.0f;
	posCell[lId] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for (uint i = 0; i < (simParams->numCells / lSize); i++){

		uint x = lSize * i + lId;
		float4 p = (float4)(((x % plane) % simParams->gridSize.x) * simParams->cellSize.x + simParams->cellSize.x / 2, (x / plane)   * simParams->cellSize.y + simParams->cellSize.y / 2, ((x % plane) / simParams->gridSize.x) * simParams->cellSize.z + simParams->cellSize.z / 2, 0.0f);

		float4 distV = posOwn - p;
		float dist = fast_distance(p, posOwn);

		if (i == cell){
			sh_eval_localX[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_eval_localY[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_eval_localZ[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_c0_localX[lId] = 0.0f;
			sh_c0_localY[lId] = 0.0f;
			sh_c0_localZ[lId] = 0.0f;
		}
		else {
			sh_eval_localX[lId] = (1.f / (dist)) * sh_evalX[x];
			sh_eval_localY[lId] = (1.f / (dist)) * sh_evalY[x];
			sh_eval_localZ[lId] = (1.f / (dist)) * sh_evalZ[x];
			sh_c0_localX[lId] = (1.f / (dist)) * coef0X[x];
			sh_c0_localY[lId] = (1.f / (dist)) * coef0Y[x];
			sh_c0_localZ[lId] = (1.f / (dist)) * coef0Z[x];
		}

		posCell[lId] = p;
		barrier(CLK_LOCAL_MEM_FENCE);

		for (int j = 0; j < lSize; j++){
			float4 posOther = posCell[j];
			float4 d = posOther - posOwn;
			d.w = 0.f;

			SHSelf = SHEval3(normalize(d));

			float8 SHOtherX = sh_eval_localX[j];
			float8 SHOtherY = sh_eval_localY[j];
			float8 SHOtherZ = sh_eval_localZ[j];

			float sumAllSHX = sh_c0_localX[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHY = sh_c0_localY[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHZ = sh_c0_localZ[j] * 0.2820947917738781f * M_PI_F;

			sumAllSHX += SHSelf.s0 * SHOtherX.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s1 * SHOtherX.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s2 * SHOtherX.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s3 * SHOtherX.s3 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s4 * SHOtherX.s4 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s5 * SHOtherX.s5 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s6 * SHOtherX.s6 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s7 * SHOtherX.s7 * M_PI_F / 4.0f;

			sumAllSHY += SHSelf.s0 * SHOtherY.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s1 * SHOtherY.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s2 * SHOtherY.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s3 * SHOtherY.s3 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s4 * SHOtherY.s4 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s5 * SHOtherY.s5 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s6 * SHOtherY.s6 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s7 * SHOtherY.s7 * M_PI_F / 4.0f;

			sumAllSHZ += SHSelf.s0 * SHOtherZ.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s1 * SHOtherZ.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s2 * SHOtherZ.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s3 * SHOtherZ.s3 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s4 * SHOtherZ.s4 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s5 * SHOtherZ.s5 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s6 * SHOtherZ.s6 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s7 * SHOtherZ.s7 * M_PI_F / 4.0f;

			shCor += (float4)(-sumAllSHZ, sumAllSHY, sumAllSHX, 0.0f) * FACTOR;
			//shCor += (float4)(sumAllSHY, -sumAllSHX, sumAllSHZ, 0.0f);
			//shCor += (float4)(sumAllSHX, sumAllSHZ, -sumAllSHY, 0.0f);
		}
	}

	float len = length(shCor);


	velOwn += shCor;
	//velOwn = velOwn / 2;
	velOwn.w = 0.0f;

	len = length(velOwn);

	if (len > simParams->maxVel){
		velOwn.x = (velOwn.x / len) * simParams->maxVel;
		velOwn.y = (velOwn.y / len) * simParams->maxVel;
		velOwn.z = (velOwn.z / len) * simParams->maxVel;
	}

	//apply correction velocity dependend on boid cell position (border case)
	velOwn += velCor;
	posOwn.w = 1.0;

	vel_out[id] = velOwn;
	pos_out[id] = posOwn + velOwn * dt;
}


__kernel void useSHLookahead(__global const float4* vel,
	__global float4* vel_out,
	__global const uint* startIndex,
	__global const uint* endIndex,
	__global const float8* sh_evalX,
	__global const float8* sh_evalY,
	__global const float8* sh_evalZ,
	__constant simParams_t* simParams,
	__global const float4* pos,
	__global float4* pos_out,
	__local float8* sh_eval_localX,
	__local float8* sh_eval_localY,
	__local float8* sh_eval_localZ,
	__global const float* coef0X,
	__global const float* coef0Y,
	__global const float* coef0Z,
	__local float* sh_c0_localX,
	__local float* sh_c0_localY,
	__local float* sh_c0_localZ,
	float dt)
{
	uint id = get_global_id(0);
	uint lSize = get_local_size(0);
	uint lId = get_local_id(0);

	float8 sumSHX;
	float8 sumSHY;
	float8 sumSHZ;

	__local float4 posCell[256];
	float4 shCor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 velOwn = vel[id];
	float4 posOwn = pos[id];
	posOwn.w = 0.0f;
	velOwn.w = 0.0f;

	float test1 = 10000.f;
	float4 pX = posOwn;

	float8 SHSelf = SHEval3(normalize(velOwn));

	int4 gridPos = getGridPos(posOwn, simParams);
	int cell = gridPos.x + (simParams->gridSize.x) * gridPos.z + (simParams->gridSize.z) * (simParams->gridSize.x) * gridPos.y;
	uint plane = simParams->gridSize.x * simParams->gridSize.z;
	float4 velCor = checkAndCorrectBoundariesWithPos(gridPos, simParams);

	sh_eval_localX[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[lId] = 0.0f;
	sh_c0_localY[lId] = 0.0f;
	sh_c0_localZ[lId] = 0.0f;
	posCell[lId] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for (uint i = 0; i < (simParams->numCells / lSize); i++){

		uint x = lSize * i + lId;
		float4 p = (float4)(((x % plane) % simParams->gridSize.x) * simParams->cellSize.x + simParams->cellSize.x / 2, (x / plane)   * simParams->cellSize.y + simParams->cellSize.y / 2, ((x % plane) / simParams->gridSize.x) * simParams->cellSize.z + simParams->cellSize.z / 2, 0.0f);

		float4 distV = posOwn - p;
		float dist = fast_distance(p, posOwn);

		if (i == cell){
			sh_eval_localX[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_eval_localY[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_eval_localZ[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			sh_c0_localX[lId] = 0.0f;
			sh_c0_localY[lId] = 0.0f;
			sh_c0_localZ[lId] = 0.0f;
		}
		else {
			sh_eval_localX[lId] = (1.f / (dist)) * sh_evalX[x];
			sh_eval_localY[lId] = (1.f / (dist)) * sh_evalY[x];
			sh_eval_localZ[lId] = (1.f / (dist)) * sh_evalZ[x];
			sh_c0_localX[lId] = (1.f / (dist)) * coef0X[x];
			sh_c0_localY[lId] = (1.f / (dist)) * coef0Y[x];
			sh_c0_localZ[lId] = (1.f / (dist)) * coef0Z[x];
		}

		posCell[lId] = p;
		barrier(CLK_LOCAL_MEM_FENCE);

		for (int j = 0; j < lSize; j++){
			float4 posOther = posCell[j];
			float4 d = posOwn - posOther;
			d.w = 0.f;

			float factor = 1.000f;
			float dotP = dot(d, velOwn);
			float angle = degrees(dotP / (fast_length(velOwn)));

			if (dotP < 0.0){
				factor = .00000f;
				angle += 90;
			}

			SHSelf = SHEval3(normalize(d));

			float8 SHOtherX = sh_eval_localX[j];
			float8 SHOtherY = sh_eval_localY[j];
			float8 SHOtherZ = sh_eval_localZ[j];

			float sumAllSHX = sh_c0_localX[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHY = sh_c0_localY[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHZ = sh_c0_localZ[j] * 0.2820947917738781f * M_PI_F;

			sumAllSHX += SHSelf.s0 * SHOtherX.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s1 * SHOtherX.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s2 * SHOtherX.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHX += SHSelf.s3 * SHOtherX.s3 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s4 * SHOtherX.s4 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s5 * SHOtherX.s5 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s6 * SHOtherX.s6 * M_PI_F / 4.0f;
			sumAllSHX += SHSelf.s7 * SHOtherX.s7 * M_PI_F / 4.0f;

			sumAllSHY += SHSelf.s0 * SHOtherY.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s1 * SHOtherY.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s2 * SHOtherY.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHY += SHSelf.s3 * SHOtherY.s3 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s4 * SHOtherY.s4 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s5 * SHOtherY.s5 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s6 * SHOtherY.s6 * M_PI_F / 4.0f;
			sumAllSHY += SHSelf.s7 * SHOtherY.s7 * M_PI_F / 4.0f;

			sumAllSHZ += SHSelf.s0 * SHOtherZ.s0 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s1 * SHOtherZ.s1 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s2 * SHOtherZ.s2 * M_PI_F * 2.0f / 3.0f;
			sumAllSHZ += SHSelf.s3 * SHOtherZ.s3 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s4 * SHOtherZ.s4 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s5 * SHOtherZ.s5 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s6 * SHOtherZ.s6 * M_PI_F / 4.0f;
			sumAllSHZ += SHSelf.s7 * SHOtherZ.s7 * M_PI_F / 4.0f;

			float4 test = (float4)(sumAllSHX, sumAllSHY, sumAllSHZ, 0.0f);

			//float len = length(test);

			//if (len > simParams->maxVel){
			//	test.x = (test.x / len) * simParams->maxVel;
			//	test.y = (test.y / len) * simParams->maxVel;
			//	test.z = (test.z / len) * simParams->maxVel;
			//}

			if ((test1) > (sumAllSHX * 180 * 2  + angle) && sumAllSHX > 0.1f){
				pX = posOther;
				test1 = sumAllSHX * 180 + angle;
				shCor = normalize(pX - posOwn) * sumAllSHX * 1000;
			}

			//for (int i = 1; i < 5; i++){
			//	float4 distTest = posOther + test * i * dt * 5;
			//	float4 dTest = ((posOwn + i * dt * velOwn) - distTest);

			//	dotP = dot(dTest, velOwn);

			//	if (dotP < 0.0)
			//		factor = .00000f;

			//	if (length(dTest) < 5.0 * i)
			//		shCor += normalize(dTest)* 5 * factor / i;
			//}
			//shCor += (float4)(sumAllSHX, sumAllSHY, sumAllSHZ, 0.0f) * factor;
			//shCor += (float4)(sumAllSHY, -sumAllSHX, sumAllSHZ, 0.0f);
			//shCor += (float4)(sumAllSHX, sumAllSHZ, -sumAllSHY, 0.0f);
		}
	}

	//shCor = pX - posOwn;

	float len = length(shCor);

	if (len > simParams->maxVel){
		shCor.x = (shCor.x / len) * simParams->maxVel;
		shCor.y = (shCor.y / len) * simParams->maxVel;
		shCor.z = (shCor.z / len) * simParams->maxVel;
	}

	velOwn += shCor;
	velOwn /= 2;
	velOwn.w = 0.0f;

	len = length(velOwn);

	if (len > simParams->maxVel){
		velOwn.x = (velOwn.x / len) * simParams->maxVel;
		velOwn.y = (velOwn.y / len) * simParams->maxVel;
		velOwn.z = (velOwn.z / len) * simParams->maxVel;
	}

	//apply correction velocity dependend on boid cell position (border case)
	velOwn += velCor;
	posOwn.w = 1.0;

	vel_out[id] = velOwn;
	pos_out[id] = posOwn + velOwn * dt;
}

/*kernel to use the SH calculations on boids*/
__kernel void dontUseSH(__global float4* vel,
						__global float4* vel_out,
						__global uint* startIndex,
						__global uint* endIndex,
						__global float8* sh_evalX,
						__global float8* sh_evalY,
						__global float8* sh_evalZ,
						__constant simParams_t* simParams,
						__global float4* pos,
						__global float4* pos_out,
						__local float8* sh_eval_localX,
						__local float8* sh_eval_localY,
						__local float8* sh_eval_localZ,
						__global float* coef0X,
						__global float* coef0Y,
						__global float* coef0Z,
						__local float* sh_c0_localX,
						__local float* sh_c0_localY,
						__local float* sh_c0_localZ,
						float dt)
{
	uint id = get_global_id(0);

	float4 velOwn = vel[id];
	float4 posOwn = pos[id];
	posOwn.w = 0.0f;
	velOwn.w = 0.0f;

	int4 gridPos = getGridPos(posOwn, simParams);
	int cell = gridPos.x + (simParams->gridSize.x) * gridPos.z + (simParams->gridSize.z) * (simParams->gridSize.x) * gridPos.y;
	uint plane = simParams->gridSize.x * simParams->gridSize.z;
	float4 velCor = checkAndCorrectBoundariesWithPos(gridPos, simParams);

	//truncate velocity to max velocity
	//velOwn = clamp(velOwn, -mVel, mVel);
	//using the magnitude of the velocity is nicer than clamping it
	float len = length(velOwn);

	if(len > simParams->maxVel){
		velOwn.x = (velOwn.x / len) * simParams->maxVel;
		velOwn.y = (velOwn.y / len) * simParams->maxVel;
		velOwn.z = (velOwn.z / len) * simParams->maxVel;
	}
			
	//apply correction velocity dependend on boid cell position (border case)
	velOwn += velCor;

	vel_out[id] = velOwn;
	posOwn.w = 1.0f;
	pos_out[id] = posOwn + velOwn * dt;	
}


/* unused less efficient simulation evaluation kernel

__kernel void simulate2(__global const float4* pos,
	__global float4* pos_out,
	__global float4* vel,
	__global float4* vel_out,
	__global const uint *cellStart,
	__global const uint *cellEnd,
	__local float4 *localPos,
	__local float4 *localVel,
	__global const float4 *goal,
	__constant simParams_t* simParams,
	__global uint *range_out,
	float dt)
{
	uint id = get_local_id(0);
	uint cell = get_group_id(0);

	uint cellPosTest = cell;

	float4 velOwn = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 posOwn = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 perceivedPos = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 perceivedVel = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 separation = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
	float4 distance = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);	//maximum velocity

	uint startIndex = cellStart[cell];
	uint endIndex = cellEnd[cell];
	uint range = endIndex - startIndex;

	int flockMatesVisible = 0;

	float angle;

	if (get_local_id(0) == 0)
		range_out[cell] = range;

	//test if cell is empty
	if (range == 0)
		return;

	//get correction velocity
	float4 velCor = checkAndCorrectBoundaries(cell, simParams);

	//case if all boids in cell fit into local memory
	if (range <= get_local_size(0)){
		uint cellPos = id + startIndex;

		//check if number of boids in cell smaller than local work size
		if (cellPos < endIndex){
			localPos[id] = pos[cellPos];	//load vel and pos into local memory
			localVel[id] = vel[cellPos];

			barrier(CLK_LOCAL_MEM_FENCE);

			//compare self with all other boids in cell
			for (int i = 1; i < range; i++){
				cellPos = (id + i) % range;

				//other boids velocity
				distance = localPos[cellPos] - localPos[id];		//distance vector to other boid
				distance.w = 0.0f;

				float dotP = dot(-localVel[id], distance);
				float lenV = length(localVel[id]);
				float lenD = length(distance);
				angle = dotP / (lenV * lenD);						//calculate acute angle between self and other boid

				if (dotP < 0.f || fabs(degrees(acos(angle))) > 45){	//check if other boid is visible, dot product indicates that angle is > 90°

					flockMatesVisible++;							//increase counter how many boids are visible
					perceivedPos += localPos[cellPos];				//add other boids position to perceived center of mass
					perceivedVel += localVel[cellPos];				//add other boids velocity to perceived aligned velocity

					if (length(distance) < 2.5f)						//check if other boid is near enough to separate
						separation -= distance;

				}
			}

			//if other boids are visible calculate perceived steering to center of mass / alignment of velocities
			if (flockMatesVisible >= 1){
				perceivedPos = (perceivedPos / flockMatesVisible) - localPos[id];
				perceivedVel = (perceivedVel / flockMatesVisible) - localVel[id];
			}

			//calculate straight path to goal
			float4 g = goal[startIndex + id];
			g.w = 0.0f;

			float4 path = g - localPos[id];
			path.w = 0.0f;
			path = fast_normalize(path);

			//calculate new velocities 
			velOwn = localVel[id] * simParams->wOwn + path * simParams->wPath + perceivedPos * simParams->wCohesion + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;

			//write back velocity and new position
			vel_out[startIndex + id] = velOwn;
			pos_out[startIndex + id] = localPos[id];
		}
		//else case: If more boids are in cell than can fit into local memory
	}
	else {
		//start with dividing the number of boids in cell into chunks of local_size()
		for (int i = 0; i < ((range / get_local_size(0)) + 1); i++){
			uint cellPos;
			int rRange = range % get_local_size(0);			//remaining boids when number of boids not a number of local_size()

			//interaction with other boids. Divided into same chunk size to load them into local memory
			for (int x = 0; x < ((range / get_local_size(0)) + 1); x++){
				cellPos = id + startIndex + (((i + x) % (range / get_local_size(0) + 1)) * get_local_size(0));	//global memory position from where the vel/pos should be loaded

				if (cellPos < endIndex){		//check if cellPos is outside own cell

					localVel[id] = vel[cellPos];	//load velocity into local memory
					localPos[id] = pos[cellPos];	//load position into local memory

					if (x == 0){
						velOwn = localVel[id];		//every time x is 0 means we are using calculating the interaction for a new boid 
						posOwn = localPos[id];		//could also be velOwn/posOwn
					}


					barrier(CLK_LOCAL_MEM_FENCE);	//continue when everything is in local memory	

					/*Here are two different distinctive cases. First case is when a chunk is processed and the size of local_size()
					Second case is the border case when we have the remainder of boids in cell % local_size(). This is necessary because
					otherwise we would access vel/pos data from other cells or outside the declared buffer*/
/*					if ((i + x) != (range / get_local_size(0))){
						//iterate over all boids in local memory, skip self (j starts as 1)
						for (uint j = 1; j < get_local_size(0); j++){
							cellPos = (id + j) % get_local_size(0);

							//same calculations as case numBoids < localMem
							distance = localPos[cellPos] - posOwn;
							distance.w = 0.0f;

							float dotP = dot(-velOwn, distance);
							float lenV = length(localVel[id]);
							float lenD = length(distance);
							angle = dotP / (lenV * lenD);

							if (dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if (length(distance) < 2.5f)
									separation -= distance;

							}
						}
					}
					else {
						//case where local memory is bigger than remaining boids in cell
						for (uint j = 1; j < rRange; j++){
							cellPos = (id + j) % rRange;

							distance = localPos[cellPos] - posOwn;
							distance.w = 0.0f;

							float dotP = dot(-velOwn, distance);
							float lenV = length(localVel[id]);
							float lenD = length(distance);
							angle = dotP / (lenV * lenD);

							if (dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if (length(distance) < 2.5f)
									separation -= distance;

							}
						}
					}//end if case last chunk	
				}//end if case cellPos < endIndex
			}//end inner for loop

			//calculate perceived position and velocity
			if (flockMatesVisible >= 1){
				perceivedPos = (perceivedPos / flockMatesVisible) - posOwn;
				perceivedVel = (perceivedVel / flockMatesVisible) - velOwn;
			}

			float4 g = goal[startIndex + id];
			g.w = 0.0f;

			float4 path = g - posOwn;
			path.w = 0.0f;
			path = fast_normalize(path);

			//apply steering to velocity
			velOwn = velOwn * simParams->wOwn + path * simParams->wPath + perceivedPos * simParams->wCohesion + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;

			cellPos = startIndex + id + i * get_local_size(0);

			//write back to output, but check if it is not outside own cell (could be possible in border case)
			if (cellPos < endIndex){
				vel_out[cellPos] = velOwn;
				pos_out[cellPos] = posOwn;
			}

			perceivedPos = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
			perceivedVel = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
			separation = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
			flockMatesVisible = 0;

		}//end outer for loop
	} //end else
}
*/