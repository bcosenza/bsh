/*
	As boidModelGrid_kernel except it uses localMem bigger than the local group size.
	Using fast methods instead of normal ones.
*/
#define FACTOR_NO 0.0001f
#define FACTOR .006f
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
    if(index < numParticles){
        hash = gridHash[index];

        //Load hash data into local memory so that we can look 
        //at neighboring particle's hash value without loading
        //two hash values per thread
        localHash[get_local_id(0) + 1] = hash;

        //First thread in block must load neighbor particle hash
        if(index > 0 && get_local_id(0) == 0)
            localHash[0] = gridHash[index - 1];
    }

    barrier(CLK_LOCAL_MEM_FENCE);

    if(index < numParticles){
        //Border case
        if(index == 0)
            cellStart[hash] = 0;

        //Main case
        else{
            if(hash != localHash[get_local_id(0)])
                cellEnd[localHash[get_local_id(0)]]  = cellStart[hash] = index;
        };

        //Another border case
        if(index == numParticles - 1)
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


/*
	Evaluate 3. order SH. Numerical implementation from P.P. Sloan
	float8 is used to store the 9 coefficients because coefficient 0 is
	constant it is just added at the reconstruction.
*/
float8 SHEval3(float4 vel_in)
{
   float4 vel = vel_in;
   float8 pSH;
   float fC0,fC1,fS0,fS1,fTmpA,fTmpB,fTmpC;
   float fZ2 = vel.z*vel.z;

   //pSH.0 = 0.2820947917738781f;
   pSH.s1 = 0.4886025119029199f*vel.z;
   pSH.s5 = 0.9461746957575601f*fZ2 + -0.3153915652525201f;
   fC0 = vel.x;
   fS0 = vel.y;

   fTmpA = -0.48860251190292f;
   pSH.s2 = fTmpA*fC0;
   pSH.s0 = fTmpA*fS0;
   fTmpB = -1.092548430592079f*vel.z;
   pSH.s6 = fTmpB*fC0;
   pSH.s4 = fTmpB*fS0;
   fC1 = vel.x*fC0 - vel.y*fS0;
   fS1 = vel.x*fS0 + vel.y*fC0;

   fTmpC = 0.5462742152960395f;
   pSH.s7 = fTmpC*fC1;
   pSH.s3 = fTmpC*fS1;

   return pSH;
}

/*simple reduction kernel to sum up the velocities of all boids in a cell*/
__kernel void evalSH(__global float4* vel,
					 __global float8* sh_evalX, 
					 __global float8* sh_evalY, 
					 __global float8* sh_evalZ, 
					 __global float* coef0X, 
					 __global float* coef0Y, 
					 __global float* coef0Z){
	uint id = get_global_id(0);
	float4 v = vel[id];
	v.w = 0.0f;
	float8 sh = SHEval3(fast_normalize(v));
	sh_evalX[id] = sh * v.x;
	sh_evalY[id] = sh * v.y;
	sh_evalZ[id] = sh * v.z;

	coef0X[id] = 0.2820947917738781f * v.x;
	coef0Y[id] = 0.2820947917738781f * v.y;
	coef0Z[id] = 0.2820947917738781f * v.z;
}

/*kernel to use the SH calculations on boids*/
__kernel void useSH(__global const float4* vel,
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
					__local float4* lPos,
					__local float4* lVel,
					 float dt)
{
	uint id = get_global_id(0);
	uint lSize = get_local_size(0);
	uint lId = get_local_id(0);

	float8 sumSHX;
	float8 sumSHY;
	float8 sumSHZ;

	float4 velOwn = vel[id];
	float4 posOwn = pos[id];
	float8 SHSelf = SHEval3(normalize(velOwn));
	posOwn.w = 0.0f;
	velOwn.w = 0.0f;

	int4 gridPos = getGridPos(posOwn, simParams);
	int cell = gridPos.x + (simParams->gridSize.x) * gridPos.z + (simParams->gridSize.z) * (simParams->gridSize.x) * gridPos.y;
	float4 velCor = checkAndCorrectBoundaries(cell, simParams);

	sh_eval_localX[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[lId] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[lId] = 0.0f;
	sh_c0_localY[lId] = 0.0f;
	sh_c0_localZ[lId] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);
	int test = 0;

	for(uint i = 0; i < (simParams->numBodies / lSize); i++){

		sh_eval_localX[lId] = sh_evalX[lSize * i + lId];
		sh_eval_localY[lId] = sh_evalY[lSize * i + lId];
		sh_eval_localZ[lId] = sh_evalZ[lSize * i + lId];
		sh_c0_localX[lId] = coef0X[lSize * i + lId];
		sh_c0_localY[lId] = coef0Y[lSize * i + lId];
		sh_c0_localZ[lId] = coef0Z[lSize * i + lId];
		lVel[lId] = vel[lSize * i + lId];
		lPos[lId] = pos[lSize * i + lId];
		
		barrier(CLK_LOCAL_MEM_FENCE);
		
		for (int j = 0; j < lSize; j++){
			float4 p = lPos[j];
			float4 v = lVel[j];
			p.w = 0.0f;
			v.w = 0.0f;

			float4 distV = posOwn - p;
			float dist = fast_distance(p, posOwn);

			//float lenOther = fast_length(v);

			float factor = FACTOR_NO;
			float dotP = dot(distV, velOwn);

			SHSelf = SHEval3(normalize(distV));

			if (dotP < 0.0){
				factor = FACTOR;
			}

			float8 SHOtherX = (1.f / (dist + 0.01)) * sh_eval_localX[j];
			float8 SHOtherY = (1.f / (dist + 0.01)) * sh_eval_localY[j];
			float8 SHOtherZ = (1.f / (dist + 0.01)) * sh_eval_localZ[j];

			float sumAllSHX = (1.f / (dist + 0.01)) * sh_c0_localX[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHY = (1.f / (dist + 0.01)) * sh_c0_localY[j] * 0.2820947917738781f * M_PI_F;
			float sumAllSHZ = (1.f / (dist + 0.01)) * sh_c0_localZ[j] * 0.2820947917738781f * M_PI_F;

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

			float4 shCor = (float4)(-sumAllSHZ, sumAllSHY, sumAllSHX, 0.0f);
			shCor += (float4)(sumAllSHY, -sumAllSHX, sumAllSHZ, 0.0f);
			shCor += (float4)(sumAllSHX, sumAllSHZ, -sumAllSHY, 0.0f);
				
			velOwn += shCor * factor;//(1.f / (simParams->numBodies - range) );
			velOwn.w = 0.0f;
		}
	}

	float len = length(velOwn);

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
					__global float8* sh_eval, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					__local float8* sh_eval_local,
					 float dt)
{
	uint id = get_local_id(0);
	uint lSize = get_local_size(0);
	uint cell = get_group_id(0);

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	
	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint index = start + id;

	while(index < end){
		float4 velOwn = vel[index];
		velOwn.w = 0.0f;

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

		vel_out[index] = velOwn;
		pos_out[index] = pos[index] + velOwn * dt;

		index += lSize;
	}
	
}


/*kernel to use the SH calculations on boids*/
/*
__kernel void useSH(__global float4* vel,
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
	uint id = get_local_id(0);
	uint lSize = get_local_size(0);
	uint cell = get_group_id(0);

	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint range = end - start;

	if (range == 0)
		return;

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);


	uint plane = simParams->gridSize.x * simParams->gridSize.z;

	float8 sumSHX;
	float8 sumSHY;
	float8 sumSHZ;

	float4 velOwn;
	float4 posOwn = pos[start];
	posOwn.w = 0.0f;

	sh_eval_localX[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[id] = 0.0f;
	sh_c0_localY[id] = 0.0f;
	sh_c0_localZ[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for (uint i = id; i < (simParams->numCells - range); i += lSize){
		uint index = (i + end) % simParams->numCells;

		float4 p = pos[index];
		float4 v = vel[index];
		p.w = 0.0f;
		v.w = 0.0f;

		float4 distV = posOwn - p;
		float dist = fast_distance(p, posOwn);//fabs(fast_length(distV));
		float lenOther = fast_length(v);

		float factor = 1.0f;
		float dotP = dot(distV, v);
		float angle = dotP / (dist * lenOther);

		//if(45.f > degrees(acos(angle)))
		//	factor = -.01f;

		sh_eval_localX[id] += (factor / dist) * sh_evalX[index];
		sh_eval_localY[id] += (factor / dist) * sh_evalY[index];
		sh_eval_localZ[id] += (factor / dist) * sh_evalZ[index];
		sh_c0_localX[id] += (factor / dist) * coef0X[index];
		sh_c0_localY[id] += (factor / dist) * coef0Y[index];
		sh_c0_localZ[id] += (factor / dist) * coef0Z[index];
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
	}
	barrier(CLK_LOCAL_MEM_FENCE);

	uint index = start + id;
	while (index < end){
		velOwn = vel[index];
		posOwn = pos[index];
		posOwn.w = 0.0f;
		velOwn.w = 0.0f;

		sumSHX = sh_eval_localX[0];
		sumSHY = sh_eval_localY[0];
		sumSHZ = sh_eval_localZ[0];


		/*for(uint i = 1; i < (range - 1); i++){
		uint index2 = start + (i + id) % range;
		float4 p = pos[index2];
		p.w = 0.0;
		float dist = distance(p, posOwn);

		sumSHX += (1.f / (1.f)) * sh_evalX[index];
		sumSHY += (1.f / (1.f)) * sh_evalY[index];
		sumSHZ += (1.f / (1.f)) * sh_evalZ[index];
		}*//*

		float8 SHSelf = SHEval3(normalize(velOwn));

		float8 SHOtherX = sumSHX;
		float8 SHOtherY = sumSHY;
		float8 SHOtherZ = sumSHZ;

		float sumAllSHX = sh_c0_localX[0] * 0.2820947917738781f;
		float sumAllSHY = sh_c0_localY[0] * 0.2820947917738781f;
		float sumAllSHZ = sh_c0_localZ[0] * 0.2820947917738781f;

		sumAllSHX += SHSelf.s0 * SHOtherX.s0;
		sumAllSHX += SHSelf.s1 * SHOtherX.s1;
		sumAllSHX += SHSelf.s2 * SHOtherX.s2;
		sumAllSHX += SHSelf.s3 * SHOtherX.s3 * 2;
		sumAllSHX += SHSelf.s4 * SHOtherX.s4 * 2;
		sumAllSHX += SHSelf.s5 * SHOtherX.s5 * 2;
		sumAllSHX += SHSelf.s6 * SHOtherX.s6 * 2;
		sumAllSHX += SHSelf.s7 * SHOtherX.s7 * 2;

		sumAllSHY += SHSelf.s0 * SHOtherY.s0;
		sumAllSHY += SHSelf.s1 * SHOtherY.s1;
		sumAllSHY += SHSelf.s2 * SHOtherY.s2;
		sumAllSHY += SHSelf.s3 * SHOtherY.s3 * 2;
		sumAllSHY += SHSelf.s4 * SHOtherY.s4 * 2;
		sumAllSHY += SHSelf.s5 * SHOtherY.s5 * 2;
		sumAllSHY += SHSelf.s6 * SHOtherY.s6 * 2;
		sumAllSHY += SHSelf.s7 * SHOtherY.s7 * 2;

		sumAllSHZ += SHSelf.s0 * SHOtherZ.s0;
		sumAllSHZ += SHSelf.s1 * SHOtherZ.s1;
		sumAllSHZ += SHSelf.s2 * SHOtherZ.s2;
		sumAllSHZ += SHSelf.s3 * SHOtherZ.s3 * 2;
		sumAllSHZ += SHSelf.s4 * SHOtherZ.s4 * 2;
		sumAllSHZ += SHSelf.s5 * SHOtherZ.s5 * 2;
		sumAllSHZ += SHSelf.s6 * SHOtherZ.s6 * 2;
		sumAllSHZ += SHSelf.s7 * SHOtherZ.s7 * 2;

		float4 shCor = (float4)(sumAllSHX, sumAllSHY, sumAllSHZ, 0.0f);
		float len = length(shCor);

		if (len > simParams->maxVel){
			shCor.x = (shCor.x / len) * simParams->maxVel;
			shCor.y = (shCor.y / len) * simParams->maxVel;
			shCor.z = (shCor.z / len) * simParams->maxVel;
		}

		len = length(velOwn);

		if (len > simParams->maxVel){
			velOwn.x = (velOwn.x / len) * simParams->maxVel;
			velOwn.y = (velOwn.y / len) * simParams->maxVel;
			velOwn.z = (velOwn.z / len) * simParams->maxVel;
		}

		float lenOwn = fast_length(shCor);
		float lenOther = fast_length(velOwn);

		float factor = 0.1f;
		float dotP = dot(shCor, velOwn);
		float angle = dotP / (lenOwn * lenOther);

		if (25.f < degrees(acos(angle)))
			factor = 1.05f;


		velOwn += shCor * factor;//(1.f / (simParams->numBodies - range) );
		velOwn.w = 0.0f;

		//truncate velocity to max velocity
		//velOwn = clamp(velOwn, -mVel, mVel);
		//using the magnitude of the velocity is nicer than clamping it
		len = length(velOwn);

		if (len > simParams->maxVel){
			velOwn.x = (velOwn.x / len) * simParams->maxVel;
			velOwn.y = (velOwn.y / len) * simParams->maxVel;
			velOwn.z = (velOwn.z / len) * simParams->maxVel;
		}

		//apply correction velocity dependend on boid cell position (border case)
		velOwn += velCor;

		vel_out[index] = velOwn;
		pos_out[index] = pos[index] + velOwn * dt;

		index += lSize;
	}
}
*/