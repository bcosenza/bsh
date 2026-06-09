/*
	Generally the same as boidModelGrid_kernel except it uses localMem bigger than the local group size.
	Using fast methods instead of normal ones.
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

        reorderedPos[index] = pos;
        reorderedVel[index] = vel;
	}  
}



__kernel void simulate( __global float4* pos,
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
	uint id = get_local_id(0);
	uint cell = get_group_id(0);

	uint cellPosTest = cell;
	
	float4 velOwn = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 posOwn = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 perceivedPos = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 perceivedVel = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 separation = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 distance = (float4)(0.0f,0.0f,0.0f,0.0f);

	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);	//maximum velocity

	uint startIndex = cellStart[cell];
	uint endIndex = cellEnd[cell];
	uint range = endIndex - startIndex;

	int flockMatesVisible = 0;

	float angle;

	if(get_local_id(0) == 0)
		range_out[cell] = range;

	//test if cell is empty
	if(range == 0)
		return;

	//get correction velocity
	float4 velCor = checkAndCorrectBoundaries(cell, simParams);

	//case if all boids in cell fit into local memory
	if(range <= get_local_size(0)){
		uint cellPos = id + startIndex;

		//check if number of boids in cell smaller than local work size
		if(cellPos < endIndex){
			localPos[id] = pos[cellPos];	//load vel and pos into local memory
			localVel[id] = vel[cellPos];
		
			barrier(CLK_LOCAL_MEM_FENCE);

			//compare self with all other boids in cell
			for(int i = 1; i < range; i++){
				cellPos = (id + i) % range;

				//other boids velocity
				distance = localPos[cellPos] - localPos[id];		//distance vector to other boid
				distance.w = 0.0f;

				float dotP = dot(-localVel[id], distance);
				float lenV = length(localVel[id]);
				float lenD = length(distance);
				angle = dotP / (lenV * lenD);						//calculate acute angle between self and other boid
				
				if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){	//check if other boid is visible, dot product indicates that angle is > 90°

					flockMatesVisible++;							//increase counter how many boids are visible
					perceivedPos += localPos[cellPos];				//add other boids position to perceived center of mass
					perceivedVel += localVel[cellPos];				//add other boids velocity to perceived aligned velocity
				
					if(length(distance) < 2.5f)						//check if other boid is near enough to separate
						separation -= distance;
					
				}
			}

			//if other boids are visible calculate perceived steering to center of mass / alignment of velocities
			if(flockMatesVisible >= 1){								
				perceivedPos = (perceivedPos / flockMatesVisible) - localPos[id];
				perceivedVel = (perceivedVel / flockMatesVisible) - localVel[id];
			}
				
			//calculate new velocities 
			velOwn = localVel[id] * simParams->wOwn + perceivedPos * simParams->wCohesion  + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
			
			//write back velocity and new position
			vel_out[startIndex + id] = velOwn;
			pos_out[startIndex + id] = localPos[id];
		}
	//else case: If more boids are in cell than can fit into local memory
	} else {
		//start with dividing the number of boids in cell into chunks of local_size()
		for(int i = 0; i < ((range/get_local_size(0)) + 1); i++){
			uint cellPos;
			int rRange = range % get_local_size(0);			//remaining boids when number of boids not a number of local_size()
		
			//interaction with other boids. Divided into same chunk size to load them into local memory
			for( int x = 0; x < ((range/get_local_size(0)) + 1); x++ ){
				cellPos = id + startIndex + (((i + x) % (range/get_local_size(0) + 1)) * get_local_size(0));	//global memory position from where the vel/pos should be loaded

				if(cellPos < endIndex){		//check if cellPos is outside own cell
					
					localVel[id] = vel[cellPos];	//load velocity into local memory
					localPos[id] = pos[cellPos];	//load position into local memory

					if(x == 0){
						velOwn = localVel[id];		//every time x is 0 means we are using calculating the interaction for a new boid 
						posOwn = localPos[id];		//could also be velOwn/posOwn
					}
					
				
					barrier(CLK_LOCAL_MEM_FENCE);	//continue when everything is in local memory	
					
					/*Here are two different distinctive cases. First case is when a chunk is processed and the size of local_size()
					  Second case is the border case when we have the remainder of boids in cell % local_size(). This is necessary because
					  otherwise we would access vel/pos data from other cells or outside the declared buffer*/
					if((i + x) != (range/get_local_size(0))){
						//iterate over all boids in local memory, skip self (j starts as 1)
						for(uint j = 1; j < get_local_size(0); j++){
							cellPos = (id + j) % get_local_size(0);

							//same calculations as case numBoids < localMem
							distance = localPos[cellPos] - posOwn;
							distance.w = 0.0f;

							float dotP = dot(-velOwn, distance);
							float lenV = length(localVel[id]);
							float lenD = length(distance);
							angle = dotP / (lenV * lenD);	
				
							if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(length(distance) < 2.5f)
									separation -= distance;
								
							}
						}
					} else {
						//case where local memory is bigger than remaining boids in cell
						for(uint j = 1; j < rRange; j++){
							cellPos = (id + j) % rRange;

							distance = localPos[cellPos] - posOwn;
							distance.w = 0.0f;

							float dotP = dot(-velOwn, distance);
							float lenV = length(localVel[id]);
							float lenD = length(distance);
							angle = dotP / (lenV * lenD);	
				
							if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(length(distance) < 2.5f)
									separation -= distance;
								
							}
						}
					}//end if case last chunk	
				}//end if case cellPos < endIndex
			}//end inner for loop

			//calculate perceived position and velocity
			if(flockMatesVisible >= 1){
				perceivedPos = (perceivedPos / flockMatesVisible) - posOwn;
				perceivedVel = (perceivedVel / flockMatesVisible) - velOwn;
			}
				
			//apply steering to velocity
			velOwn =  velOwn * simParams->wOwn + perceivedPos * simParams->wCohesion  + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
			
			cellPos = startIndex + id + i * get_local_size(0);

			//write back to output, but check if it is not outside own cell (could be possible in border case)
			if(cellPos < endIndex){
				vel_out[cellPos] = velOwn;
				pos_out[cellPos] = posOwn;
			}

			perceivedPos = (float4)(0.0f,0.0f,0.0f,0.0f);
			perceivedVel = (float4)(0.0f,0.0f,0.0f,0.0f);
			separation = (float4)(0.0f,0.0f,0.0f,0.0f);
			flockMatesVisible = 0;
			
		}//end outer for loop
	} //end else
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
__kernel void sumVelSH(__global float4* vel, __global uint* startIndex, __global uint* endIndex, __global float4* vel_sum, __local float4* sumArray){
	uint id = get_local_id(0);
	uint cell = get_group_id(0);
	uint lSize = get_local_size(0);

	uint cellPosTest = cell;

	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint index = start + id;

	sh_eval_localX[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localY[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_eval_localZ[id] = (float8)(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	sh_c0_localX[id] = 0.0f;
	sh_c0_localY[id] = 0.0f;
	sh_c0_localZ[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	while(index < end){
		float4 v = vel[index];
		v.w = 0.0f;

		float8 sh = SHEval3(fast_normalize(v));
		sh_evalX[id] += sh * v.x;
		sh_evalY[id] += sh * v.y;
		sh_evalZ[id] += sh * v.z;

		coef0X[id] += 0.2820947917738781f * v.x;
		coef0Y[id] += 0.2820947917738781f * v.y;
		coef0Z[id] += 0.2820947917738781f * v.z;

		index += lSize ;
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

	while(id < range){
		sumArray[id] += sumArray[id + range];
		range = range / 2;
	}

	if(id == 0){
		if(range > 0)
			vel_sum[cell] = sumArray[id] ;
		else
			vel_sum[cell] = sumArray[id];
	}

}

/*kernel to use the SH calculations on boids*/
__kernel void useSH(__global float4* vel,
					__global float4* vel_out,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					__local float4* shSum,
					 float dt)
{
	uint id = get_local_id(0);
	uint lSize = get_local_size(0);
	uint cell = get_group_id(0);
	
	uint start = startIndex[cell];
	uint end = endIndex[cell];

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);
	const float4 mVel2 = (float4)(2.5f, 2.5f, 0, 0);

	uint range = end - start;
	uint plane = simParams->gridSize.x * simParams->gridSize.z;

	float4 shVelSum;

	float4 velOwn = vel_sum[cell];
	float8 SHSelf = SHEval3(normalize(velOwn));
	float8 SHOther;
	float sumSH;

	float3 posOwn = (float3)((cell / plane), ((cell % plane) / simParams->gridSize.x), ((cell % plane) % simParams->gridSize.x));
	float3 posOther;

	shSum[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for(uint i = id; i < simParams->numCells; i += lSize){
		if(i != cell){
			posOther = (float3)(((i) / plane), (((i) % plane) / simParams->gridSize.x), (((i) % plane) % simParams->gridSize.x));
			
			
			float4 velOther = vel_sum[i];
			float4 dist = (float4)(posOther - posOwn, 0.0f);
			
			float factor = .0001f;
			float lenOwn = length(velOwn);
			float lenOther = length(velOther);
			float dotP = dot(dist, -velOwn);
			float angle = dotP / (lenOwn * lenOther);

			if(dotP < 0.0f)
				factor = 0.01;

			SHOther = SHEval3(normalize(velOther));	

			sumSH = 0.2820947917738781f * 0.2820947917738781f;

			sumSH += SHSelf.s0 * SHOther.s0;
			sumSH += SHSelf.s1 * SHOther.s1;
			sumSH += SHSelf.s2 * SHOther.s2;
			sumSH += SHSelf.s3 * SHOther.s3;
			sumSH += SHSelf.s4 * SHOther.s4;
			sumSH += SHSelf.s5 * SHOther.s5;
			sumSH += SHSelf.s6 * SHOther.s6;
			sumSH += SHSelf.s7 * SHOther.s7;

			
			float fu = fast_distance(posOwn, posOther);
			shVelSum += (sumSH * factor) / ( fabs(fu) * fabs(fu)) * velOther;

			//shVelSum += (sumSH * 0.01) / ( fabs(fu) * fabs(fu)) * velOther;
			///shVelSum += (sumSH * 0.01) / ( fabs(fu)) * velOther;
			//shVelSum += ((sumSH * 0.01) * length(velOther)  / ( fabs(fu))) * normalize(velOwn);
				
		}
	}

	//shVelSum /= M_PI_F;

	//shVelSum = (shVelSum < -mVel) ? -mVel : shVelSum;
	//shVelSum = (shVelSum > mVel) ? mVel : shVelSum;

	shSum[id] = shVelSum;
	barrier(CLK_LOCAL_MEM_FENCE);

	uint k = lSize / 2;
	while(id < k){
		shSum[id] += shSum[id + k];
		k = k / 2;
	}
	

	uint index = start + id;
	while(index < end){
		velOwn = shSum[0];

		//truncate velocity to max velocity
		//velOwn = clamp(velOwn, -mVel2, mVel2);
		//using the magnitude of the velocity is nicer than clamping it
		velOwn.w = 0.0f;

		float len = length(velOwn);
		
		if(len > simParams->maxVel){
			velOwn.x = (velOwn.x / len) * (simParams->maxVel);
			velOwn.y = (velOwn.y / len) * simParams->maxVel;
			velOwn.z = (velOwn.z / len) * (simParams->maxVel);
		}

		float4 velOwn2 = vel[index];
		velOwn2.w = 0.0f;

		len = length(velOwn2);

		/*if(len > simParams->maxVel){
			velOwn2.x = (velOwn2.x / len) * simParams->maxVel;
			velOwn2.y = (velOwn.y / len) * simParams->maxVel;
			velOwn2.z = (velOwn2.z / len) * simParams->maxVel;
		}*/

		velOwn += velOwn2;
		velOwn.w = 0.0f;

		//truncate velocity to max velocity
		//velOwn = clamp(velOwn, -mVel, mVel);
		//using the magnitude of the velocity is nicer than clamping it
		len = length(velOwn);

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


//###############################
//all following kernels are tests 
//###############################

/*__kernel void useSH(__global float4* vel,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					__local float4* shSum,
					 float dt){
	uint id = get_local_id(0);
	uint lSize = get_local_size(0);
	uint cell = get_group_id(0);
	
	uint start = startIndex[cell];
	uint end = endIndex[cell];

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);

	uint range = end - start;
	uint plane = simParams->gridSize.x * simParams->gridSize.y;

	float4 shVelSum;

	float4 velOwn = vel_sum[cell];
	float8 SHSelf = SHEval3(normalize(velOwn));
	float8 SHOther;
	float sumSH;

	float3 posOwn = (float3)(cell / plane, (cell % plane) / simParams->gridSize.x, ((cell % plane) % simParams->gridSize.x));
	float3 posOther;

	shSum[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for(uint i = id; i < simParams->numCells; i += lSize){
		if(i != cell){
			float4 velOther = vel_sum[i];
			SHOther = SHEval3(normalize(velOther));	

			sumSH = 0.0f;//0.2820947917738781f * 0.2820947917738781f * M_PI_F;

			sumSH += SHSelf.s0 * SHOther.s0 * 2.0f/3.0f;
			sumSH += SHSelf.s1 * SHOther.s1 * 2.0f * M_PI_F/3.0f;
			sumSH += SHSelf.s2 * SHOther.s2 * 2.0f * M_PI_F/3.0f;
			sumSH += SHSelf.s3 * SHOther.s3 * M_PI_F/4.0f;
			sumSH += SHSelf.s4 * SHOther.s4 * M_PI_F/4.0f;
			sumSH += SHSelf.s5 * SHOther.s5 * M_PI_F/4.0f;
			sumSH += SHSelf.s6 * SHOther.s6 * M_PI_F/4.0f;
			sumSH += SHSelf.s7 * SHOther.s7 * M_PI_F/4.0f;

			posOther = (float3)((i) / plane, ((i) % plane) / simParams->gridSize.x, (((i) % plane) % simParams->gridSize.x));
			float fu = fast_distance(posOwn, posOther);
			shVelSum += ((sumSH * length(velOther) / 9000) / (fabs(fu) * fabs(fu) * fabs(fu) )) * velOther * M_PI_F;
			}
	}

	shVelSum /= M_PI_F;

	//shVelSum = (shVelSum < -mVel) ? -mVel : shVelSum;
	//shVelSum = (shVelSum > mVel) ? mVel : shVelSum;

	shSum[id] = shVelSum;
	barrier(CLK_LOCAL_MEM_FENCE);

	uint index = start + id;
	while(index < end){
		velOwn = (float4)(0.0f, 0.0f, 0.0f, 0.0f);
		
		for(uint j = 0; j < lSize; j++){
			velOwn += shSum[j];
		}


		velOwn = vel[index];

		velOwn = (velOwn < -mVel) ? -mVel : velOwn;
		velOwn = (velOwn > mVel) ? mVel : velOwn;
			
		velOwn += velCor;

		vel[index] = velOwn;
		pos_out[index] = pos[index] + velOwn * dt;

		index += lSize;
	}
}*/


/*
__kernel void useSH(__global float4* vel,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					__local float4* shSum,
					 float dt){
	uint id = get_local_id(0);
	uint lSize = get_local_size(0);
	uint cell = get_group_id(0);
	
	uint start = startIndex[cell];
	uint end = endIndex[cell];

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);

	uint range = end - start;
	uint plane = simParams->gridSize.x * simParams->gridSize.y;

	float4 shVelSum;

	float4 velOwn = vel_sum[cell];
	float8 SHSelf = SHEval3(normalize(velOwn));
	float8 SHOther;
	float sumSH;

	float3 posOwn = (float3)(cell / plane, (cell % plane) / simParams->gridSize.x, ((cell % plane) % simParams->gridSize.x));
	float3 posOther;

	shSum[id] = 0.0f;
	barrier(CLK_LOCAL_MEM_FENCE);

	for(uint i = id; i < simParams->numCells; i += lSize){
		if(i != cell){
			float4 velOther = vel_sum[i];
			SHOther = SHEval3(normalize(velOther));	

			sumSH = 0.2820947917738781f * 0.2820947917738781f;

			sumSH += SHSelf.s0 * SHOther.s0;
			sumSH += SHSelf.s1 * SHOther.s1;
			sumSH += SHSelf.s2 * SHOther.s2;
			sumSH += SHSelf.s3 * SHOther.s3;
			sumSH += SHSelf.s4 * SHOther.s4;
			sumSH += SHSelf.s5 * SHOther.s5;
			sumSH += SHSelf.s6 * SHOther.s6;
			sumSH += SHSelf.s7 * SHOther.s7;

			posOther = (float3)((i) / plane, ((i) % plane) / simParams->gridSize.x, (((i) % plane) % simParams->gridSize.x));
			float fu = fast_distance(posOwn, posOther);
			shVelSum += ((sumSH * length(velOther)) / (fu)) * velOther;
			}
	}

	shVelSum /= 30;

	//shVelSum = (shVelSum < -mVel) ? -mVel : shVelSum;
	//shVelSum = (shVelSum > mVel) ? mVel : shVelSum;

	shSum[id] = shVelSum;
	barrier(CLK_LOCAL_MEM_FENCE);

	uint index = start + id;
	while(index < end){
		velOwn = vel[index];
		
		for(uint j = 0; j < lSize; j++){
			velOwn += shSum[j];
		}


		velOwn = (velOwn < -mVel) ? -mVel : velOwn;
		velOwn = (velOwn > mVel) ? mVel : velOwn;
			
		velOwn += velCor;

		vel[index] = velOwn;
		pos_out[index] = pos[index] + velOwn * dt;

		index += lSize;
	}
}
*/

/*
__kernel void useSH(__global float4* vel,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					 float dt){
	uint id = get_local_id(0);
	uint cell = get_group_id(0);
	
	uint start = startIndex[cell];
	uint end = endIndex[cell];

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);

	uint range = end - start;
	uint plane = simParams->gridSize.x * simParams->gridSize.y;

	__local float4 shSum[LOCAL_SUM];
	float4 shVelSum;

	float8 SHSelf = SHEval3(vel_sum[cell]);
	float8 SHOther;
	float sumSH;

	float3 posOwn = (float3)(cell / plane, (cell % plane) / simParams->gridSize.x, ((cell % plane) % simParams->gridSize.x));
	float3 posOther;

	shSum[id] = 0;
	barrier(CLK_LOCAL_MEM_FENCE);

	for(int i = id; i < simParams->numCells; i += LOCAL_SUM){
		if(i != cell){
			float4 velOther = vel_sum[i];
			SHOther = SHEval3(velOther);	

			sumSH = 0.2820947917738781f * 0.2820947917738781f;

			sumSH += SHSelf.s0 * SHOther.s0;
			sumSH += SHSelf.s1 * SHOther.s1;
			sumSH += SHSelf.s2 * SHOther.s2;
			sumSH += SHSelf.s3 * SHOther.s3;
			sumSH += SHSelf.s4 * SHOther.s4;
			sumSH += SHSelf.s5 * SHOther.s5;
			sumSH += SHSelf.s6 * SHOther.s6;
			sumSH += SHSelf.s7 * SHOther.s7;

			//posOther = (float3)((i+id) / plane, ((i+id) % plane) / simParams->gridSize.x, (((i+id) % plane) % simParams->gridSize.x));
			//float fu = fast_distance(posOwn, posOther);
			shVelSum += (sumSH / (1000 )) * velOther ;
			}
	}

	shSum[id] = shVelSum;
	barrier(CLK_LOCAL_MEM_FENCE);

	uint index = start + id;
	while(index < end){
		float4 velOwn = vel[index];
		
		for(int j = 0; j < lSize-1; j++){
			velOwn += shSum[j];
		}

		velOwn = (velOwn < -mVel) ? -mVel : velOwn;
		velOwn = (velOwn > mVel) ? mVel : velOwn;
			
		velOwn += velCor;

		vel[index] = velOwn;
		pos_out[index] = pos[index] + velOwn * dt;

		index += LOCAL_SUM;
	}
}
*/

/*
__kernel void useSH(__global float4* vel,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					 float dt){
	
	uint id = get_local_id(0);
	uint cell = get_group_id(0);

	float4 velCor = checkAndCorrectBoundaries(cell, simParams);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);

	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint index = start + id;

	uint plane = simParams->gridSize.x * simParams->gridSize.y;
	uint numCells = simParams->numCells;

	float3 posOwn = (float3)(cell / plane, (cell % plane) / simParams->gridSize.x, ((cell % plane) % simParams->gridSize.x));

	while(index < end){
		float4 velSum = (float4)(0.0f,0.0f,0.0f,0.0f);

		for(int i = 0; i < numCells; i++){
				if(vel_sum[i].x > 0.0f && vel_sum[i].y > 0.0f && vel_sum[i].z > 0.0f){
					float8 SHSelf = SHEval3(vel[index]);
					float8 SHOther = SHEval3(vel_sum[i]);
					float sumSH = 0.2820947917738781f * 0.2820947917738781f;

					sumSH += SHSelf.s0 * SHOther.s0;
					sumSH += SHSelf.s1 * SHOther.s1;
					sumSH += SHSelf.s2 * SHOther.s2;
					sumSH += SHSelf.s3 * SHOther.s3;
					sumSH += SHSelf.s4 * SHOther.s4;
					sumSH += SHSelf.s5 * SHOther.s5;
					sumSH += SHSelf.s6 * SHOther.s6;
					sumSH += SHSelf.s7 * SHOther.s7;

					//float3 posOther = (float3)(i / plane, (i % plane) / simParams->gridSize.x, ((i % plane) % simParams->gridSize.x));
					//velSum += sumSH * vel_sum[i] / (10 * fast_distance(posOwn, posOther));
				}
			}

		velSum += vel[index];

		velSum = (velSum < -mVel) ? -mVel : velSum;
		velSum = (velSum > mVel) ? mVel : velSum;
			
		velSum += velCor;

		vel[index] = velSum;
		pos_out[index] = pos[index] + velSum * dt;

		index += LOCAL_SUM;
	}
}

*/









