// using fast_fast_length instead of fast_length
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

/*return cell position of boid*/
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
    if(index < numParticles){
        hash = gridHash[index];

        //Load hash data into local memory so that we can look 
        //at neighboring boids's hash value without loading
        //two hash values per thread
        localHash[get_local_id(0) + 1] = hash;

        //First thread in block must load neighbor boids hash
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



/*simulation step*/
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
				angle = dotP / (fast_length(localVel[id]) * fast_length(distance));		//calculate acute angle between self and other boid
				
				if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){	//check if other boid is visible, dot product indicates that angle is > 90°

					flockMatesVisible++;							//increase counter how many boids are visible
					perceivedPos += localPos[cellPos];				//add other boids position to perceived center of mass
					perceivedVel += localVel[cellPos];				//add other boids velocity to perceived aligned velocity
				
					if(fast_length(distance) < 2.5f)						//check if other boid is near enough to separate
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
			velOwn.w = 0.0;

			//truncate velocity to max velocity
			//velOwn = clamp(velOwn, -mVel, mVel);
			//using the magnitude of the velocity is nicer than clamping it

			float len = fast_length(velOwn);

			if(len > simParams->maxVel){
				velOwn.x = (velOwn.x / len) * simParams->maxVel;
				velOwn.z = (velOwn.z / len) * simParams->maxVel;
				velOwn.y = (velOwn.y / len) * simParams->maxVel;
			}
			

			//add correction velocity if boid in border cell
			velOwn += velCor;

			//write back velocity and new position
			vel_out[startIndex + id] = velOwn;
			pos_out[startIndex + id] = localPos[id] + velOwn * dt;
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
							angle = dotP / (fast_length(velOwn) * fast_length(distance));
				
							if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(fast_length(distance) < 2.5f)
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
							angle = dotP / (fast_length(velOwn) * fast_length(distance));
				
							if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(fast_length(distance) < 2.5f)
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
			velOwn.w = 0.0;

			//truncate velocity to max velocity
			//velOwn = clamp(velOwn, -mVel, mVel);
			//using the magnitude of the velocity is nicer than clamping it

			float len = fast_length(velOwn);

			if(len > simParams->maxVel){
				velOwn.x = (velOwn.x / len) * simParams->maxVel;
				velOwn.z = (velOwn.z / len) * simParams->maxVel;
				velOwn.y = (velOwn.y / len) * simParams->maxVel;
			}
			
			//apply correction velocity dependend on boid cell position (border case)
			velOwn += velCor;

			cellPos = startIndex + id + i * get_local_size(0);

			//write back to output, but check if it is not outside own cell (could be possible in border case)
			if(cellPos < endIndex){
				vel_out[cellPos] = velOwn;
				pos_out[cellPos] = posOwn + velOwn * dt;
			}

			perceivedPos = (float4)(0.0f,0.0f,0.0f,0.0f);
			perceivedVel = (float4)(0.0f,0.0f,0.0f,0.0f);
			separation = (float4)(0.0f,0.0f,0.0f,0.0f);
			flockMatesVisible = 0;
			
		}//end outer for loop
	} //end else
}

