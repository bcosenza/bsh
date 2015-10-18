/*
	As boidModelGrid_kernel except it uses localMem bigger than the local group size.
	Using fast methods instead of normal ones.
*/

#define LOCAL_SIZE_LIMIT 512U
#define LOCAL_SUM 64U

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

float4 checkAndCorrectBoundaries(   uint cell, __constant simParams_t* params)
{
	uint sizePlane = params->gridSize.x * params->gridSize.y;
	float4 cor = (float4)(0.0f,0.0f,0.0f,0.0f);

	if(cell >= (params->numCells - sizePlane))
		cor.z = -params->maxVelCor;
	else if(cell < sizePlane)
		cor.z = params->maxVelCor;

	cell = cell % sizePlane;

	if(cell >= (sizePlane - params->gridSize.x))
		cor.y = -params->maxVelCor;
	else if(cell < params->gridSize.x)
		cor.y = params->maxVelCor;

	cell = cell % params->gridSize.x;

	if(cell >= (params->gridSize.x - 1))
		cor.x = -params->maxVelCor;

	if(cell < 1)
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

	gridHashUnsorted[id] = gridPos.x + (params->gridSize.x) * gridPos.y + (params->gridSize.y) * (params->gridSize.x) * gridPos.z;
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
	
	float4 velSum = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 posSum = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 perceivedPos = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 perceivedVel = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 separation = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 distance = (float4)(0.0f,0.0f,0.0f,0.0f);
	const float4 mVel = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);

	uint startIndex = cellStart[cell];
	uint endIndex = cellEnd[cell];
	uint range = endIndex - startIndex;

	int flockMatesVisible = 0;
	int flockMatesNear = 0;

	float angle;

	if(get_local_id(0) == 0)
		range_out[cell] = range;

	//test if cell is empty
	if(range == 0)
		return;

	float4 velCor = (float4)(0.0f, 0.0f, 0.0f, 0.0f);

	//load boids into local memory, 
	if(range <= get_local_size(0)){
		uint cellPos = id + startIndex;

		if(cellPos < endIndex){
			localPos[id] = pos[cellPos];
			localVel[id] = vel[cellPos];
		

			barrier(CLK_LOCAL_MEM_FENCE);

			for(int i = 1; i < range; i++){
				cellPos = (id + i) % range;

				float4 v = localVel[cellPos];
				distance = localPos[cellPos] - localPos[id];
				angle = dot(-v, distance) / (sqrt((v.x*v.x + v.y*v.y + v.z*v.z) * (distance.x*distance.x + distance.y*distance.y + distance.z*distance.z)));
				
				if(fabs(degrees(acos(angle))) > 45){

					flockMatesVisible++;
					perceivedPos += localPos[cellPos];
					perceivedVel += localVel[cellPos];
				
					if(fabs(distance.x) < 1.0f && fabs(distance.y) < 1.0f && fabs(distance.z) < 1.0f){
						separation -= distance;
						flockMatesNear++;
					}
				}
			}

			if(flockMatesVisible > 1)
				perceivedPos = (perceivedPos / flockMatesVisible) - localPos[id];
				
			if(flockMatesNear > 1)
				perceivedVel = (perceivedVel / flockMatesNear) - localVel[id];

			velSum = localVel[id] + perceivedPos * simParams->wCohesion  + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
			
			velSum = (velSum < -mVel) ? -mVel : velSum;
			velSum = (velSum > mVel) ? mVel : velSum;

			//velSum += velCor;

			vel_out[startIndex + id] = velSum;
			//pos_out[startIndex + id] = localPos[id] + velSum * dt;
		}
	} else {
		for(int i = 0; i < ((range/get_local_size(0)) + 1); i++){
			uint cellPos;
			int j = 1;
			int rRange = range % get_local_size(0);
			flockMatesVisible = 0;
			flockMatesNear = 0;
			
			for( int x = 0; x < ((range/get_local_size(0)) + 1); x++ ){
				cellPos = id + startIndex + (((i + x) % (range/get_local_size(0) + 1)) * get_local_size(0));

				if(cellPos < endIndex){
					
					localVel[id] = vel[cellPos];
					localPos[id] = pos[cellPos];

					if(x == 0){
						velSum = localVel[id];	
						posSum = localPos[id];
					}
					
				
					barrier(CLK_LOCAL_MEM_FENCE);		
					
					if((i + x) != (range/get_local_size(0))){
						for(j; j < get_local_size(0); j++){
							cellPos = (id + j) % get_local_size(0);

							float4 v = velSum;
							distance = localPos[cellPos] - posSum;
							angle = dot(-v, distance) / (sqrt((v.x*v.x + v.y*v.y + v.z*v.z) * (distance.x*distance.x + distance.y*distance.y + distance.z*distance.z)));
				
							if(fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(fabs(distance.x) < 1.0f && fabs(distance.y) < 1.0f && fabs(distance.z) < 1.0f){
									separation -= distance;
									flockMatesNear++;
								}
							}
						}
					} else {
						for(j; j < rRange; j++){
							cellPos = (id + j) % rRange;

							float4 v = velSum;
							distance = localPos[cellPos] - posSum;
							angle = dot(-v, distance) / (sqrt((v.x*v.x + v.y*v.y + v.z*v.z) * (distance.x*distance.x + distance.y*distance.y + distance.z*distance.z)));
				
							if(fabs(degrees(acos(angle))) > 45){
								perceivedPos += localPos[cellPos];
								perceivedVel += localVel[cellPos];
								flockMatesVisible++;

								if(fabs(distance.x) < 1.0f && fabs(distance.y) < 1.0f && fabs(distance.z) < 1.0f){
									separation -= distance;
									flockMatesNear++;
								}
							}
						}
					}//end if case last chunk
					
					j = 0;	
				}//end if case cellPos < endIndex
			}//end inner for loop

			if(flockMatesVisible > 1)
				perceivedPos = (perceivedPos / flockMatesVisible) - localPos[id];
				
			if(flockMatesNear > 1)
				perceivedVel = (perceivedVel / flockMatesNear) - localVel[id];

			velSum = localVel[id] + perceivedPos * simParams->wCohesion  + perceivedVel * simParams->wAlignment + separation * simParams->wSeparation;
			
			velSum = (velSum < -mVel) ? -mVel : velSum;
			velSum = (velSum > mVel) ? mVel : velSum;
			
			//velSum += velCor;

			cellPos = startIndex + id + i * get_local_size(0);

			if(cellPos < endIndex){
				vel_out[cellPos] = velSum;
			//	pos_out[cellPos] = posSum + velSum * dt;
			}
		}//end outer for loop
	} //end else
	
}


__kernel void SHEval3(float4* vel_in, float8* SHCoeff)
{
   float4 vel = vel_in[get_global_id(0)];
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

   SHCoeff[get_global_id(0)] = pSH;
}

__kernel void sumVelSH(__global float4* vel, __global uint* startIndex, __global uint* endIndex, __global float4* vel_sum, __local float4* sumArray){
	uint id = get_local_id(0);
	uint cell = get_group_id(0);

	uint cellPosTest = cell;

	uint start = startIndex[cell];
	uint end = endIndex[cell];

	uint index = start + id;

	sumArray[id] = (float4)(0.0f,0.0f,0.0f,0.0f);
	sumArray[id+LOCAL_SUM/2] = (float4)(0.0f,0.0f,0.0f,0.0f);
	barrier(CLK_LOCAL_MEM_FENCE);

	while(index < end){
		sumArray[id] += vel[index];
		index += LOCAL_SUM ;
	}

	uint range = LOCAL_SUM / 2;

	while(id < range){
		sumArray[id] += sumArray[id + range];
		range = range / 2;
	}

	if(id == 0){
		vel_sum[cell] = sumArray[id];
	}

}

__kernel void useSH(__global float4* vel,
					__global uint* startIndex, 
					__global uint* endIndex, 
					__global float4* vel_sum, 
					__constant simParams_t* simParams, 
					__global float4* pos,
					__global float4* pos_out,
					 float dt){
	

}











