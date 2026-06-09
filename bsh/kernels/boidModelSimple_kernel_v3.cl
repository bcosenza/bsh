/*
	As v2 with other memory access pattern. (However, runtimes suggest it is really ineffective)
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

//must be same definition as in boidModel.h
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

//return cell position of boid. 
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



//simulation step kernel
__kernel void boidKernel(__global float4* pos, __global float4* pos_out, __global float4* vel, __global float4* vel_out, float dt, __constant simParams_t* simParams)
{
    //get our index in the array
    unsigned int id = get_global_id(0);

    float4 p = pos[id];
    float4 v = vel[id];

	float4 cohesion = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 separation = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 alignment = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 dist = (float4)(0.0f,0.0f,0.0f,0.0f);
	float4 maxVelocity = (float4)(simParams->maxVel, simParams->maxVel, simParams->maxVel, 0);
	float4 velCor = (float4)(0.f, 0.f, 0.f, 0.f);
	float4 otherBoid = (float4)(0.f, 0.f, 0.f, 0.f);

	float distLength;
	float angle;

	uint index;

	int numFlockMates = 0;
	
	//compare every boid with every other boid
	for(int j = 1; j < simParams->numBodies; j++){
		index = (j + id) % simParams->numBodies;

		otherBoid = pos[index];			//position of other boid
		dist = otherBoid - p;			//distance vector to other boid
		distLength = length(dist);		//length of distance vector
			
		if(distLength < 10.f){										//check if other boid is local neighborhood
			float dotP = dot(-v, dist);								
			float lenV = length(v);
			angle = dotP / (lenV * distLength);						//calculate acute angle between other boid and self (0° to 90°)
				
			if(dotP < 0.f || fabs(degrees(acos(angle))) > 45){		//check if angle is greaten than 45° because acute angle is used negative dot product indicates > 90°
				cohesion += otherBoid;									
				alignment += vel[index];	
				numFlockMates += 1;									//increase number of boids in local neighborhood
			
				if(distLength < 2.5f)								//check if other boid is near enough to apply separation rule
					separation -= dist;
			}
		}

		
	}


	if(numFlockMates > 0){
		alignment = (alignment / numFlockMates) - v;					//calculate perceived velocity
		cohesion =  (cohesion / numFlockMates) - p;						//calculate perceived center of mass
	}

	//apply weights to rules and calculate new velocity
	v = v * simParams->wOwn + (cohesion * simParams->wCohesion + alignment * simParams->wAlignment + separation * simParams->wSeparation) / 2.0;
	v.w = 0.0f;

	//check if velocity is greater than maximum or smaller than minimum velocity
	float len = length(v);

	if(len > simParams->maxVel){
		v.x = (v.x / len) * simParams->maxVel;
		v.z = (v.z / len) * simParams->maxVel;
		v.y = (v.y / len) * simParams->maxVel;
	}

	//get grid position and check if boid is in border cell
	int4 gridPos = getGridPos(p, simParams);

	if(gridPos.x < boundingBoxFactor)
		velCor.x = simParams->maxVelCor;
	
	if(gridPos.x >= (simParams->gridSize.x - boundingBoxFactor))
		velCor.x = -simParams->maxVelCor;

	if(gridPos.y < boundingBoxFactor)
		velCor.y = simParams->maxVelCor;
	
	if(gridPos.y >= (simParams->gridSize.y - boundingBoxFactor))
		velCor.y = -simParams->maxVelCor;
   	
	if(gridPos.z < boundingBoxFactor)
		velCor.z = simParams->maxVelCor;
	
	if(gridPos.z >= (simParams->gridSize.z - boundingBoxFactor))
		velCor.z = -simParams->maxVelCor;

	//add velocity correction if boid is in border case
	v += velCor;
	//calculate new position
	p += v*dt;

	//write back to output memory
	pos_out[id] = p;
	vel_out[id] = v;
}