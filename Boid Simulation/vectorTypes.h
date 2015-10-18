#ifndef _VECTOR_TYPES_
#define _VECTOR_TYPES_

typedef struct Vec4 {
	float x, y, z, w;
	Vec4(){};
	//convenience functions
	Vec4(float xx, float yy, float zz, float ww) :
		x(xx),
		y(yy),
		z(zz),
		w(ww)
	{}
	void set(float xx, float yy, float zz, float ww = 1.) {
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}
} Vec4; //__attribute__((aligned(16)));

typedef struct Vec4_int
{
	unsigned int x, y, z, w;
	Vec4_int(){};
	//convenience functions
	Vec4_int(unsigned int xx, unsigned  int yy, unsigned  int zz, unsigned  int ww) :
		x(xx),
		y(yy),
		z(zz),
		w(ww)
	{}
	void set(unsigned int xx, unsigned  int yy, unsigned  int zz, unsigned  int ww = 1.) {
		x = xx;
		y = yy;
		z = zz;
		w = ww;
	}
} Vec4_int; //__attribute__((aligned(16)));

typedef struct Vec3_float{
	float x, y, z;
	Vec3_float(){};
	Vec3_float(float xx, float yy, float zz) : x(xx), y(yy), z(zz){}
} Vec3_float;

#endif