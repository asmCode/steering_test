#include "MathUtils.h"
#include <float.h>

float MathUtils::PI = 3.141592653589793f;
float MathUtils::PI2 = 1.57079632679489f;
float MathUtils::PI4 = 0.78539816339744f;

void MathUtils::FixToZero(float& v)
{
	if (Abs(v) <= FLT_EPSILON)
		v = 0.0f;
}

void MathUtils::FixToZero(sm::Vec3& v)
{
	FixToZero(v.x);
	FixToZero(v.y);
	FixToZero(v.z);
}
