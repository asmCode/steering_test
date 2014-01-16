#include "MathUtils.h"
#include <float.h>

float MathUtils::PI = 3.141592653589793f;
float MathUtils::PI2 = 1.57079632679489f;
float MathUtils::PI4 = 0.78539816339744f;

void MathUtils::FixToZero(sm::Vec3& v)
{
	if (Abs(v.x) <= FLT_EPSILON)
		v.x = 0.0f;

	if (Abs(v.y) <= FLT_EPSILON)
		v.y = 0.0f;

	if (Abs(v.z) <= FLT_EPSILON)
		v.z = 0.0f;
}

