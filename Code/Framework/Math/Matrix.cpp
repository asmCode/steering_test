#include "Matrix.h"

namespace sm
{

Matrix Matrix::Identity = CreateIdentityMatrix();

Matrix Matrix::CreateIdentityMatrix()
{
	Matrix mat;
	mat.a[0] = 1.0f;
	mat.a[1] = 0.0f;
	mat.a[2] = 0.0f;
	mat.a[3] = 0.0f;
	mat.a[4] = 0.0f;
	mat.a[5] = 1.0f;
	mat.a[6] = 0.0f;
	mat.a[7] = 0.0f;
	mat.a[8] = 0.0f;
	mat.a[9] = 0.0f;
	mat.a[10] = 1.0f;
	mat.a[11] = 0.0f;
	mat.a[12] = 0.0f;
	mat.a[13] = 0.0f;
	mat.a[14] = 0.0f;
	mat.a[15] = 1.0f;

	return mat;
}

Matrix Matrix::CreateLookAt2(const sm::Vec3 &direction, const sm::Vec3 &up)
{
	sm::Matrix rot = sm::Matrix::IdentityMatrix();

	sm::Vec3 right = (direction * up).GetNormalized();
	sm::Vec3 yAxis = (right * direction).GetNormalized();

	rot.a[0] = right.x;
	rot.a[1] = right.y;
	rot.a[2] = right.z;

	rot.a[4] = yAxis.x;
	rot.a[5] = yAxis.y;
	rot.a[6] = yAxis.z;

	rot.a[8] = direction.x;
	rot.a[9] = direction.y;
	rot.a[10] = direction.z;

	return rot;
}

Vec3 Matrix::TransformNormal(const Vec3& normal)
{
	Vec3 ret;

	for (int i = 0; i < 3; i++)
	{
		float val = 0.0f;
		for (int k = 0; k < 3; k++)
			val += a[(k * 4) + i] * normal[k];
		ret[i] = val;
	}

	return ret;
}

}
