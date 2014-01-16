#include "Animation.h"
#include "Model.h"
#include "Mesh.h"

Animation::Animation(void) :
	m_animLength(-1.0f)
{
	id = 0;
	mesh = NULL;
	worldTMInv = sm::Matrix::IdentityMatrix();

	pos = NULL;
	rot = NULL;
	scale = NULL;

	localPos.Set(0, 0, 0);
	localRot.RotateToQuat(0, 0, 0, 0);
	localScale.Set(1, 1, 1);

	hasOwnRotate = false;
	ownAngle = 0.0f;
	angleScale = 0.0f;
}

Animation::~Animation(void)
{
	if (pos != NULL)
		delete pos;

	if (rot != NULL)
		delete rot;

	if (scale != NULL)
		delete scale;

	for (unsigned i = 0; i < subAnims.size(); i++)
		delete subAnims[i];
}

void Animation::AssignModel(Model *model)
{
	for (unsigned i = 0; i < subAnims.size(); i++)
		subAnims[i] ->AssignModel(model);

	if (id == 0)
		return;

	std::vector<Mesh*> &meshes = model ->GetMeshes();
	for (unsigned i = 0; i < meshes.size(); i++)
	{
		if (meshes[i] ->id == id)
		{
			mesh = meshes[i];
			return;
		}
	}
}

void Animation::AssignTransformable(ITransformable *transformable)
{
	for (unsigned i = 0; i < subAnims.size(); i++)
		subAnims[i] ->AssignTransformable(transformable);

	if (id == 0)
		return;

	if (transformable->GetId() == id)
		mesh = transformable;
}

void Animation::Update(float time, const sm::Matrix &transform, float seconds)
{
	if (mesh != NULL && mesh->IsCam())
	{
		sm::Matrix tr = transform;

		sm::Vec3 posVal;
		sm::Quat rotVal;

		float angle;
		sm::Vec3 axis;

		if (pos != NULL)
			pos ->GetValue(time, posVal);
		else
			posVal = localPos;

		if (rot != NULL)
			rot ->GetValue(time, rotVal);
		else
			rotVal = localRot;

		rotVal.Normalize();
		rotVal.QuatToRotate(angle, axis);

		tr *= sm::Matrix::TranslateMatrix(posVal);
		tr *= sm::Matrix::RotateAxisMatrix(angle, axis);

		if (mesh != NULL)
		{
			mesh ->Transform() = sm::Matrix::RotateAxisMatrix(1.5707963f, sm::Vec3(1, 0, 0));
			mesh ->Transform() *= tr.GetInversed();
			//mesh ->Transform() *= worldTMInv.GetInversed();
		}
	}
	else
	{
		sm::Matrix tr = transform;

		sm::Vec3 posVal;
		sm::Quat rotVal;
		sm::Vec3 scaleVal;

		float angle;
		sm::Vec3 axis;

		if (pos != NULL)
			pos ->GetValue(time, posVal);
		else
			posVal = localPos;

		if (rot != NULL)
			rot ->GetValue(time, rotVal);
		else
			rotVal = localRot;

		if (scale != NULL)
			scale ->GetValue(time, scaleVal);
		else
			scaleVal = localScale;

		rotVal.Normalize();
		rotVal.QuatToRotate(angle, axis);

		tr *= sm::Matrix::TranslateMatrix(posVal);
		tr *= sm::Matrix::ScaleMatrix(scaleVal);

		sm::Matrix loda = tr;
		
		loda *= sm::Matrix::RotateAxisMatrix(angle, axis);

		if (hasOwnRotate)
		{
			if (time > 82.5f)
			{
				tr *= sm::Matrix::RotateAxisMatrix(ownAngle, axis);
				//tr *= sm::Matrix::RotateAxisMatrix(angle, axis);
				ownAngle += seconds * 8.0f * angleScale;
			}
		}
		else
		{
			tr *= sm::Matrix::RotateAxisMatrix(angle, axis);
		}

		if (mesh != NULL)
		{
			mesh ->AnimTransform() = tr;
			mesh ->Transform() = tr;
			mesh ->Transform() *= worldTMInv;
		}

		for (unsigned i = 0; i < subAnims.size(); i++)
			subAnims[i] ->Update(time, loda, seconds);
	}
}

Animation *Animation::GetAnimationById(int id)
{
	if (this->id == id)
		return this;

	Animation *anim = NULL;

	for (unsigned i = 0; i < subAnims.size(); i++)
	{
		anim = subAnims[i]->GetAnimationById(id);
		if (anim != NULL)
			return anim;
	}

	return NULL;
}

float Animation::GetAnimLength()
{
	if (m_animLength != -1.0f)
		return m_animLength;

	float posTime = 0.0f;
	float rotTime = 0.0f;
	float scaleTime = 0.0f;

	sm::Vec3 vdummy;
	sm::Quat qdummy;
	bool bdummy;

	if (pos != NULL && pos->GetKeysCount() > 0)
		pos->GetKeyframe(pos->GetKeysCount() - 1, posTime, vdummy, bdummy);
	if (rot != NULL && rot->GetKeysCount() > 0)
		rot->GetKeyframe(rot->GetKeysCount() - 1, rotTime, qdummy, bdummy);
	if (scale != NULL && scale->GetKeysCount() > 0)
		scale->GetKeyframe(scale->GetKeysCount() - 1, scaleTime, vdummy, bdummy);

	float maxAnimLength = 0;
	for (int i = 0; i < subAnims.size(); i++)
	{
		float length = subAnims[i]->GetAnimLength();
		if (length > maxAnimLength)
			maxAnimLength = length;
	}

	m_animLength = fmax(fmax(fmax(posTime, rotTime), scaleTime), maxAnimLength);

	return m_animLength;
}

