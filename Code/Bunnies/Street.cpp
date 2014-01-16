#include "Street.h"
#include "StreetPiece.h"
#include "StreetSegment.h"
#include "StreetLights.h"
#include "PedsManager.h"
#include "TrafficManager.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "StreetMap.h"
#include "Environment.h"
#include "Sprite.h"

#include <Utils/Randomizer.h>
#include <Graphics/Model.h>
#include <Graphics/Texture.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <assert.h>
#include "BoxCollider.h"

#include <Graphics/OpenglPort.h>

Street *Street::Instance;

Street::Street(PedsManager *pedsManager, TrafficManager* trafficManager) :
	m_trafficManager(trafficManager)
{
	Instance = this;
	m_pedsManager = pedsManager;

	Content *content = InterfaceProvider::GetContent();

	m_lightTexture = content->Get<Texture>("circle");
	assert(m_lightTexture != NULL);

	m_streetLightsModel = content->Get<Model>("lights");
	assert(m_streetLightsModel != NULL);

	m_streetModel = content->Get<Model>("street_straight");
	assert(m_streetModel != NULL);

	Model *streetTurn = content->Get<Model>("street_turn");
	assert(streetTurn != NULL);

	Model *streetTCross = content->Get<Model>("street_t_cross");
	assert(streetTCross != NULL);

	Model *streetCross = content->Get<Model>("street_cross");
	assert(streetCross != NULL);

	m_skycrapper1 = content->Get<Model>("skycrapper01");
	assert(m_skycrapper1 != NULL);
	m_skycrapperTexture1 = content->Get<Texture>("flat_1_diffuse");
	assert(m_skycrapperTexture1 != NULL);

	m_skycrapper2 = content->Get<Model>("skycrapper02");
	assert(m_skycrapper2 != NULL);
	m_skycrapperTexture2 = content->Get<Texture>("flat_2_diffuse");
	assert(m_skycrapperTexture2 != NULL);

	m_skycrapper3 = content->Get<Model>("skycrapper03");
	assert(m_skycrapper3 != NULL);
	m_skycrapperTexture3 = content->Get<Texture>("flat_3_diffuse");
	assert(m_skycrapperTexture3 != NULL);

	m_pavementTexture = content->Get<Texture>("street_pavement");
	assert(m_pavementTexture != NULL);

	m_streetMap = new StreetMap(TaxiGame::Environment::GetInstance()->GetBasePath() + "data/");

	m_streetPieces[StreetPiece::PieceType_Pavement] = new StreetPiece(
		StreetPiece::PieceType_Pavement,
		m_streetModel,
		content->Get<Texture>("street_pavement"),
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_StraightVert_1] = new StreetPiece(
		StreetPiece::PieceType_StraightVert_1,
		m_streetModel,
		content->Get<Texture>("street_1_straight"),
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_StraightHori_1] = new StreetPiece(
		StreetPiece::PieceType_StraightHori_1,
		m_streetModel,
		content->Get<Texture>("street_1_straight"),
		sm::Matrix::RotateAxisMatrix(MathUtils::PI2, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TurnUpRight_1] = new StreetPiece(
		StreetPiece::PieceType_TurnUpRight_1,
		streetTurn,
		content->Get<Texture>("street_1_turn"),
		sm::Matrix::RotateAxisMatrix(MathUtils::PI2, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TurnUpLeft_1] = new StreetPiece(
		StreetPiece::PieceType_TurnUpLeft_1,
		streetTurn,
		content->Get<Texture>("street_1_turn"),
		sm::Matrix::RotateAxisMatrix(MathUtils::PI, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TurnDownRight_1] = new StreetPiece(
		StreetPiece::PieceType_TurnDownRight_1,
		streetTurn,
		content->Get<Texture>("street_1_turn"),
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_TurnDownLeft_1] = new StreetPiece(
		StreetPiece::PieceType_TurnDownLeft_1,
		streetTurn,
		content->Get<Texture>("street_1_turn"),
		sm::Matrix::RotateAxisMatrix(-MathUtils::PI2, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TUp_1] = new StreetPiece(
		StreetPiece::PieceType_TUp_1,
		streetTCross,
		content->Get<Texture>("street_1_t"),
		sm::Matrix::RotateAxisMatrix(MathUtils::PI, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TDown_1] = new StreetPiece(
		StreetPiece::PieceType_TDown_1,
		streetTCross,
		content->Get<Texture>("street_1_t"),
		sm::Matrix::ScaleMatrix(1, 1, 1));

	m_streetPieces[StreetPiece::PieceType_TLeft_1] = new StreetPiece(
		StreetPiece::PieceType_TLeft_1,
		streetTCross,
		content->Get<Texture>("street_1_t"),
		sm::Matrix::RotateAxisMatrix(MathUtils::PI2, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_TRight_1] = new StreetPiece(
		StreetPiece::PieceType_TRight_1,
		streetTCross,
		content->Get<Texture>("street_1_t"),
		sm::Matrix::RotateAxisMatrix(-MathUtils::PI2, 0, 1, 0));

	m_streetPieces[StreetPiece::PieceType_Cross_1] = new StreetPiece(
		StreetPiece::PieceType_Cross_1,
		streetCross,
		content->Get<Texture>("street_1_cross"),
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_Skycrapper_1] = new StreetPiece(
		StreetPiece::PieceType_Skycrapper_1,
		m_skycrapper1,
		m_skycrapperTexture1,
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_Skycrapper_2] = new StreetPiece(
		StreetPiece::PieceType_Skycrapper_2,
		m_skycrapper2,
		m_skycrapperTexture2,
		sm::Matrix::IdentityMatrix());

	m_streetPieces[StreetPiece::PieceType_Skycrapper_3] = new StreetPiece(
		StreetPiece::PieceType_Skycrapper_3,
		m_skycrapper3,
		m_skycrapperTexture3,
		sm::Matrix::IdentityMatrix());

	m_streetSegments = new StreetSegment*[m_streetMap->GetWidth() * m_streetMap->GetHeight()];
	for (uint32_t y = 0; y < m_streetMap->GetHeight(); y++)
	{
		for (uint32_t x = 0; x < m_streetMap->GetWidth(); x++)
		{
			int index = y * m_streetMap->GetWidth() + x;

			StreetPiece::PieceType streetType = m_streetMap->GetPieceType(x, y);

			sm::Matrix randRotation = sm::Matrix::IdentityMatrix();
			if (streetType == StreetPiece::PieceType_Skycrapper_1 ||
				streetType == StreetPiece::PieceType_Skycrapper_2 ||
				streetType == StreetPiece::PieceType_Skycrapper_3)
			{
				float randAngle[4];
				randAngle[0] = 0;
				randAngle[1] = MathUtils::PI2 + MathUtils::PI;
				randAngle[2] = MathUtils::PI2;
				randAngle[3] = MathUtils::PI;

				static Randomizer random;

				randRotation = sm::Matrix::RotateAxisMatrix(randAngle[random.GetInt(0, 3)], 0, 1, 0);
			}

			sm::Vec3 pivot = sm::Vec3(x * 10.0f, 0, y * 10.0f);
			m_streetSegments[index] =
				new StreetSegment(
					sm::Matrix::TranslateMatrix(pivot) * randRotation,
					pivot,
					m_streetPieces[(uint8_t)streetType]);

			if (m_streetSegments[index]->GetStreetPiece()->HasPavement())
				m_pavementSegments.push_back(m_streetSegments[index]);
		}
	}

	for (unsigned int i = 0; i < m_streetMap->GetWidth() * m_streetMap->GetHeight(); i++)
		m_streetSegments[i]->Initialize();
}

Street::~Street()
{
}

void Street::Update(float time, float seconds)
{
	StreetSegment *streetSegment = GetSegmentAtPosition(m_taxiPosition);
	if (streetSegment == NULL) // out of map
		return;

	int ix = streetSegment->CoordX();
	int iy = streetSegment->CoordY();

	std::vector<StreetSegment*> segmentsChanged;

	for (int y = iy - Range - 1; y <= iy + Range + 1; y++)
	{
		for (int x = ix - Range - 1; x <= ix + Range + 1; x++)
		{
			if (x < 0 || x >= (int)m_streetMap->GetWidth() || y < 0 || y >= (int)m_streetMap->GetHeight())
				continue;

			StreetSegment *ss = GetSegment(x, y);

			ss->Update(seconds);
		}
	}

	if (streetSegment == m_lastTaxiSegment)
		return;

	for (int y = iy - Range - 1; y <= iy + Range + 1; y++)
	{
		for (int x = ix - Range - 1; x <= ix + Range + 1; x++)
		{
			if (x < 0 || x >= (int)m_streetMap->GetWidth() || y < 0 || y >= (int)m_streetMap->GetHeight())
				continue;

			StreetSegment *ss = GetSegment(x, y);

			if (x < ix - Range ||
				x > ix + Range ||
				y < iy - Range ||
				y > iy + Range)
			{
				if (ss->IsVisible())
				{
					ss->SetVisibility(false);
					segmentsChanged.push_back(ss);
				}
			}
			else
			{
				if (!ss->IsVisible())
				{
					ss->SetVisibility(true);
					segmentsChanged.push_back(ss);
				}
			}
		}
	}

	for (uint32_t i = 0; i < segmentsChanged.size(); i++)
	{
		m_pedsManager->NotifyStreetSegmentVisibilityChanged(segmentsChanged[i]);
		m_trafficManager->NotifyStreetSegmentVisibilityChanged(segmentsChanged[i]);
	}

	/*if (ix > m_lastTaxiSegment->CoordX() && ix < m_streetMap->GetWidth() - Range)
	{
		StreetSegment *ss;

		ss = GetSegment(ix + Range, iy);
		if (!ss->IsVisible())
		{
			ss->SetVisibility(true);
			m_pedsManager->NotifyStreetSegmentVisibilityChanged(ss);
		}
	}

	m_lastTaxiSegment = streetSegment;*/
}

void Street::Draw(float time, float seconds)
{
	sm::Matrix world = sm::Matrix::IdentityMatrix();

	for (uint32_t y = 0; y < m_streetMap->GetHeight(); y++)
	{
		for (uint32_t x = 0; x < m_streetMap->GetWidth(); x++)
		{
			StreetPiece::PieceType streetType = m_streetMap->GetPieceType(x, y);

			StreetSegment *seg = m_streetSegments[y * m_streetMap->GetWidth() + x];

			if (!seg->IsVisible())
				continue;

			m_streetPieces[streetType]->Draw(seg->GetWorldTransform());
		}
	}
}

void Street::DrawStreetLights(float time, float seconds)
{
	StreetSegment *centerSegment = GetSegmentAtPosition(m_taxiPosition);
	assert(centerSegment != NULL);

	int ix = centerSegment->CoordX();
	int iy = centerSegment->CoordY();

	for (int y = iy - Range - 1; y <= iy + Range + 1; y++)
	{
		for (int x = ix - Range - 1; x <= ix + Range + 1; x++)
		{
			if (x < 0 || x >= (int)m_streetMap->GetWidth() || y < 0 || y >= (int)m_streetMap->GetHeight())
				continue;

			StreetSegment *ss = GetSegment(x, y);

			StreetLights **streetLights = ss->GetStreetLights();

			for (int i = 0; i < StreetSegment::MaxLights; i++)
			{
				if (streetLights[i] != NULL)
				{
					DrawingRoutines::DrawWithMaterial(m_streetLightsModel->m_meshParts, streetLights[i]->GetTransform());
				}
			}
		}
	}
}

void Street::DrawStreetLightsSprites(float time, float seconds)
{
	Sprite::Setup();
	Sprite::SetTexture(m_lightTexture);

	StreetSegment *centerSegment = GetSegmentAtPosition(m_taxiPosition);
	assert(centerSegment != NULL);

	int ix = centerSegment->CoordX();
	int iy = centerSegment->CoordY();

	for (int y = iy - Range - 1; y <= iy + Range + 1; y++)
	{
		for (int x = ix - Range - 1; x <= ix + Range + 1; x++)
		{
			if (x < 0 || x >= (int)m_streetMap->GetWidth() || y < 0 || y >= (int)m_streetMap->GetHeight())
				continue;

			StreetSegment *ss = GetSegment(x, y);

			StreetLights **streetLights = ss->GetStreetLights();

			for (int i = 0; i < StreetSegment::MaxLights; i++)
			{
				if (streetLights[i] != NULL)
				{
					switch (streetLights[i]->GetLightColor())
					{
					case StreetLights::LightColor_Red: Sprite::SetColor(sm::Vec4(1, 0, 0, 1)); break;
					case StreetLights::LightColor_Orange: Sprite::SetColor(sm::Vec4(1, 0.5f, 0.0f, 1)); break;
					case StreetLights::LightColor_Green: Sprite::SetColor(sm::Vec4(0, 1, 0, 1)); break;
					}

					Sprite::Draw(
						DrawingRoutines::GetProjMatrix() *
						DrawingRoutines::GetViewMatrix() *
						streetLights[i]->GetTransform() *
						sm::Matrix::TranslateMatrix(0, 0.1f, 0) *
						sm::Matrix::RotateAxisMatrix(MathUtils::PI2, 1, 0, 0) *
						sm::Matrix::ScaleMatrix(1, 1, 1));
				}
			}
		}
	}

	Sprite::Clean();
}

StreetPiece* Street::GetStreetPiece(uint8_t type)
{
	return m_streetPieces[type];
}

void Street::SetTaxiPosition(const sm::Vec3 &taxiPosition)
{
	m_taxiPosition = taxiPosition;
}

void Street::GetSegmentCoords(const sm::Vec3 &position, int32_t &x, int32_t &y) const
{
	x = (int32_t)((position.x + 5.0f) / 10.0f);
	y = (int32_t)((position.z + 5.0f) / 10.0f);
}

StreetSegment* Street::GetSegmentAtPosition(const sm::Vec3 &position) const
{
	int32_t x;
	int32_t y;
	GetSegmentCoords(position, x, y);

	if (x < 0 || y < 0 || x >= (int)m_streetMap->GetWidth() || y >= (int)m_streetMap->GetHeight())
		return NULL;

	return m_streetSegments[y * m_streetMap->GetWidth() + x];
}

StreetSegment* Street::GetSegment(int x, int y) const
{
	if (x < 0 || x >= (int)m_streetMap->GetWidth() || y < 0 || y >= (int)m_streetMap->GetHeight())
		return NULL;

	return m_streetSegments[y * m_streetMap->GetWidth() + x];
}

StreetSegment *Street::GetRandomPavement()
{
	static Randomizer random;

	int randomIndex = random.GetInt(0, m_pavementSegments.size() - 1);

	return m_pavementSegments[randomIndex];
}

void Street::SetInitialVisibility(const sm::Vec3 &taxiPosition)
{
	for (uint32_t i = 0; i < m_streetMap->GetWidth() * m_streetMap->GetHeight(); i++)
		m_streetSegments[i]->SetVisibility(false);

	m_lastTaxiSegment = GetSegmentAtPosition(taxiPosition);

	uint32_t ix = m_lastTaxiSegment->CoordX();
	uint32_t iy = m_lastTaxiSegment->CoordY();

	for (int y = MathUtils::Max((int)iy - Range, 0); y <= MathUtils::Min((int)iy + Range, (int)m_streetMap->GetHeight() - 1); y++)
	{
		for (int x = MathUtils::Max((int)ix - Range, 0); x <= MathUtils::Min((int)ix + Range, (int)m_streetMap->GetWidth() - 1); x++)
		{
			m_streetSegments[y * m_streetMap->GetWidth() + x]->SetVisibility(true);
			//m_pedsManager->NotifyStreetSegmentVisibilityChanged(m_streetSegments[y * m_streetMap->GetWidth() + x]);
		}
	}

	for (int y = MathUtils::Max((int)iy - Range, 0); y <= MathUtils::Min((int)iy + Range, (int)m_streetMap->GetHeight() - 1); y++)
	{
		for (int x = MathUtils::Max((int)ix - Range, 0); x <= MathUtils::Min((int)ix + Range, (int)m_streetMap->GetWidth() - 1); x++)
		{
			//m_streetSegments[y * m_streetMap->GetWidth() + x]->SetVisibility(true);
			m_pedsManager->NotifyStreetSegmentVisibilityChanged(m_streetSegments[y * m_streetMap->GetWidth() + x]);
		}
	}
}

bool Street::GetCollistion(const sm::Vec3 &pStart, const sm::Vec3 &pEnd, sm::Vec3 &collisionPoint, sm::Vec3 &normal)
{
	static BoxCollider streetSegmentCollider(sm::Vec3(0, 0, 0), sm::Vec3(10, 0, 10));

	StreetSegment *seg = GetSegmentAtPosition(pEnd);
	if (seg == NULL)
		return false;

	StreetSegment *segs[9];

	segs[0] = seg;
	segs[1] = GetSegment(seg->CoordX() - 1, seg->CoordY());
	segs[2] = GetSegment(seg->CoordX() + 1, seg->CoordY());
	segs[3] = GetSegment(seg->CoordX(), seg->CoordY() - 1);
	segs[4] = GetSegment(seg->CoordX(), seg->CoordY() + 1);

	segs[5] = GetSegment(seg->CoordX() + 1, seg->CoordY() + 1);
	segs[6] = GetSegment(seg->CoordX() + 1, seg->CoordY() - 1);
	segs[7] = GetSegment(seg->CoordX() - 1, seg->CoordY() + 1);
	segs[8] = GetSegment(seg->CoordX() - 1, seg->CoordY() - 1);
	 
	float closestDistance = 99999999.0f;
	bool collision = false;
	normal.Set(0, 0, 0);

	for (int i = 0; i < 9; i++)
	{
		if (segs[i] == NULL)
			continue;

		if (!segs[i]->GetStreetPiece()->IsCollider())
			continue;

		sm::Matrix segWorldInv = segs[i]->GetWorldTransform().GetInversed(); // TODO: performance

		sm::Vec3 localPStart = segWorldInv * pStart;
		sm::Vec3 localPEnd = segWorldInv * pEnd;

		sm::Vec3 cPoint;
		sm::Vec3 cNorm;

		if (streetSegmentCollider.CheckCollision(localPStart, localPEnd, cPoint, cNorm))
		{
			float dist = (localPStart - cPoint).GetLength();
			if (dist < closestDistance)
			{
				closestDistance = dist;
				collision = true;

				collisionPoint = segs[i]->GetWorldTransform() * cPoint;

				sm::Matrix a = segs[i]->GetWorldTransform();
				a.a[12] = 0.0f;
				a.a[13] = 0.0f;
				a.a[14] = 0.0f;
				normal += a * cNorm;
			}
		}
	}
	
	//normal.Normalize();

	return collision;

/*
	StreetSegment *seg = GetSegmentAtPosition(pEnd);
	if (seg == NULL)
		return false;

	if (!seg->GetStreetPiece()->IsCollider())
		return false;

	sm::Vec3 localPoint = pEnd - seg->GetPivotPosition();

	normal.Set(MathUtils::Sign(localPoint.x), 0, MathUtils::Sign(localPoint.z));

	if (MathUtils::Abs(localPoint.x) >= MathUtils::Abs(localPoint.z))
	{
		normal.z = 0.0f;
		collisionPoint.Set(pEnd.x - ((5.0f - MathUtils::Abs(localPoint.x)) * normal.x), 0, pEnd.z);
	}
	else
	{
		normal.x = 0.0f;
		collisionPoint.Set(pEnd.x, 0, pEnd.z - ((5.0f - MathUtils::Abs(localPoint.z)) * normal.z));
	}

	return true;
	*/
}

