#ifndef STREET
#define STREET

#include <Math/Matrix.h>
#include <vector>
#include <Core/stdint.h>

class Model;
class Texture;
class StreetMap;
class StreetPiece;
class StreetSegment;
class PedsManager;
class TrafficManager;

class Street
{
public:
	static Street *Instance;

	Street(PedsManager *pedsManager, TrafficManager* trafficManager);
	~Street();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);
	void DrawStreetLights(float time, float seconds);
	void DrawStreetLightsSprites(float time, float seconds);

	StreetPiece* GetStreetPiece(uint8_t type);

	void SetTaxiPosition(const sm::Vec3 &taxiPosition);

	void GetSegmentCoords(const sm::Vec3 &position, int32_t &x, int32_t &y) const;
	StreetSegment* GetSegmentAtPosition(const sm::Vec3 &position) const;
	StreetSegment* GetSegment(int x, int y) const;
	void SetInitialVisibility(const sm::Vec3 &taxiPosition);

	StreetSegment *GetRandomPavement();

	bool GetCollistion(const sm::Vec3 &pStart, const sm::Vec3 &pEnd, sm::Vec3 &collisionPoint, sm::Vec3 &normal);

private:
	static const int Range = 2;

	PedsManager *m_pedsManager;
	TrafficManager *m_trafficManager;

	sm::Vec3 m_taxiPosition;
	Model *m_streetModel;
	Model *m_skycrapper1;
	Model *m_skycrapper2;
	Model *m_skycrapper3;

	Model *m_streetLightsModel;

	std::vector<StreetSegment*> m_pavementSegments;

	Texture *m_skycrapperTexture1;
	Texture *m_skycrapperTexture2;
	Texture *m_skycrapperTexture3;
	Texture *m_pavementTexture;

	Texture *m_lightTexture;

	StreetMap *m_streetMap;

	StreetPiece *m_streetPieces[256];

	StreetSegment **m_streetSegments;

	StreetSegment *m_lastTaxiSegment;
};

#endif // STREET

