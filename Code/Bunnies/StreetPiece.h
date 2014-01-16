#ifndef STREET_PIECE
#define STREET_PIECE

#include <Math/Matrix.h>
#include <Core/stdint.h>

class Model;
class Texture;
class StreetMap;

class StreetPiece
{
public:
	enum PieceType
	{
		PieceType_Pavement = 0,
		PieceType_StraightHori_1,
		PieceType_StraightVert_1,
		PieceType_TurnUpRight_1,
		PieceType_TurnUpLeft_1,
		PieceType_TurnDownRight_1,
		PieceType_TurnDownLeft_1,
		PieceType_Cross_1,
		PieceType_TUp_1,
		PieceType_TDown_1,
		PieceType_TLeft_1,
		PieceType_TRight_1,

		PieceType_Skycrapper_1,
		PieceType_Skycrapper_2,
		PieceType_Skycrapper_3,
	};

	StreetPiece(
		PieceType pieceType,
		Model *model,
		Texture *texture,
		const sm::Matrix &world);

	void Draw(const sm::Matrix &transform);

	bool HasPavement() const;
	bool HasRoad() const;
	bool IsCollider() const;
	PieceType GetType() const;

	const sm::Matrix& GetTransform() const;

	Model *m_model;

private:
	PieceType m_pieceType;
	Texture *m_texture;
	sm::Matrix m_world;
};

#endif // STREET_PIECE
