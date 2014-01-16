#include "StreetPiece.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "StreetMap.h"
#include "Environment.h"

#include <Graphics/Model.h>
#include <Graphics/Texture.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

StreetPiece::StreetPiece(
		StreetPiece::PieceType pieceType,
		Model *model,
		Texture *texture,
		const sm::Matrix &world)
{
	m_pieceType = pieceType;
	m_model = model;
	m_texture = texture;
	m_world = world;
}

void StreetPiece::Draw(const sm::Matrix &transform)
{
	DrawingRoutines::DrawStreet(
		m_model,
		m_texture,
		transform * m_world);
}

const sm::Matrix& StreetPiece::GetTransform() const
{
	return m_world;
}

bool StreetPiece::HasPavement() const
{
	return
		m_pieceType == StreetPiece::PieceType_StraightHori_1 ||
		m_pieceType == StreetPiece::PieceType_StraightVert_1 ||
		m_pieceType == StreetPiece::PieceType_TDown_1 ||
		m_pieceType == StreetPiece::PieceType_TLeft_1 ||
		m_pieceType == StreetPiece::PieceType_TRight_1 ||
		m_pieceType == StreetPiece::PieceType_TUp_1 ||
		m_pieceType == StreetPiece::PieceType_TurnDownLeft_1 ||
		m_pieceType == StreetPiece::PieceType_TurnDownRight_1 ||
		m_pieceType == StreetPiece::PieceType_TurnUpLeft_1 ||
		m_pieceType == StreetPiece::PieceType_TurnUpRight_1;
}

bool StreetPiece::IsCollider() const
{
	return
		m_pieceType == StreetPiece::PieceType_Skycrapper_1 ||
		m_pieceType == StreetPiece::PieceType_Skycrapper_2 ||
		m_pieceType == StreetPiece::PieceType_Skycrapper_3 ||
		m_pieceType == StreetPiece::PieceType_Pavement;
}

