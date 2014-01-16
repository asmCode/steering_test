#include "InterfaceProvider.h"
#include <stddef.h>
#include <assert.h>

IGraphicsEngine *InterfaceProvider::m_graphicsEngine;
Content *InterfaceProvider::m_content;
SpriteBatch *InterfaceProvider::m_spriteBatch;
SpritesMap* InterfaceProvider::m_spritesMap;
InterfaceProvider::FontsMap InterfaceProvider::m_fonts;

IGraphicsEngine* InterfaceProvider::GetGraphicsEngine()
{
	assert(m_graphicsEngine != NULL);
	return m_graphicsEngine;
}

Content* InterfaceProvider::GetContent()
{
	return m_content;
}

SpriteBatch* InterfaceProvider::GetSpriteBatch()
{
	return m_spriteBatch;
}

SpritesMap* InterfaceProvider::GetSpritesMap()
{
	return m_spritesMap;
}

FontRenderer* InterfaceProvider::GetFontRenderer(const std::string &fontName)
{
	FontsMap::iterator it = m_fonts.find(fontName);
	if (it == m_fonts.end())
	{
		assert(false);
		return NULL;
	}

	return (*it).second;
}

