#include "SpritesMap.h"
#include <Graphics/TexPart.h>
#include <Graphics/Content/Content.h>
#include <XML/XMLLoader.h>
#include <XML/XMLNode.h>
#include <Core/stdint.h>
#include <stdio.h>

SpritesMap::SpritesMap()
{
}

SpritesMap::~SpritesMap()
{
	std::map<std::string, TexPart*>::iterator it;
	for (it = m_sprites.begin(); it != m_sprites.end(); it++)
		delete it->second;
}

bool SpritesMap::LoadFromFile(const std::string &path, Content *content)
{
	XMLNode *root = XMLLoader::LoadFromFile(path);
	if (root == NULL)
		return false;

	for (uint32_t i = 0; i < root->GetChildrenCount(); i++)
	{
		XMLNode &sprite = (*root)[i];

		std::string name = sprite.GetAttribAsString("name");
		std::string texName = sprite.GetAttribAsString("file");
		std::string bounds = sprite.GetAttribAsString("bounds");

		uint32_t left;
		uint32_t top;
		uint32_t width;
		uint32_t height;

		sscanf(bounds.c_str(), "%d,%d,%d,%d", &left, &top, &width, &height);

		Texture *tex = content->Get<Texture>(texName);
		assert(tex != NULL && "no such resource name");

		//TexPart *texPart = new TexPart(tex, sm::Rect<int>(left, top, width, height));
		TexPart *texPart = new TexPart(tex, sm::Rect<int>(left, tex->GetHeight() - top - height, width, height)); // TODO: nie zawsze mozna obracac texture
																												  // jesli tekstura ma wiele innych tekstur, to to nie dziala
		m_sprites[name] = texPart;
	}

	return true;
}

TexPart* SpritesMap::GetTexPart(const std::string &name)
{
	std::map<std::string, TexPart*>::iterator it = m_sprites.find(name);
	if (it == m_sprites.end())
		return NULL;

	return it->second;
}

