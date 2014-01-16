#ifndef SPRITES_MAP
#define SPRITES_MAP

#include <map>
#include <string>

class TexPart;
class Content;

class SpritesMap
{
public:
	SpritesMap();
	~SpritesMap();

	bool LoadFromFile(const std::string &path, Content *content);
	TexPart* GetTexPart(const std::string &name);

private:
	std::map<std::string, TexPart*> m_sprites;
};

#endif // SPRITES_MAP