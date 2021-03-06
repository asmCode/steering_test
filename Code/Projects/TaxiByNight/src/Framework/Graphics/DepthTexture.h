#pragma once

class DepthTexture
{
private:
	unsigned	id;
	int			width;
	int			height;

public:
	DepthTexture(int width, int height);
	~DepthTexture();

	int GetWidth() const;
	int GetHeight() const;

	void BindTexture();
	unsigned GetId();
	void SetTextureData(const unsigned char *data);
};
