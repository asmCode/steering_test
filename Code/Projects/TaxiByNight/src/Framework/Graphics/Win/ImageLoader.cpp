#ifdef _WIN32

#include "../ImageLoader.h"
#include "Utils/StringUtils.h"
#include <IO/Path.h>
#include <libpng/png.h>
#include <string>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

bool ReadPNG(const std::string &fileName,
	uint8_t *&data,
	uint32_t &width,
	uint32_t &height,
	uint32_t &bytesCount)
{
	png_structp png_ptr;
	png_infop info_ptr;

	FILE *fp = fopen(fileName.c_str(), "rb");
	if (fp == NULL)
		return false;

	png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL)
	{
		fclose(fp);
		return false;
	}

	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL)
	{
		fclose(fp);
		png_destroy_read_struct(&png_ptr, NULL, NULL);
		return false;
	}

	if (setjmp(png_jmpbuf(png_ptr)))
	{
		fclose(fp);
		png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
		return false;
	}

	png_init_io(png_ptr, fp);

	png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_SCALE_16 | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_GRAY_TO_RGB, NULL);

	width = png_get_image_width(png_ptr, info_ptr);
	height = png_get_image_height(png_ptr, info_ptr);
	bytesCount = png_get_channels(png_ptr, info_ptr);
	png_bytepp rows = png_get_rows(png_ptr, info_ptr);

	uint32_t rowSize = width * bytesCount;
	uint32_t imageSize = height * rowSize;

	data = new uint8_t[imageSize];

	for (uint32_t i = 0; i < height; i++)
		memcpy(data + i * rowSize, rows[i], rowSize);

	png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

	fclose(fp);

	return true;
}

bool ImageLoader::LoadFromFile(const std::string &path,
							     uint8_t *&data,
								 uint32_t &width,
								 uint32_t &height,
								 uint32_t &bytesCount)
{
	assert(!path.empty());
	if (path.empty())
		return false;

	Path filePath(path);
	if (StringUtils::LowerCase(filePath.GetExt()) != "png")
		return false;

	return ReadPNG(path, data, width, height, bytesCount);

	/*HDC hdc = GetDC(NULL);
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			SetPixel(hdc, x, y, RGB(data[(y * width * bytesCount) + x * bytesCount + 0],
				data[(y * width * bytesCount) + x * bytesCount + 1],
								    data[(y * width * bytesCount) + x * bytesCount + 2]));
		}
	}*/
}

#else

#include "../ImageLoader.h"
#include <Utils/StringUtils.h>
#include <IO/Path.h>
#include <FMedia.h>
#include <FBase.h>
#include <stdio.h>
#include <memory.h>
#include <stddef.h>
#include <assert.h>

using namespace Tizen::Base;
using namespace Tizen::Graphics;

bool ImageLoader::LoadFromFile(const std::string &path,
							     uint8_t *&data,
								 uint32_t &width,
								 uint32_t &height,
								 uint32_t &bytesCount)
{
	assert(!path.empty());
	if (path.empty())
		return false;

	Path filePath(path);
	if (StringUtils::LowerCase(filePath.GetExt()) != "png" &&
		StringUtils::LowerCase(filePath.GetExt()) != "jpg")
		return false;

	Tizen::Media::Image image;
	image.Construct();

	Bitmap *bitmap = image.DecodeN(path.c_str(), BITMAP_PIXEL_FORMAT_R8G8B8A8);
	if (bitmap == null)
	{
		AppLogException("Couldn't load image");
		return false;
	}

	int w, h;

	ByteBuffer *buffer = image.DecodeToBufferN(path.c_str(), BITMAP_PIXEL_FORMAT_R8G8B8A8, w, h);

	width = w;
	height = h;
	bytesCount = 4;

	data = new uint8_t[width * height * bytesCount];

	memcpy(data, buffer->GetPointer(), width * height * bytesCount);

	delete buffer;
	delete bitmap;

	return true;
}

#endif
