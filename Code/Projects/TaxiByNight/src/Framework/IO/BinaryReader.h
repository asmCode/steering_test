#ifndef BINARY_READER
#define BINARY_READER

#include <string>
#include <memory.h>

class BinaryReader
{
public:
	BinaryReader(const void *mem);
	
	template <typename T>
	T Read()
	{
		T val;
		memcpy(&val, m_mem, sizeof(T));
		m_mem = (T*)m_mem + 1;
		return val;
	}
	
	void ReadBuffer(char *buff, unsigned size);

private:
	const void *m_mem;
};

template <>
std::string BinaryReader::Read<std::string>();

#endif // BINARY_READER

