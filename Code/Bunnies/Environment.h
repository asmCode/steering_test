#ifndef SSG_ENVIRONMENT_H
#define SSG_ENVIRONMENT_H

#include <string>

namespace TaxiGame
{

class Environment
{
private:
	static Environment *m_instance;

	unsigned m_screenWidth;
	unsigned m_screenHeight;
	std::string m_iosVersion;
	std::string m_basePath;
	std::string m_writePath;
	
public:
	Environment();
	static Environment* GetInstance();
	
	void SetBasePath(const std::string &basePath);
	void SetWritePath(const std::string &writePath);
	void SetScreenSize(unsigned width, unsigned height);
	void SetIOSVersion(const std::string &version);
	
	unsigned GetScreenWidth() const;
	unsigned GetScreenHeight() const;
	std::string GetIOSVersion() const;
	std::string GetBasePath() const;
	std::string GetWritePath() const;
};

#endif

}
