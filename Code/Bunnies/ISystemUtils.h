#ifndef ISYSTEMUTILS_H_
#define ISYSTEMUTILS_H_

class ISystemUtils
{
public:
	virtual ~ISystemUtils() {}

	virtual void QuitApplication() = 0;
};

#endif

