#ifndef ICOLLIDER_HOLDER
#define ICOLLIDER_HOLDER

class Collider;

class IColliderHolder
{
public:
	virtual ~IColliderHolder() {}

	virtual const Collider* GetCollider() const = 0;
};

#endif // !ICOLLIDER_HOLDER
