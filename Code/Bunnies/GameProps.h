#pragma once

class GameProps
{
public:
	// gameplay
	static const float RespawnDelay;
	static const float ReproductionDelay;

	static const float FarmerMoveSpeed;
	static const float HealthyBunnyWalkSpeed;
	static const float HealthyBunnyRunSpeed;
	static const float HealthyBunnyResistance;
	static const float InfectedBunnyHuntingSpeed;
	static const float InfectedBunnyRestingSpeed;

	static const float DelayBetweenWalkJump;
	static const float DelayBetweenRunJump;

	static const float DangerShotAngle; // value of cos(angle)
	static const float HealthyBunnyDistanceToFarmer;
	//static const float ReproductionTime;
	static const float ReproductionHipCycles;
	static const float FuckingHipCycles;
	static const float RestingAfterReproduction;
	static const float GrowingUpTime;
	static const float BorningTime;
	static const float SpawningTime;
	static const float DistanceToFuck;
	static const float RestingAfterFuckingTimeFrom;
	static const float RestingAfterFuckingTimeTo;
	static const float RestingAfterTryingToFuckTimeFrom;
	static const float RestingAfterTryingToFuckTimeTo;
	static const float RestingAfterChangedToInfectedTime;
	static const float RefreshNewTargetPositionFrom;
	static const float RefreshNewTargetPositionTo;
	static const float FuckingTime;
	static const float RunningAwayDistance;
	static const float SickBunnySpeed;
	static const float ChangingToInfectedTime;

	static const float ShotgunShotDelay;
	static const float ShotgunRange;
	static const float ShotgunFieldOfDamage;
	static const float ShotgunDamage;

	static const float PlayerHeight;
	static const float KickMinDistance;
	static const float KickMaxDistance;
	static const float KickFOV;
	static const float KickCooldown;

	// GUI
	static const float SplashScreenDuration;

};

