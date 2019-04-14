#pragma once
namespace Lyra
{
	struct PlaneForceSwitch;
	struct SpaceForceSwitch;
}

struct Lyra::PlaneForceSwitch
{
	bool enableStretchForce;
	bool enableShearForce;

	bool enableDampingStretchForce;
	bool enableDampingShearForce;

	PlaneForceSwitch()
	{
		enableStretchForce = true;
		enableShearForce = true;

		enableDampingStretchForce = true;
		enableDampingShearForce = true;
	}

	PlaneForceSwitch &operator=(PlaneForceSwitch &forceSwitch)
	{
		enableStretchForce = forceSwitch.enableStretchForce;
		enableShearForce = forceSwitch.enableShearForce;

		enableDampingStretchForce = forceSwitch.enableDampingStretchForce;
		enableDampingShearForce = forceSwitch.enableDampingShearForce;

		return *this;
	}
};

struct Lyra::SpaceForceSwitch
{
	bool enableBendingForce;
	bool enableDampingBendingForce;

	SpaceForceSwitch()
	{
		enableBendingForce = true;
		enableDampingBendingForce = true;
	}

	SpaceForceSwitch &operator=(SpaceForceSwitch &forceSwitch)
	{
		enableBendingForce = forceSwitch.enableBendingForce;
		enableDampingBendingForce = forceSwitch.enableDampingBendingForce;

		return *this;
	}
};