#pragma once

#include<cmath>
#include<typeinfo>


//º¯Êý
namespace Lyra
{
	template<typename T>
	T Angle2Radian(T angle) { return PI / 180. * angle; }

	template<typename T>
	T Radian2Angle(T radian) { return 180. / PI * radian; }

	template<typename T>
	bool IsZero(T v)
	{
		if (std::abs(v) < 10e-8) return true;
		return false;
	}

	template<typename T>
	T SafeACos(T radian)
	{
		if (typeid(T) == typeid(float)) {

			if (radian < -1.f) radian = -1.f;
			else if (radian > 1.f) radian = 1.f;
		}

		if (typeid(T) == typeid(double)) {

			if (radian < -1.) radian = -1.;
			else if (radian > 1.) radian = 1.;
		}

		return std::acos(radian);
	}
}