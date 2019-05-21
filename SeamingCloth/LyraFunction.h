#pragma once

#include<cmath>
#include<typeinfo>

#include"Common.h"


//函数
namespace Lyra
{
	//局部变量不要用引用返回，直接值返回，所以会拷贝两次，一次拷贝临时变量，一次拷给需要赋值的变量
	//想要局部变量的值，除了用值拷贝的方式以外，另外一头用右值引用也行，可以减少一次构造和析构，效率更高些
	template<typename T>
	T Min(T& t1, T& t2, T& t3)
	{
		return t1 < t2 ? t1 < t3 ? t1 : t3
			: t2 < t3 ? t2 : t3;

		/*if (t1 < t2)
			return t1 < t3 ? t1 : t3;
		else
			return t2 < t3 ? t2 : t3;*/
	}

	template<typename T>
	T Max(T& t1, T& t2, T& t3)
	{
		return  t1 > t2 ? (t1 > t3 ? t1 : t3)
			: (t2 > t3 ? t2 : t3);
	}

	template<typename T>
	vec3<T> Min(vec3<T>& v1, vec3<T>& v2)
	{
		vec3<T> tmp;
		tmp(0) = v1(0) < v2(0) ? v1(0) : v2(0);
		tmp(1) = v1(1) < v2(1) ? v1(1) : v2(1);
		tmp(2) = v1(2) < v2(2) ? v1(2) : v2(2);

		return tmp;
	}

	template<typename T>
	const vec3<T> Min(const vec3<T>& v1,const vec3<T>& v2)
	{
		vec3<T> tmp;
		tmp(0) = v1(0) < v2(0) ? v1(0) : v2(0);
		tmp(1) = v1(1) < v2(1) ? v1(1) : v2(1);
		tmp(2) = v1(2) < v2(2) ? v1(2) : v2(2);

		return tmp;
	}

	template<typename T>
	vec3<T> Min(vec3<T>& v1, vec3<T>& v2, vec3<T>& v3) 
	{
		vec3<T> tmp;
		tmp(0) = Min(v1(0), v2(0), v3(0));
		tmp(1) = Min(v1(1), v2(1), v3(1));
		tmp(2) = Min(v1(2), v2(2), v3(2));

		return tmp;
	}

	template<typename T>
	vec3<T> Max(vec3<T>& v1, vec3<T>& v2)
	{
		vec3<T> tmp;
		tmp(0) = v1(0) > v2(0) ? v1(0) : v2(0);
		tmp(1) = v1(1) > v2(1) ? v1(1) : v2(1);
		tmp(2) = v1(2) > v2(2) ? v1(2) : v2(2);

		return tmp;
	}

	template<typename T>
	const vec3<T> Max(const vec3<T>& v1, const vec3<T>& v2)
	{
		vec3<T> tmp;
		tmp(0) = v1(0) > v2(0) ? v1(0) : v2(0);
		tmp(1) = v1(1) > v2(1) ? v1(1) : v2(1);
		tmp(2) = v1(2) > v2(2) ? v1(2) : v2(2);

		return tmp;
	}

	template<typename T>
	vec3<T> Max(vec3<T>& v1, vec3<T>& v2, vec3<T>& v3)
	{
		vec3<T> tmp;
		tmp(0) = Max(v1(0), v2(0), v3(0));
		tmp(1) = Max(v1(1), v2(1), v3(1));
		tmp(2) = Max(v1(2), v2(2), v3(2));

		return tmp;
	}

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
	bool IsZeroHp(T v)
	{
		if (std::abs(v) < 10e-12) return true;
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