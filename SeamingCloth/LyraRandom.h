#pragma once
#include<random>
#include<ctime>

#include "Common.h"

namespace Lyra
{
	std::default_random_engine randomEngine(time(0));
	template<typename T>
	using randomDist = std::uniform_real_distribution<T>;

	template<typename T>
	glm::vec<3, T> ColorGenerator()
	{
		randomDist<T> color(T(0), T(1));
		return glm::vec<3, T>(color(randomEngine), color(randomEngine), color(randomEngine));
	}
}
