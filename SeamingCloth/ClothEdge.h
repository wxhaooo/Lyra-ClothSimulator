#pragma once
#include"Particle.h"

namespace Lyra
{
	template<typename T> struct ClothEdge;
}

namespace Lyra
{
	template<typename T>
	struct ClothEdge
	{
	public:
		ClothEdge() = default;
		ClothEdge(particle_pt<T> pp0, particle_pt<T> pp1) :p0(pp0), p1(pp1) {}

		particle_pt<T> p0, p1;
	};
}