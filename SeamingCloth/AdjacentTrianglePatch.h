#pragma once

#include"Particle.h"
#include"LyraFunction.h"

namespace Lyra
{
	template<typename T> class AdjacentTrianglePatch;
}

namespace Lyra
{
	template<typename T>
	class AdjacentTrianglePatch
	{
	public:
		AdjacentTrianglePatch() = default;
		~AdjacentTrianglePatch() = default;

		AdjacentTrianglePatch(particle_pt<T> pp0, particle_pt<T> pp1, particle_pt<T> pp2, particle_pt<T> pp3,
							  uint32 ind0, uint32 ind1, uint32 ind2, uint32 ind3,T bc,T dbc);

	public:
		/*Simulation Functions*/
		void ExplicitBendingForce();
		void ExplicitDampingBendingForce();

	private:
		T bendingCoefficent, dampingBendCoefficent;

		particle_pt<T> p0, p1, p2, p3;
		uint32 ind0, ind1, ind2, ind3;

		T restAngle;
	};
}

template<typename T>
Lyra::AdjacentTrianglePatch<T>::AdjacentTrianglePatch(particle_pt<T> pp0, particle_pt<T> pp1,
													  particle_pt<T> pp2, particle_pt<T> pp3,
													  uint32 ind0, uint32 ind1, 
													  uint32 ind2, uint32 ind3, T bc, T dbc)
{
	p0 = pp0; p1 = pp1; p2 = pp2; p3 = pp3;
	this->ind0 = ind0; this->ind1 = ind1; this->ind2 = ind2; this->ind3 = ind3;

	bendingCoefficent = bc;
	dampingBendCoefficent = dbc;

	vec3<T> e02 = p0->position - p2->position;
	vec3<T> e03 = p0->position - p3->position;
	vec3<T> e12 = p1->position - p2->position;
	vec3<T> e13 = p1->position - p3->position;

	vec3<T> N1 = e02.cross(e03);
	vec3<T> N2 = e13.cross(e12);

	vec3<T> n1 = N1.normalized(); vec3<T> n2 = N2.normalized();

	restAngle = SafeACos(n1.dot(n2));
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::ExplicitBendingForce()
{
	//printf_s("23333\n");
	vec3<T> e02 = p0->position - p2->position;
	vec3<T> e03 = p0->position - p3->position;
	vec3<T> e12 = p1->position - p2->position;
	vec3<T> e13 = p1->position - p3->position;
	vec3<T> E = p3->position - p2->position;

	vec3<T> N1 = e02.cross(e03);
	vec3<T> N2 = e13.cross(e12);

	T El = E.norm();
	T N1l2 = std::pow(N1.norm(), 2);
	T N2l2 = std::pow(N2.norm(), 2);

	vec3<T> u1 = N1 * (El / N1l2);
	vec3<T> u2 = N2 * (El / N2l2);
	vec3<T> u3 = N1 * e03.dot(E) / (El*N1l2) + N2 * e13.dot(E) / (El*N2l2);
	vec3<T> u4 = -N1 * e02.dot(E) / (El*N1l2) - N2 * e12.dot(E) / (El*N2l2);

	vec3<T> n1 = N1.normalized(); vec3<T> n2 = N2.normalized();
	//这里可能得到-0，结果就得到sqrt()得到NaN，所以加个abs()
	//判断是不是NaN的方法,if(A!=A){......}
	//T sinHalf = sqrt(std::abs((1.0f - n1.dot(n2)) / 2.0f));
	T sinHalf = (std::sin(SafeACos(n1.dot(n2))) - std::sin(restAngle));
	if (!IsZero(sinHalf)) {
		if (n1.cross(n2).dot(E.normalized()) < 0) { sinHalf = -sinHalf; }
		T bendFactor = bendingCoefficent * std::pow(El, 2)*sinHalf / (N1l2 + N2l2);

		p0->ApplyForce(u1 * bendFactor);
		p1->ApplyForce(u2 * bendFactor);
		p2->ApplyForce(u3 * bendFactor);
		p3->ApplyForce(u4 * bendFactor);
	}
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::ExplicitDampingBendingForce()
{
	vec3<T> e02 = p0->position - p2->position;
	vec3<T> e03 = p0->position - p3->position;
	vec3<T> e12 = p1->position - p2->position;
	vec3<T> e13 = p1->position - p3->position;
	vec3<T> E = p3->position - p2->position;

	vec3<T> N1 = e02.cross(e03);
	vec3<T> N2 = e13.cross(e12);

	T El = E.norm();
	T N1l2 = std::pow(N1.norm(), 2);
	T N2l2 = std::pow(N2.norm(), 2);

	vec3<T> u1 = N1 * (El / N1l2);
	vec3<T> u2 = N2 * (El / N2l2);
	vec3<T> u3 = N1 * e03.dot(E) / (El*N1l2) + N2 * e13.dot(E) / (El*N2l2);
	vec3<T> u4 = -N1 * e02.dot(E) / (El*N1l2) - N2 * e12.dot(E) / (El*N2l2);

	vec3<T> n1 = N1.normalized(); vec3<T> n2 = N2.normalized();

	T dThetaDT = u1.dot(p0->velocity) + u2.dot(p1->velocity) + u3.dot(p2->velocity) + u4.dot(p3->velocity);
	T bdFactor = -dampingBendCoefficent * El*dThetaDT;
	p0->ApplyForce(u1*bdFactor);
	p1->ApplyForce(u2*bdFactor);
	p2->ApplyForce(u3*bdFactor);
	p3->ApplyForce(u4*bdFactor);
}
