#pragma once

#include"Particle.h"
#include"LyraFunction.h"
#include"Common.h"

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

		void ImplicitDampingBendingForce(T delta_t, VelocityUpdate updateCat);

	private:
		void CollectMassInvMat(mat<T, 12, 12>& massMat);
		void CollectAMat(mat<T, 12, 12>& Amat, mat<T, 12, 12>& bendingMat,  T delta_t);
		void CollectDampingBendingMat(mat<T, 12, 12>& bendingMat, mat<T, 12, 12>& massInvMat, VelocityUpdate updateCat);
		void CollectbVec(vec<T, 12>& bVec, VelocityUpdate updateCat);

		void SolveLinearSystem(vec<T, 12>& xVec, mat<T, 12, 12>& AMat, vec<T, 12>& bVec);

	private:
		T bendingCoefficent, dampingBendCoefficent;

		particle_pt<T> p0, p1, p2, p3;
		uint32 ind0, ind1, ind2, ind3;

		T restAngle;

		mat<T, 12, 12> massInvMat;
		mat<T, 12, 12> AMat;
		vec<T, 12> bVec;
		vec<T, 12> xVec;
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
	vec3<T> e02 = p0->pseudoPosition - p2->pseudoPosition;
	vec3<T> e03 = p0->pseudoPosition - p3->pseudoPosition;
	vec3<T> e12 = p1->pseudoPosition - p2->pseudoPosition;
	vec3<T> e13 = p1->pseudoPosition - p3->pseudoPosition;
	vec3<T> E = p3->pseudoPosition - p2->pseudoPosition;
	//vec3<T> e02 = p0->position - p2->position;
	//vec3<T> e03 = p0->position - p3->position;
	//vec3<T> e12 = p1->position - p2->position;
	//vec3<T> e13 = p1->position - p3->position;
	//vec3<T> E = p3->position - p2->position;

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
	T sinHalf = sqrt(std::abs((1.0f - n1.dot(n2)) / 2.0f));
	//std::cout << sinHalf << "\n";
	//T sinHalf = (std::sin(SafeACos(n1.dot(n2))) - std::sin(restAngle));
	//std::cout << sinHalf << "\n";
	//std::cout << restAngle << "\n";
	if (!IsZero(sinHalf)) {
		if (n1.cross(n2).dot(E.normalized()) < 0) { sinHalf = -sinHalf; }
		T bendFactor = bendingCoefficent * std::pow(El, 2) * sinHalf / (N1l2 + N2l2);

		//std::cout << sinHalf << "\n";
		/*std::cout << u1.norm() << "\n";
		std::cout << u2.norm() << "\n";
		std::cout << u3.norm() << "\n";
		std::cout << u4.norm() << "\n";*/
		//std::cout << bendFactor << "\n";
		/*std::cout << (u1 * bendFactor).norm() / p0->mass << "\n";
		std::cout << (u2 * bendFactor).norm() / p1->mass << "\n";
		std::cout << (u3 * bendFactor).norm() / p2->mass << "\n";
		std::cout << (u4 * bendFactor).norm() / p3->mass << "\n";*/
		//std::cout << "23333\n";
		p0->ApplyForce(u1 * bendFactor);
		p1->ApplyForce(u2 * bendFactor);
		p2->ApplyForce(u3 * bendFactor);
		p3->ApplyForce(u4 * bendFactor);
	}
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::ExplicitDampingBendingForce()
{
	/*vec3<T> e02 = p0->pseudoPosition - p2->pseudoPosition;
	vec3<T> e03 = p0->pseudoPosition - p3->pseudoPosition;
	vec3<T> e12 = p1->pseudoPosition - p2->pseudoPosition;
	vec3<T> e13 = p1->pseudoPosition - p3->pseudoPosition;
	vec3<T> E = p3->pseudoPosition - p2->pseudoPosition;*/
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

	//T dThetaDT = u1.dot(p0->velocity) + u2.dot(p1->velocity) + u3.dot(p2->velocity) + u4.dot(p3->velocity);
	T dThetaDT = u1.dot(p0->middleVelocity) + u2.dot(p1->middleVelocity) + u3.dot(p2->middleVelocity) + u4.dot(p3->middleVelocity);
	T bdFactor = -dampingBendCoefficent * El*dThetaDT;
	p0->ApplyForce(u1*bdFactor);
	p1->ApplyForce(u2*bdFactor);
	p2->ApplyForce(u3*bdFactor);
	p3->ApplyForce(u4*bdFactor);
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::ImplicitDampingBendingForce(T delta_t, VelocityUpdate updateCat)
{
	massInvMat.setZero();
	AMat.setZero();
	bVec.setZero();
	xVec.setZero();

	mat<T, 12, 12> bendingMat = mat<T, 12, 12>::Zero();

	CollectMassInvMat(massInvMat);
	CollectDampingBendingMat(bendingMat, massInvMat, updateCat);
	//std::cout << bendingMat << "\n";
	CollectAMat(AMat, bendingMat, delta_t);
	CollectbVec(bVec, updateCat);
	SolveLinearSystem(xVec, AMat, bVec);

	if (updateCat == VelocityUpdate::MIDDLE_VELOCITY) {

		p0->middleVelocity = xVec.block<3, 1>(0, 0);
		p1->middleVelocity = xVec.block<3, 1>(3, 0);
		p2->middleVelocity = xVec.block<3, 1>(6, 0);
		p3->middleVelocity = xVec.block<3, 1>(9, 0);

		/*p0->velocity = xVec.block<3, 1>(0, 0);
		p1->velocity = xVec.block<3, 1>(3, 0);
		p2->velocity = xVec.block<3, 1>(6, 0);
		p3->velocity = xVec.block<3, 1>(9, 0);*/
		//accumulation of delta v
		/*p0->middleVelocity += (xVec.block<3, 1>(0, 0) - p0->velocity);
		p1->middleVelocity += (xVec.block<3, 1>(3, 0) - p1->velocity);
		p2->middleVelocity += (xVec.block<3, 1>(6, 0) - p2->velocity);
		p3->middleVelocity += (xVec.block<3, 1>(9, 0) - p3->velocity);*/
	}
	else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		p0->middleVelocity = xVec.block<3, 1>(0, 0);
		p1->middleVelocity = xVec.block<3, 1>(3, 0);
		p2->middleVelocity = xVec.block<3, 1>(6, 0);
		p3->middleVelocity = xVec.block<3, 1>(9, 0);
		/*	p0->pseudoVelocity += (xVec.block<3, 1>(0, 0) - p0->middleVelocity);
			p1->pseudoVelocity += (xVec.block<3, 1>(3, 0) - p1->middleVelocity);
			p2->pseudoVelocity += (xVec.block<3, 1>(6, 0) - p2->middleVelocity);
			p3->pseudoVelocity += (xVec.block<3, 1>(9, 0) - p3->middleVelocity);*/
	}
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::CollectMassInvMat(mat<T, 12, 12>& massMat)
{
	T x0MassInv = p0->massInv;
	T x1MassInv = p1->massInv;
	T x2MassInv = p2->massInv;
	T x3MassInv = p3->massInv;

	massInvMat(0, 0) = massInvMat(1, 1) = massInvMat(2, 2) = x0MassInv;
	massInvMat(3, 3) = massInvMat(4, 4) = massInvMat(5, 5) = x1MassInv;
	massInvMat(6, 6) = massInvMat(7, 7) = massInvMat(8, 8) = x2MassInv;
	massInvMat(9, 9) = massInvMat(10, 10) = massInvMat(11, 11) = x3MassInv;

	//std::cout << x0MassInv << " " << x1MassInv << " " << x2MassInv << " " << x3MassInv << "\n";
	/*T x0Mass = p0->mass;
	T x1Mass = p1->mass;
	T x2Mass = p2->mass;
	T x3Mass = p3->mass;

	massInvMat(0, 0) = massInvMat(1, 1) = massInvMat(2, 2) = T(1) / x0Mass;
	massInvMat(3, 3) = massInvMat(4, 4) = massInvMat(5, 5) = T(1) / x1Mass;
	massInvMat(6, 6) = massInvMat(7, 7) = massInvMat(8, 8) = T(1) / x2Mass;
	massInvMat(9, 9) = massInvMat(10, 10) = massInvMat(11, 11) = T(1) / x3Mass;*/
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::CollectAMat(mat<T, 12, 12>& Amat, mat<T, 12, 12>& bendingMat, T delta_t)
{
	//std::cout << bendingMat << "\n";
	AMat = mat<T, 12, 12>::Identity() - 0.5 * delta_t * massInvMat * bendingMat;
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::CollectDampingBendingMat(mat<T, 12, 12>& bendingMat, mat<T, 12, 12>& massInvMat, VelocityUpdate updateCat)
{
	vec3<T> e02, e03, e12, e13, E;

	if (updateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		e02 = p0->position - p2->position;
		e03 = p0->position - p3->position;
		e12 = p1->position - p2->position;
		e13 = p1->position - p3->position;
		E = p3->position - p2->position;
	}
	else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		e02 = p0->pseudoPosition - p2->pseudoPosition;
		e03 = p0->pseudoPosition - p3->pseudoPosition;
		e12 = p1->pseudoPosition - p2->pseudoPosition;
		e13 = p1->pseudoPosition - p3->pseudoPosition;
		E = p3->pseudoPosition - p2->pseudoPosition;
	}

	vec3<T> N1 = e02.cross(e03);
	vec3<T> N2 = e13.cross(e12);

	T El = E.norm();
	T N1l2 = std::pow(N1.norm(), 2);
	T N2l2 = std::pow(N2.norm(), 2);

	vec3<T> u1 = N1 * (El / N1l2);
	vec3<T> u2 = N2 * (El / N2l2);
	vec3<T> u3 = N1 * e03.dot(E) / (El * N1l2) + N2 * e13.dot(E) / (El * N2l2);
	vec3<T> u4 = -N1 * e02.dot(E) / (El * N1l2) - N2 * e12.dot(E) / (El * N2l2);

	vec<T, 12> u;
	mat<T, 1, 12> uT;
	u.block<3, 1>(0, 0) = u1;
	u.block<3, 1>(3, 0) = u2;
	u.block<3, 1>(6, 0) = u3;
	u.block<3, 1>(9, 0) = u4;
	uT = u.transpose();

	T dampingFactor = - dampingBendCoefficent * El;
	//std::cout << dampingFactor << "\n";
	bendingMat = dampingFactor * u * uT;
	//std::cout << bendingMat << "\n";
	//std::cout << u * uT << "\n";
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::CollectbVec(vec<T, 12>& bVec, VelocityUpdate updateCat)
{
	vec3<T> x0Vel, x1Vel, x2Vel, x3Vel;

	if (updateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		x0Vel = p0->middleVelocity;
		x1Vel = p1->middleVelocity;
		x2Vel = p2->middleVelocity;
		x3Vel = p3->middleVelocity;
		/*x0Vel = p0->velocity;
		x1Vel = p1->velocity;
		x2Vel = p2->velocity;
		x3Vel = p3->velocity;*/
	}
	else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		x0Vel = p0->middleVelocity;
		x1Vel = p1->middleVelocity;
		x2Vel = p2->middleVelocity;
		x3Vel = p3->middleVelocity;
	}

	bVec.block<3, 1>(0, 0) = x0Vel;
	bVec.block<3, 1>(3, 0) = x1Vel;
	bVec.block<3, 1>(6, 0) = x2Vel;
	bVec.block<3, 1>(9, 0) = x3Vel;
}

template<typename T>
void Lyra::AdjacentTrianglePatch<T>::SolveLinearSystem(vec<T, 12>& xVec, mat<T, 12, 12>& AMat, vec<T, 12>& bVec)
{
	//std::cout << AMat.eigenvalues() << "\n";
	//std::cout << AMat << "\n";
	//std::cout << mat<T, 12, 12>::Identity().eigenvalues() << "\n";
	//mat<std::complex<T>, 12, 1> eigenValues = AMat.eigenvalues();
	//for (int i = 0; i < 12; i++) {
	//	if (!IsZero(eigenValues(i).real() - 1))
	//		std::cout << eigenValues(i).real() << "\n";
	//	//std::cout << eigenValues(i).real() << "\n";
	//}
	//std::cout << "\n";
	xVec = AMat.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(bVec);
	//std::cout << xVec << "\n\n";
}