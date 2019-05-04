#pragma once

#include"LyraFunction.h"
#include"Particle.h"

#include"BBox.h"

#include<Eigen\Dense>

namespace Lyra
{
	template<typename T> class TrianglePatch;
	enum VelocityUpdate { PSEUDO_VELOCITY,MIDDLE_VELOCITY,CURRENT_VELOCITY,PRE_VELOCITY};
}

namespace Lyra
{
	template<typename T>
	class TrianglePatch
	{
	public:
		TrianglePatch() = default;
		~TrianglePatch() = default;

		TrianglePatch(particle_pt<T> xx1, particle_pt<T> xx2, particle_pt<T> xx3,
					  uint32 ind0, uint32 ind1, uint32 ind2, T stc, T shc, T dstc, T dshc, T stU, T stV,T alpha);

	public:
		/*Interface to private Functions*/
		uint32 index0() { return ind0; }
		uint32 index1() { return ind1; }
		uint32 index2() { return ind2; }

		particle_pt<T> X0() { return x0; }
		particle_pt<T> X1() { return x1; }
		particle_pt<T> X2() { return x2; }

		/*Common Functions*/
		vec3<T> Normal(bool inverse=false)
		{
			vec3<T> x21 = x1->position - x0->position;
			vec3<T> x31 = x2->position - x0->position;

			if (!inverse)
				return (x21.cross(x31)).normalized();

			return (x31.cross(x21)).normalized();
		}

		T Area() 
		{
			vec3<T> x21 = x1->position - x0->position;
			vec3<T> x31 = x2->position - x0->position;

			T areaTmp = 0.5 * x21.cross(x31).norm();

			return areaTmp;
		}

		/*Simulation Functions*/
		void ExplicitStretchForce();
		void ExplicitShearForce();

		void ImplicitDampingPlaneForce(T delta_t, bool enableDampingStretch, bool enableDampingShear, VelocityUpdate updateCat);

		void SemiImplicitStretchForce();
		void SemiImplicitShearForce();

		void ExplicitDampingStretchForce();
		void ExplicitDampingShearForce();

	private:
		void CollectMassInvMat(mat<T, 9, 9>& massMat);
		void CollectAMat(mat<T, 9, 9>& stretchMat, mat<T, 9, 9>& shearMat,T delta_t);
		void CollectDampingStretchMat(mat<T, 9, 9>& stretchMat, mat<T, 9, 9>& massInvMat, VelocityUpdate velUpdateCat);
		void CollectDampingShearMat(mat<T, 9, 9>& shearMat, mat<T, 9, 9>& massInvMat, VelocityUpdate velUpdateCat);
		void CollectbVec(vec<T, 9>& bVec, VelocityUpdate updateCat);

		void SolveLinearSystem(vec<T, 9>& xVec, mat<T, 9, 9>& AMat, vec<T, 9>& bVec);

	private:
		T area;
		T alpha;

		T stretchCoefficent, shearCoefficent;				//strecth和shear的系数
		T dampingStretchCoefficent, dampingShearCoefficent;	//damping的系数

		T stretchFactorInUDir, stretchFactorInVDir;			//U,V方向上的变化量

		T wux0, wux1, wux2;
		T wvx0, wvx1, wvx2;

		particle_pt<T> x0, x1, x2;

		uint32 ind0, ind1, ind2;

		mat2<T> theInverse;

		mat<T, 9, 9> massInvMat;
		mat<T, 9, 9> AMat;
		vec<T, 9> bVec;
		vec<T, 9> xVec;
	};
}

template<typename T>
Lyra::TrianglePatch<T>::TrianglePatch(particle_pt<T> xx1, particle_pt<T> xx2, particle_pt<T> xx3,
									  uint32 ind0, uint32 ind1, uint32 ind2,
									  T stc, T shc, T dstc, T dshc, T stU, T stV,T alpha)
{
	x0 = xx1; x1 = xx2; x2 = xx3;

	this->ind0 = ind0; this->ind1 = ind1; this->ind2 = ind2;

	stretchCoefficent = stc;
	shearCoefficent = shc;

	dampingStretchCoefficent = dstc;
	dampingShearCoefficent = dshc;

	stretchFactorInUDir = stU;
	stretchFactorInVDir = stV;

	this->alpha = alpha;

	T deltaU1 = x1->planeCoordinate[0] - x0->planeCoordinate[0];
	T deltaU2 = x2->planeCoordinate[0] - x0->planeCoordinate[0];
	T deltaV1 = x1->planeCoordinate[1] - x0->planeCoordinate[1];
	T deltaV2 = x2->planeCoordinate[1] - x0->planeCoordinate[1];

	mat2<T> matTmp;
	
	matTmp << deltaU1, deltaU2, deltaV1, deltaV2;

	//std::cout << matTmp << "\n";

	/*mat2<T> Sigma = mat2<T>::Zero();
	mat2<T> SigmaT;

	auto U = matTmp.bdcSvd(Eigen::ComputeFullU).matrixU();
	auto V = matTmp.bdcSvd(Eigen::ComputeFullV).matrixV();
	auto SingularValues = matTmp.bdcSvd().singularValues();

	T sv1 = SingularValues(0);
	T sv2 = SingularValues(1);

	if (IsZero(sv1))
		Sigma(0, 0) = 0.0;
	else
		Sigma(0, 0) = 1.0 / sv1;

	if (IsZero(sv2))
		Sigma(1, 1) = 0.0;
	else
		Sigma(1, 1) = 1.0 / sv2;

	SigmaT = Sigma.transpose();

	theInverse = V * SigmaT * U.transpose();*/

	theInverse = matTmp.inverse();
	
	//std::cout << theInverse << "\n\n";

	T u1v2u2v1 = deltaU1 * deltaV2 - deltaU2 * deltaV1;
	area = std::abs(0.5 * (u1v2u2v1));
	area = std::pow(area, alpha);

	//printf_s("%f\n", alpha);

	if (!IsZero(area)) {
		//wu的偏导数
		wux0 = (deltaV1 - deltaV2) / (u1v2u2v1);
		wux1 = (deltaV2) / (u1v2u2v1);
		wux2 = (-deltaV1) / (u1v2u2v1);
		//wv的偏导数
		wvx0 = (deltaU2 - deltaU1) / (u1v2u2v1);
		wvx1 = (-deltaU2) / (u1v2u2v1);
		wvx2 = (deltaU1) / (u1v2u2v1);
	} else {
		wux0 = 0.;
		wux1 = 0.;
		wux2 = 0.;

		wvx0 = 0.;
		wvx1 = 0.;
		wvx2 = 0.;
	}
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitStretchForce()
{
	/*vec3<T> deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
	vec3<T> deltaX2 = x2->pseudoPosition - x0->pseudoPosition;*/

	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();

	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;

	//std::cout << deltaX12 << "\n";
	//std::cout << theInverse << "\n";
	//std::cout << wUV << "\n";
	//std::cout << theInverse << "\n\n";

	//normalized wU,wV
	vec3<T> wU = wUV.col(0).normalized();
	vec3<T> wV = wUV.col(1).normalized();

	T wUl = wUV.col(0).norm();
	T wVl = wUV.col(1).norm();
	
	vec2<T> Cst = area * vec2<T>(wUl - stretchFactorInUDir, wVl - stretchFactorInVDir);

	//std::cout << Cst << "\n\n";

	//std::cout << stretchCoefficent << '\n';

	//stretch force for p0;
	mat<T, 3, 2> pcpx0;
	pcpx0.col(0) = area * wux0 * wU;
	pcpx0.col(1) = area * wvx0 * wV;

	vec3<T> stretchp0 = - stretchCoefficent * pcpx0 * Cst;
	x0->ApplyForce(stretchp0);

	//std::cout << "x0's force:" << stretchp0.norm() << "\n";

	//stretch force for p1;
	mat<T, 3, 2> pcpx1;
	pcpx1.col(0) = area * wux1 * wU;
	pcpx1.col(1) = area * wvx1 * wV;

	vec3<T> stretchp1 = -stretchCoefficent * pcpx1 * Cst;
	x1->ApplyForce(stretchp1);

	//std::cout << "x1's force:" << stretchp1.norm() << "\n";

	//stretch force for p2;
	mat<T, 3, 2> pcpx2;
	pcpx2.col(0) = area * wux2 * wU;
	pcpx2.col(1) = area * wvx2 * wV;

	vec3<T> stretchp2 = -stretchCoefficent * pcpx2 * Cst;
	x2->ApplyForce(stretchp2);

	//std::cout << "x2's force:" << stretchp2.norm() << "\n";
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitShearForce()
{
	/*vec3<T> deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
	vec3<T> deltaX2 = x2->pseudoPosition - x0->pseudoPosition;*/
	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();

	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;

	//wU,wV,注意这里不是归一化后的
	vec3<T> wU = wUV.col(0);
	vec3<T> wV = wUV.col(1);

	T wUl = wUV.col(0).norm();
	T wVl = wUV.col(1).norm();

	T Csh = area * wU.dot(wV);
	T shearCoefficentTmp = - shearCoefficent * area * Csh;

	//std::cout << area << " " << Csh << " " << "\n";
	//std::cout << shearCoefficentTmp << "\n";

	//shear force p0
	vec3<T> shearp0 = vec3<T>(wux0 * wV(0) + wvx0 * wU(0), wux0 * wV(1) + wvx0 * wU(1), wux0 * wV(2) + wvx0 * wU(2));
	//std::cout << shearp0.norm() << "\n";
	shearp0 *= shearCoefficentTmp;
	x0->ApplyForce(shearp0);
	//std::cout << shearp0 << "\n";
	//shear force p1
	vec3<T> shearp1 = vec3<T>(wux1 * wV(0) + wvx1 * wU(0), wux1 * wV(1) + wvx1 * wU(1), wux1 * wV(2) + wvx1 * wU(2));
	shearp1 *= shearCoefficentTmp;
	x1->ApplyForce(shearp1);
	//shear force p2
	vec3<T> shearp2 = vec3<T>(wux2 * wV(0) + wvx2 * wU(0), wux2 * wV(1) + wvx2 * wU(1), wux2 * wV(2) + wvx2 * wU(2));
	shearp2 *= shearCoefficentTmp;
	x2->ApplyForce(shearp2);
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitDampingStretchForce()
{
	vec3<T> deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
	vec3<T> deltaX2 = x2->pseudoPosition - x0->pseudoPosition;

	//vec3<T> deltaX1 = x1->position - x0->position;
	//vec3<T> deltaX2 = x2->position - x0->position;

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();

	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;

	//normalized wU,wV
	vec3<T> wU = wUV.col(0).normalized();
	vec3<T> wV = wUV.col(1).normalized();

	T wUl = wUV.col(0).norm();
	T wVl = wUV.col(1).norm(); 

	vec2<T> Cst = area * vec2<T>(wUl - stretchFactorInUDir, wVl - stretchFactorInVDir);

	T dampingCoefficent = -stretchCoefficent * dampingStretchCoefficent;
	//damping p0
	mat<T, 3, 2> pcpx0;
	pcpx0.col(0) = area * wux0 * wU;
	pcpx0.col(1) = area * wvx0 * wV;
	mat<T, 2, 1> Cdot0 = pcpx0.transpose() * /*x0->velocity*/x0->middleVelocity;
	vec3<T> dampingp0 = dampingCoefficent * pcpx0 * Cdot0;
	//std::cout << dampingp0 << "\n";
	x0->ApplyForce(dampingp0);

	//damping p1
	mat<T, 3, 2> pcpx1;
	pcpx1.col(0) = area * wux1 * wU;
	pcpx1.col(1) = area * wvx1 * wV;
	mat<T, 2, 1> Cdot1 = pcpx1.transpose() * /*x1->velocity*/x1->middleVelocity;
	vec3<T> dampingp1 = dampingCoefficent * pcpx1 * Cdot1;
	x1->ApplyForce(dampingp1);

	//damping p2
	mat<T, 3, 2> pcpx2;
	pcpx2.col(0) = area * wux2 * wU;
	pcpx2.col(1) = area * wvx2 * wV;
	mat<T, 2, 1> Cdot2 = pcpx2.transpose() * /*x2->velocity*/x2->middleVelocity;
	vec3<T> dampingp2 = dampingCoefficent * pcpx2 * Cdot2;
	x2->ApplyForce(dampingp2);

	/*std::cout << "p0: " << dampingp0 << "\n\n";
	std::cout << "p1: " <<dampingp1 << "\n\n";
	std::cout << "p2: " << dampingp2 << "\n\n";*/
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitDampingShearForce()
{
	vec3<T> deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
	vec3<T> deltaX2 = x2->pseudoPosition - x0->pseudoPosition;
	/*vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;*/

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();

	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//std::cout << deltaX12 << "\n";

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;

	//wU,wV,注意这里不是归一化后的
	vec3<T> wU = wUV.col(0);
	vec3<T> wV = wUV.col(1);

	T wUl = wUV.col(0).norm();
	T wVl = wUV.col(1).norm();

	//std::cout << wUV << "\n\n";

	T dampingCoefficent = -shearCoefficent * dampingShearCoefficent;

	//damping p0
	vec3<T> pcpx0 = area * vec3<T>(wux0 * wV(0) + wvx0 * wU(0), wux0 * wV(1) + wvx0 * wU(1), wux0 * wV(2) + wvx0 * wU(2));
	//std::cout << pcpx0.norm() << "\n";
	//std::cout << x0->middleVelocity << "\n";
	T cdot0 = pcpx0.dot(/*x0->velocity*/x0->middleVelocity);
	//std::cout << cdot0 << "\n";
	vec3<T> dampingp0 = dampingCoefficent * pcpx0 * cdot0;
	//std::cout << dampingp0.norm() << "\n";
	x0->ApplyForce(dampingp0);
	//std::cout << x0->acceleration.norm() << "\n";
	//damping p1
	vec3<T> pcpx1 = area * vec3<T>(wux1 * wV(0) + wvx1 * wU(0), wux1 * wV(1) + wvx1 * wU(1), wux1 * wV(2) + wvx1 * wU(2));
	T cdot1 = pcpx1.dot(/*x1->velocity*/x1->middleVelocity);
	vec3<T> dampingp1 = dampingCoefficent * pcpx1 * cdot1;
	x1->ApplyForce(dampingp1);
	//damping p2
	vec3<T> pcpx2 = area * vec3<T>(wux2 * wV(0) + wvx2 * wU(0), wux2 * wV(1) + wvx2 * wU(1), wux2 * wV(2) + wvx2 * wU(2));
	T cdot2 = pcpx2.dot(/*x2->velocity*/x2->middleVelocity);
	vec3<T> dampingp2 = dampingCoefficent * pcpx2 * cdot2;
	x2->ApplyForce(dampingp2);
}

template<typename T>
void Lyra::TrianglePatch<T>::CollectMassInvMat(mat<T, 9, 9>& massInvMat)
{
	T x0Mass = x0->mass;
	T x1Mass = x1->mass;
	T x2Mass = x2->mass;

	//std::cout << x0Mass << " " << x1Mass << " " << x2Mass << "\n";

	massInvMat(0, 0) = massInvMat(1, 1) = massInvMat(2, 2) = T(1) / x0Mass;
	massInvMat(3, 3) = massInvMat(4, 4) = massInvMat(5, 5) = T(1) / x1Mass;
	massInvMat(6, 6) = massInvMat(7, 7) = massInvMat(8, 8) = T(1) / x2Mass;
}

template<typename T>
void Lyra::TrianglePatch<T>::CollectAMat(mat<T, 9, 9>& stretchMat, mat<T, 9, 9>& shearMat,T delta_t)
{
	AMat = mat<T, 9, 9>::Identity() - 0.5 * delta_t * massInvMat * (stretchMat + shearMat);
}

template<typename T>
void Lyra::TrianglePatch<T>::CollectDampingStretchMat(mat<T, 9, 9>& stretchMat, mat<T, 9, 9>& massInvMat, VelocityUpdate velUpdateCat)
{
	stretchMat.setZero(); 

	vec3<T> deltaX1, deltaX2;

	if (velUpdateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		deltaX1 = x1->position - x0->position;
		deltaX2 = x2->position - x0->position;
	}
	else if (velUpdateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
		deltaX2 = x2->pseudoPosition - x0->pseudoPosition;
	}

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();
	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;
	//normalized wU,wV
	vec3<T> wUn = wUV.col(0).normalized();
	vec3<T> wVn = wUV.col(1).normalized();

	//Damping Stretch Force Matrix Construction
	mat<T, 3, 2> pcpx0_sch, pcpx1_sch, pcpx2_sch;
	pcpx0_sch.col(0) = area * wux0 * wUn;
	pcpx0_sch.col(1) = area * wvx0 * wVn;

	pcpx1_sch.col(0) = area * wux1 * wUn;
	pcpx1_sch.col(1) = area * wvx1 * wVn;

	pcpx2_sch.col(0) = area * wux2 * wUn;
	pcpx2_sch.col(1) = area * wvx2 * wVn;

	mat<T, 9, 2> pcpxStretch = mat<T, 9, 2>::Zero();
	mat<T, 2, 9> pcpxStretchT = mat<T, 2, 9>::Zero();

	pcpxStretch.block<3, 2>(0, 0) = pcpx0_sch;
	pcpxStretch.block<3, 2>(3, 0) = pcpx1_sch;
	pcpxStretch.block<3, 2>(6, 0) = pcpx2_sch;
	pcpxStretchT = pcpxStretch.transpose();

	T stretchDampingFactor = - stretchCoefficent * dampingStretchCoefficent;
	stretchMat = stretchDampingFactor * pcpxStretch * pcpxStretchT;
}

template<typename T>
void Lyra::TrianglePatch<T>::CollectDampingShearMat(mat<T, 9, 9>& shearMat, mat<T, 9, 9>& massInvMat, VelocityUpdate velUpdateCat)
{
	shearMat.setZero();
	vec3<T> deltaX1, deltaX2;

	if (velUpdateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		deltaX1 = x1->position - x0->position;
		deltaX2 = x2->position - x0->position;
	}
	else if (velUpdateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		deltaX1 = x1->pseudoPosition - x0->pseudoPosition;
		deltaX2 = x2->pseudoPosition - x0->pseudoPosition;
	}

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();
	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;
	vec3<T> wU = wUV.col(0);
	vec3<T> wV = wUV.col(1);

	//Damping Shear Force Matrix Construction 
	mat<T, 9, 1> pcpxShear = mat<T, 9, 1>::Zero();
	mat<T, 1, 9> pcpxShearT = mat<T, 1, 9>::Zero();

	vec3<T> pcpx0_sh, pcpx1_sh, pcpx2_sh;
	pcpx0_sh = vec3<T>(wux0 * wV(0) + wvx0 * wU(0), wux0 * wV(1) + wvx0 * wU(1), wux0 * wV(2) + wvx0 * wU(2));
	pcpx1_sh = vec3<T>(wux1 * wV(0) + wvx1 * wU(0), wux1 * wV(1) + wvx1 * wU(1), wux1 * wV(2) + wvx1 * wU(2));
	pcpx2_sh = vec3<T>(wux2 * wV(0) + wvx2 * wU(0), wux2 * wV(1) + wvx2 * wU(1), wux2 * wV(2) + wvx2 * wU(2));

	pcpxShear.block<3, 1>(0, 0) = pcpx0_sh;
	pcpxShear.block<3, 1>(3, 0) = pcpx1_sh;
	pcpxShear.block<3, 1>(6, 0) = pcpx2_sh;
	pcpxShearT = pcpxShear.transpose();

	T shearDampingFactor = -shearCoefficent * dampingShearCoefficent;
	shearMat = shearDampingFactor * pcpxShear * pcpxShearT;
}

template<typename T>
void Lyra::TrianglePatch<T>::CollectbVec(vec<T, 9>& bVec,VelocityUpdate updateCat)
{
	vec3<T> x0Vel, x1Vel, x2Vel;

	if (updateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		x0Vel = x0->velocity;
		x1Vel = x1->velocity;
		x2Vel = x2->velocity;
		/*x0Vel = x0->velocityTmp;
		x1Vel = x1->velocityTmp;
		x2Vel = x2->velocityTmp;*/
	}
	else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		x0Vel = x0->middleVelocity;
		x1Vel = x1->middleVelocity;
		x2Vel = x2->middleVelocity;
	}

	bVec.block<3, 1>(0, 0) = x0Vel;
	bVec.block<3, 1>(3, 0) = x1Vel;
	bVec.block<3, 1>(6, 0) = x2Vel;
}

template<typename T>
void Lyra::TrianglePatch<T>::SolveLinearSystem(vec<T, 9>& xVec, mat<T, 9, 9>& AMat, vec<T, 9>& bVec)
{
	Eigen::ColPivHouseholderQR<mat<T, 9, 9>> solver(AMat);
	xVec = solver.solve(bVec);

	//std::cout << xVec << "\n";
	//std::cout << AMat << "\n\n";
	//std::cout << bVec << "\n\n";
	
	//T relativeError;
	//if(!IsZero(bVec.norm()))
	//	relativeError = (AMat * xVec - bVec).norm() / bVec.norm();
	//else
	//	relativeError = (AMat * xVec - bVec).norm();

	///*std::cout << AMat << "\n";
	//std::cout << bVec << "\n";
	//std::cout << xVec << "\n";*/
	//std::cout << "relative error is: " << relativeError << "\n";
}

template<typename T>
void Lyra::TrianglePatch<T>::ImplicitDampingPlaneForce(T delta_t, bool enableDampingStretch,bool enableDampingShear, VelocityUpdate updateCat)
{
	AMat.setZero();
	massInvMat.setZero();
	bVec.setZero();

	//std::cout << massInvMat << "\n\n";

	mat<T, 9, 9> stretchMat, shearMat;
	CollectMassInvMat(massInvMat);

	//std::cout << massInvMat << "\n";

	if (enableDampingStretch)
		CollectDampingStretchMat(stretchMat, massInvMat, updateCat);

	if (enableDampingShear)
		CollectDampingShearMat(shearMat, massInvMat, updateCat);

	CollectAMat(stretchMat, shearMat, delta_t);
	CollectbVec(bVec,updateCat);
	SolveLinearSystem(xVec, AMat, bVec);

	if (updateCat == VelocityUpdate::MIDDLE_VELOCITY) {
		x0->middleVelocity += (xVec.block<3, 1>(0, 0) - x0->velocity);
		x1->middleVelocity += (xVec.block<3, 1>(3, 0) - x1->velocity);
		x2->middleVelocity += (xVec.block<3, 1>(6, 0) - x2->velocity);
		/*x0->velocityTmp = (xVec.block<3, 1>(0, 0));
		x1->velocityTmp = (xVec.block<3, 1>(3, 0));
		x2->velocityTmp = (xVec.block<3, 1>(6, 0));*/
	}
	else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY) {
		x0->pseudoVelocity += (xVec.block<3, 1>(0, 0) - x0->middleVelocity);
		x1->pseudoVelocity += (xVec.block<3, 1>(3, 0) - x1->middleVelocity);
		x2->pseudoVelocity += (xVec.block<3, 1>(6, 0) - x2->middleVelocity);
		/*x0->middleVelocity = (xVec.block<3, 1>(0, 0));
		x1->middleVelocity = (xVec.block<3, 1>(3, 0));
		x2->middleVelocity = (xVec.block<3, 1>(6, 0));*/
	}
}

