#pragma once

#include"LyraFunction.h"
#include"Particle.h"

#include"BBox.h"

namespace Lyra
{
	template<typename T> class TrianglePatch;
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
		void ExplicitDampingStretchForce();
		void ExplicitShearForce();
		void ExplicitDampingShearForce();

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
	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

	mat<T, 3, 2> deltaX12 = mat<T, 3, 2>::Zero();

	deltaX12.col(0) = deltaX1;
	deltaX12.col(1) = deltaX2;

	//WUV matrix
	mat<T, 3, 2> wUV = deltaX12 * theInverse;
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
void Lyra::TrianglePatch<T>::ExplicitDampingStretchForce()
{
	//std::cout << "damping Stretch Force\n";
	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

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
	mat<T, 2, 1> Cdot0 = pcpx0.transpose() * x0->velocity;
	vec3<T> dampingp0 = dampingCoefficent * pcpx0 * Cdot0;
	x0->ApplyForce(dampingp0);

	//damping p1
	mat<T, 3, 2> pcpx1;
	pcpx1.col(0) = area * wux1 * wU;
	pcpx1.col(1) = area * wvx1 * wV;
	mat<T, 2, 1> Cdot1 = pcpx1.transpose() * x1->velocity;
	vec3<T> dampingp1 = dampingCoefficent * pcpx1 * Cdot1;
	x1->ApplyForce(dampingp1);

	//damping p2
	mat<T, 3, 2> pcpx2;
	pcpx2.col(0) = area * wux2 * wU;
	pcpx2.col(1) = area * wvx2 * wV;
	mat<T, 2, 1> Cdot2 = pcpx2.transpose() * x2->velocity;
	vec3<T> dampingp2 = dampingCoefficent * pcpx2 * Cdot2;
	x2->ApplyForce(dampingp2);

	/*std::cout << "p0: " << dampingp0 << "\n\n";
	std::cout << "p1: " <<dampingp1 << "\n\n";
	std::cout << "p2: " << dampingp2 << "\n\n";*/
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitShearForce()
{
	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

	////这是条很神奇的语句，先留着
	//if (deltaX1.norm() != deltaX1.norm()) {
	//	std::cout << deltaX1 << "\n";
	//}

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
	T shearCoefficent = - shearCoefficent * area * Csh;

	//shear force p0
	vec3<T> shearp0 = vec3<T>(wux0 * wV(0) + wvx0 * wU(0), wux0 * wV(1) + wvx0 * wV(1), wux0 * wV(2) + wvx0 * wV(2));
	shearp0 *= shearCoefficent;
	x0->ApplyForce(shearp0);
	//shear force p1
	vec3<T> shearp1 = vec3<T>(wux1 * wV(0) + wvx1 * wU(0), wux1 * wV(1) + wvx1 * wV(1), wux1 * wV(2) + wvx1 * wV(2));
	shearp1 *= shearCoefficent;
	x1->ApplyForce(shearp1);
	//shear force p2
	vec3<T> shearp2 = vec3<T>(wux2 * wV(0) + wvx2 * wU(0), wux2 * wV(1) + wvx2 * wV(1), wux2 * wV(2) + wvx2 * wV(2));
	shearp2 *= shearCoefficent;
	x2->ApplyForce(shearp2);
}

template<typename T>
void Lyra::TrianglePatch<T>::ExplicitDampingShearForce()
{
	vec3<T> deltaX1 = x1->position - x0->position;
	vec3<T> deltaX2 = x2->position - x0->position;

	//这是条很神奇的语句，先留着
	/*if (deltaX1.norm() != deltaX1.norm()) {
		std::cout << deltaX1 << "\n";
	}*/

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

	T dampingCoefficent = -shearCoefficent * dampingShearCoefficent;

	//damping p0
	vec3<T> pcpx0 = vec3<T>(wux0 * wV(0) + wvx0 * wU(0), wux0 * wV(1) + wvx0 * wV(1), wux0 * wV(2) + wvx0 * wV(2));
	T cdot0 = pcpx0.dot(x0->velocity);
	vec3<T> dampingp0 = dampingCoefficent * pcpx0 * cdot0;
	x0->ApplyForce(dampingp0);
	//damping p1
	vec3<T> pcpx1 = vec3<T>(wux1 * wV(0) + wvx1 * wU(0), wux1 * wV(1) + wvx1 * wV(1), wux1 * wV(2) + wvx1 * wV(2));
	T cdot1 = pcpx1.dot(x1->velocity);
	vec3<T> dampingp1 = dampingCoefficent * pcpx1 * cdot1;
	x1->ApplyForce(dampingp1);
	//damping p2
	vec3<T> pcpx2 = vec3<T>(wux2 * wV(0) + wvx2 * wU(0), wux2 * wV(1) + wvx2 * wV(1), wux2 * wV(2) + wvx2 * wV(2));
	T cdot2 = pcpx2.dot(x2->velocity);
	vec3<T> dampingp2 = dampingCoefficent * pcpx2 * cdot2;
	x2->ApplyForce(dampingp2);
}
