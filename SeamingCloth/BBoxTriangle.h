#pragma once

#include"Shader.h"
#include"BBox.h"
#include"Particle.h"

#include<GraphicHelper/Vertex.h>

namespace Lyra
{
	template<typename T> class BBoxTriangle;
	//用于object建立BBox的类
	template<typename T> class BBoxObjTriangle;
	//用于cloth建立BBox的类
	template<typename T> class BBoxClothTriangle;

	using namespace GraphicHelper;
}

namespace Lyra
{
	template<typename T>
	class BBoxTriangle
	{
	public:
		BBoxTriangle() = default;

		virtual BBox<T> GetBBox(Shader<T>& shader, bool draw = false) = 0;
		virtual vec3<T> GetCentroid() = 0;
		//virtual bool IsIntersectWithBBox(BBox<T>& bBox) = 0;
	};
}

namespace Lyra
{
	template<typename T>
	class BBoxObjTriangle :public BBoxTriangle<T>
	{
	public:
		BBoxObjTriangle() = default;
		BBoxObjTriangle(VertexPointer<T>& vv1, VertexPointer<T>& vv2, VertexPointer<T>& vv3);

		BBox<T> GetBBox(Shader<T>& shader, bool draw = false) override;
		vec3<T> GetCentroid() override;

		VertexPointer<T> V0() { return v0; }
		VertexPointer<T> V1() { return v1; }
		VertexPointer<T> V2() { return v2; }

	private:
		VertexPointer<T> v0, v1, v2;
	};

	template<typename T>
	class BBoxClothTriangle :public BBoxTriangle<T>
	{
	public:
		BBoxClothTriangle() = default;
		BBoxClothTriangle(particle_pt<T> p0, particle_pt<T> p1, particle_pt<T> p2);

		BBox<T> GetBBox(Shader<T>& shader, bool draw = false) override;
		vec3<T> GetCentroid() override;

		bool IsIntersectWithBBox(BBox<T>& bBox);

		particle_pt<T> P0() { return p0; }
		particle_pt<T> P1() { return p1; }
		particle_pt<T> P2() { return p2; }

	private:
		particle_pt<T> p0, p1, p2;
	};
}

////////////////////////////////////////BBoxObjTriangle////////////////////////////////////////////////

template<typename T>
Lyra::BBoxObjTriangle<T>::BBoxObjTriangle(VertexPointer<T>& vv0, VertexPointer<T>& vv1, VertexPointer<T>& vv2)
{
	v0 = vv0; v1 = vv1; v2 = vv2;
}

template<typename T>
Lyra::BBox<T> Lyra::BBoxObjTriangle<T>::GetBBox(Shader<T>& shader, bool draw)
{
	//有个留下来的小问题，读入的时候是glm，这里用的都是eigen
	vec3<T> vv0(v0->position.x, v0->position.y, v0->position.z);
	vec3<T> vv1(v1->position.x, v1->position.y, v1->position.z);
	vec3<T> vv2(v2->position.x, v2->position.y, v2->position.z);

	vec3<T> minCoordTmp = Min(vv0, vv1, vv2);
	vec3<T> maxCoordTmp = Max(vv0, vv1, vv2);

	const vec3<T> minCornerCoord(minCoordTmp(0), minCoordTmp(1), minCoordTmp(2));
	const vec3<T> maxCornerCoord(maxCoordTmp(0), maxCoordTmp(1), maxCoordTmp(2));

	return Lyra::BBox<T>(minCornerCoord, maxCornerCoord, shader, draw);
}

template<typename T>
Lyra::vec3<T> Lyra::BBoxObjTriangle<T>::GetCentroid()
{
	glm::vec<3, T> centroid = T(1) / 3 * (v0->position + v1->position + v2->position);

	return vec3<T>(centroid.x, centroid.y, centroid.z);
}

////////////////////////////////////////BBoxClothTriangle////////////////////////////////////////////////

template<typename T>
Lyra::BBoxClothTriangle<T>::BBoxClothTriangle(particle_pt<T> pp0, particle_pt<T> pp1, particle_pt<T> pp2)
{
	p0 = pp0; p1 = pp1; p2 = pp2;
}

template<typename T>
Lyra::BBox<T> Lyra::BBoxClothTriangle<T>::GetBBox(Shader<T>& shader, bool draw)
{
	vec3<T> minCornerCoordTmp, maxCornerCoordTmp;
	minCornerCoordTmp = Min(p0->position, p1->position, p2->position);
	maxCornerCoordTmp = Max(p0->position, p1->position, p2->position);

	const vec3<T> minCornerCoord(minCornerCoordTmp(0), minCornerCoordTmp(1), minCornerCoordTmp(2));
	const vec3<T> maxCornerCoord(maxCornerCoordTmp(0), maxCornerCoordTmp(1), maxCornerCoordTmp(2));

	return Lyra::BBox<T>(minCornerCoord, maxCornerCoord, shader, draw);
}

template<typename T>
Lyra::vec3<T> Lyra::BBoxClothTriangle<T>::GetCentroid()
{
	vec3<T> centroid = T(1) / 3 * (p0->position + p1->position + p2->position);

	return centroid;
}

template<typename T>
bool Lyra::BBoxClothTriangle<T>::IsIntersectWithBBox(BBox<T>& bBox)
{
	//triangle-AABB intersection test
}