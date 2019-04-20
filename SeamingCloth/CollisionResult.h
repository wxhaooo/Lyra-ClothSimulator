#pragma once

#include"ClothEdge.h"
#include"ObjectEdge.h"
#include"BBoxTriangle.h"

#include<vector>

namespace Lyra
{
	template<typename T> struct Edge2Edge_C2O;
	template<typename T> struct Edge2Edge_C2C;
	template<typename T> struct Vertex2Triangle_C2O;
	template<typename T> struct Vertex2Triangle_C2C;
}

namespace Lyra
{
	template<typename T>
	struct Edge2Edge_C2O
	{
		ClothEdge<T> clothEdge0;
		ObjectEdge<T> objectEdge0;
	};

	template<typename T>
	struct Vertex2Triangle_C2O
	{
		//cloth vertex
		particle_pt<T> v0;
		//object triangle vertex
		VertexPointer<T> t0;
		VertexPointer<T> t1;
		VertexPointer<T> t2;
	};

	template<typename T>
	struct Vertex2Triangle_C2O_
	{
		//object vertex
		VertexPointer<T> v0;
		//cloth triangle
		particle_pt<T> t0;
		particle_pt<T> t1;
		particle_pt<T> t2;
	};

	template<typename T>
	struct Edge2Edge_C2C
	{
		ClothEdge<T> clothEdge0;
		ClothEdge<T> clothEdge1;
	};

	template<typename T>
	struct Vertex2Triangle_C2C
	{
		//cloth vertex
		particle_pt<T> v0;
		//cloth triangle vertex
		particle_pt<T> t0;
		particle_pt<T> t1;
		particle_pt<T> t2;
	};
}

namespace Lyra
{
	//用于cloth和object间做collision detection
	template<typename T>
	struct CollisionResults_C2O
	{
		std::vector<Vertex2Triangle_C2O_<T>> vertex2Triangle_;
		std::vector<Vertex2Triangle_C2O<T>> vertex2Triangle;
		std::vector<Edge2Edge_C2O<T>> edge2Edge;
	};
	//用于cloth和cloth间做collision detection
	template<typename T>
	struct CollisionResults_C2C
	{
		std::vector<Vertex2Triangle_C2C<T>> vertex2Triangle;
		std::vector<Edge2Edge_C2C<T>> edge2Edge;
	};
}