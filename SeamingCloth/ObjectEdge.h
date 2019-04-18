#pragma once
#include<GraphicHelper/Vertex.h>

namespace Lyra
{
	using namespace GraphicHelper;
	template<typename T> struct ObjectEdge;
}

namespace Lyra
{
	template<typename T>
	struct ObjectEdge
	{
	public:
		ObjectEdge() = default;
		ObjectEdge(VertexPointer<T>& pp0, VertexPointer<T>& pp1) :p0(pp0), p1(pp1) {}
		VertexPointer<T> p0, p1;
	};
}