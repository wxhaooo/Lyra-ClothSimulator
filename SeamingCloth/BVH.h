#pragma once
#include"BBox.h"
#include"BBoxTriangle.h"
#include"Shader.h"

#include<GraphicHelper/Model.h>
#include<GraphicHelper/Camera.h>

#include<memory>
#include<vector>

namespace Lyra
{
	using namespace GraphicHelper;
	template<typename T> class BVH;
	template<typename T> class ObjectBVH;
	template<typename T> class ClothBVH;
}

namespace Lyra
{
	template<typename T>
	class BVH
	{
	public:
		BVH() = default;
	};

	template<typename T>
	class ObjectBVH :public BVH<T>
	{
	public:
		ObjectBVH() = default;

		void Build(ModelPointer<T>& model, Shader<T>& shader, bool draw = false);
		void GlDraw(Camera<T>& camera);
		void GlBind();

	private:
		void Init(ModelPointer<T>& model);
		void DebugCreate(Shader<T>& shader, bool draw);

	private:
		std::vector<BBoxObjTriangle<T>> fragments;
		std::vector<BBox<T>> bBoxes;
	};

	template<typename T>
	class ClothBVH :public BVH<T>
	{
	public:
		ClothBVH() = default;
	};

	template<typename T>
	using objectBvh_sp = std::shared_ptr<ObjectBVH<T>>;
	template<typename T>
	using objectBvh_up = std::unique_ptr<ObjectBVH<T>>;
	template<typename T>
	using objectBvh_pt = ObjectBVH<T>*;
}

template<typename T>
void Lyra::ObjectBVH<T>::GlBind()
{
	for (auto& bbox : bBoxes) {
		bbox.GlBind();
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::GlDraw(Camera<T>& camera)
{
	for (auto& bbox : bBoxes) {
		bbox.GlDraw(camera);
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::Build(ModelPointer<T>& model, Shader<T>& shader, bool draw)
{
	std::cout << "BVH Build Start.....\n";
	Init(model);
	DebugCreate(shader, draw);
	std::cout << "BVH Build End!!!\n";
}

template<typename T>
void Lyra::ObjectBVH<T>::DebugCreate(Shader<T>& shader, bool draw)
{
	for (auto& frag : fragments) {
		bBoxes.push_back(frag.GetBBox(shader, draw));
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::Init(ModelPointer<T>& model)
{
	auto& mesh = model->Meshes()[0];
	auto& triangles = mesh.Triangles();

	for (auto& tri : triangles) {
		fragments.push_back(BBoxObjTriangle<T>(tri.V1(), tri.V2(), tri.V3()));
	}
}