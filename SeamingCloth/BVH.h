#pragma once
#include"BBox.h"
#include"BBoxTriangle.h"
#include"Shader.h"

#include<GraphicHelper/Model.h>
#include<GraphicHelper/Camera.h>

#include<memory>
#include<vector>
#include<stack>

namespace Lyra
{
	using namespace GraphicHelper;
	template<typename T> class BVH;
	template<typename T> class ObjectBVH;
	template<typename T> class ClothBVH;

	template<typename T> struct BVHFlatNode;

	struct BVHBuildEntry;

	//magic number for build BVH
	/*constexpr uint32 ROOT_PARENT = 0xfffffffc;
	constexpr uint32 UNTOUCHED = 0xffffffff;
	constexpr uint32 TOUCHEDTWICE = 0xfffffffd;*/

	enum BvhMagicNumber :uint32 {
		ROOT_PARENT = 0xfffffffc,
		UNTOUCHED = 0xffffffff,
		TOUCHEDTWICE = 0xfffffffd,
		LEAF = 0x00000000
	};
}

namespace Lyra
{
	struct BVHBuildEntry
	{
		uint32 parent;
		uint32 start, end;
	};

	template<typename T>
	struct BVHFlatNode {
		BBox<T> bBox;
		uint32 start, nPrims, rightOffset;
	};
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

		void Build(ModelPointer<T>& model, uint32 leafSize, Shader<T>& shader, bool draw = false);
		void GlDraw(Camera<T>& camera);
		void GlBind();

	private:
		void Init(ModelPointer<T>& model);
		void DebugCreate(Shader<T>& shader, bool draw);

		void Create(uint32 leafSize, Shader<T>& shader, bool draw);

	private:
		std::vector<BBoxObjTriangle<T>> fragments;
		std::vector<BBox<T>> bBoxes;
		std::vector<BVHFlatNode<T>> flatBvhTree;
		uint32 nNodes, nLeafs, leafSize;
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
void Lyra::ObjectBVH<T>::Build(ModelPointer<T>& model, uint32 leafSize, Shader<T>& shader, bool draw)
{
	std::cout << "BVH Build Start.....\n";
	//提取OBJ的triangle信息
	Init(model);
	Create(leafSize, shader, draw);
	//DebugCreate(shader, draw);
	std::cout << "BVH Build End!!!\n";
}

template<typename T>
void Lyra::ObjectBVH<T>::Create(uint32 leafSize,Shader<T>& shader, bool draw)
{
	//top-down to build BVH
	std::stack<BVHBuildEntry> todo;
	BVHBuildEntry root, curNode;
	BVHFlatNode<T> node;

	root.start = 0;
	root.end = fragments.size();
	root.parent = BvhMagicNumber::ROOT_PARENT;

	todo.push(root);

	//resize会初始化分配的内存，reserve不会
	flatBvhTree.reserve(fragments.size() * 2);

	while (!todo.empty()) {
		/////////////////////////To Build This Level's BBox//////////////////////////////////////
		BVHBuildEntry& bNode(todo.top());
		todo.pop();
		uint32_t startTmp = bNode.start;
		uint32_t endTmp = bNode.end;
		uint32_t nPrims = endTmp - startTmp;

		nNodes++;
		node.start = startTmp;
		node.nPrims = nPrims;
		node.rightOffset = BvhMagicNumber::UNTOUCHED;

		BBox<T> bb(fragments[startTmp].GetBBox(shader, draw));
		//bc用于估计整个OBJ的质心位置,不需要渲染出来
		BBox<T> bc(fragments[startTmp].GetCentroid(), shader, false);

		for (uint32 p = startTmp + 1; p < endTmp; p++) {
			bb.ExpandToInclude(fragments[p].GetBBox(shader, false));
			bc.ExpandToInclude(fragments[p].GetCentroid());
		}

		node.bBox = bb;

		if (nPrims <= leafSize) {
			node.rightOffset = BvhMagicNumber::LEAF;
			nLeafs++;
		}

		flatBvhTree.push_back(node);

		if (bNode.parent != BvhMagicNumber::ROOT_PARENT) {
			flatBvhTree[bNode.parent].rightOffset--;
			if (flatBvhTree[bNode.parent].rightOffset == BvhMagicNumber::TOUCHEDTWICE) {
				flatBvhTree[bNode.parent].rightOffset = nNodes - 1 - bNode.parent;
			}
		}

		if (node.rightOffset == BvhMagicNumber::LEAF)
			continue;

		///////////////////////////partition//////////////////////////////////////////////
		uint32 splitDim = bc.MaxDimension();
		T splitCoord = T(1) / 2 * (bc.minCorner(splitDim) + bc.maxCorner(splitDim));

		uint32 mid = startTmp;
		for (uint32 i = startTmp; i < endTmp; i++) {
			if (fragments[i].GetCentroid()[splitDim] < splitCoord)
				std::swap(fragments[i], fragments[mid]);
			++mid;
		}

		if (mid == startTmp || mid == endTmp) {
			mid = startTmp + (endTmp - startTmp) / T(2);
		}

		curNode.start = mid;
		curNode.end = endTmp;
		curNode.parent = nNodes - 1;
		todo.push(curNode);

		curNode.start = startTmp;
		curNode.end = mid;
		curNode.parent = nNodes - 1;
		todo.push(curNode);
	}
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