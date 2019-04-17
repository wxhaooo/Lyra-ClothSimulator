#pragma once
#include"BVH.h"
#include"ClothPatch.h"

namespace Lyra
{
	template<typename T> class ClothBVH;
}

namespace Lyra
{
	template<typename T>
	class ClothBVH :public BVH<T>
	{
	public:
		ClothBVH() = default;

		void Build(clothPatch_sp<T>& patch, uint32 leafSize, Shader<T>& shader, bool draw = false);
		void DebugGlDraw(Camera<T>& camera);
		void DebugGlBind();

		void GlDraw(Camera<T>& camera);
		void GlBind();

	private:
		void Init(clothPatch_sp<T>& patch);
		void DebugCreate(Shader<T>& shader, bool draw);

		void Create(uint32 leafSize, Shader<T>& shader, bool draw);

	private:
		std::vector<BBoxClothTriangle<T>> fragments;
		std::vector<BBox<T>> bBoxes;
		std::vector<BVHFlatNode<T>> flatBvhTree;
		uint32 nNodes, nLeafs, leafSize;
	};

	template<typename T>
	using clothBvh_sp = std::shared_ptr<ClothBVH<T>>;
	template<typename T>
	using clothBvh_up = std::unique_ptr<ClothBVH<T>>;
	template<typename T>
	using clothBvh_pt = ClothBVH<T>*;
}

template<typename T>
void Lyra::ClothBVH<T>::Build(clothPatch_sp<T>& patch, uint32 leafSize, Shader<T>& shader, bool draw)
{
	std::cout << "Cloth BVH Build Start.....\n";
	//提取OBJ的triangle信息
	Init(patch);
	Create(leafSize, shader, draw);
	//DebugCreate(shader, draw);
	std::cout << "Cloth BVH Build End!!!\n";
}

template<typename T>
void Lyra::ClothBVH<T>::DebugGlDraw(Camera<T>& camera)
{

}

template<typename T>
void Lyra::ClothBVH<T>::DebugGlBind()
{

}

template<typename T>
void Lyra::ClothBVH<T>::GlDraw(Camera<T>& camera)
{
	for (auto& flatNode : flatBvhTree) {
		flatNode.bBox.GlDraw(camera);
	}
}

template<typename T>
void Lyra::ClothBVH<T>::GlBind()
{
	glm::vec<3, T> color;
	ColorMapper<T> mapper;
	uint32 totalLevel = ceil(std::log2(flatBvhTree.size()));

	std::cout << totalLevel << "\n\n";
	for (auto& flatNode : flatBvhTree) {
		T tmp = (flatNode.level + 1) / T(totalLevel + 1);
		mapper.GetGlmColor(tmp, Lyra::ColorMap::COLOR__MAGMA, color);
		flatNode.bBox.GlInit(color);
	}
}

template<typename T>
void Lyra::ClothBVH<T>::Init(clothPatch_sp<T>& patch)
{
	auto& trianglePatches = patch->TrianglePatches();

	for (auto& tri : trianglePatches) {
		fragments.push_back(BBoxClothTriangle<T>(tri.X0(), tri.X1(), tri.X2()));
	}

	//std::cout << fragments.size() << "\n\n";
}

template<typename T>
void Lyra::ClothBVH<T>::DebugCreate(Shader<T>& shader, bool draw)
{

}

template<typename T>
void Lyra::ClothBVH<T>::Create(uint32 leafSize, Shader<T>& shader, bool draw)
{
	//top-down to build BVH
	std::stack<BVHBuildEntry> todo;
	BVHBuildEntry root, curNode;
	BVHFlatNode<T> node;

	root.start = 0;
	root.end = fragments.size();
	root.parent = BvhMagicNumber::ROOT_PARENT;
	root.level = 0;

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
		node.level = bNode.level;

		BBox<T> bb(fragments[startTmp].GetBBox(shader, false));
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
		curNode.level = flatBvhTree[curNode.parent].level + 1;
		//std::cout << curNode.level << "\n\n";
		todo.push(curNode);

		curNode.start = startTmp;
		curNode.end = mid;
		curNode.parent = nNodes - 1;
		curNode.level = flatBvhTree[curNode.parent].level + 1;

		//system("pause");
		todo.push(curNode);
	}
}