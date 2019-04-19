#pragma once
#include"BVH.h"
#include<GraphicHelper/Model.h>

namespace Lyra
{
	template<typename T> class ObjectBVH;
}

namespace Lyra
{
	template<typename T>
	class ObjectBVH :public BVH<T>
	{
	public:
		ObjectBVH() = default;

		/*Interface to private data memeber*/
		std::vector<BVHFlatNode<T>>& FlatBVHTree() { return flatBvhTree; }
		std::vector<BBoxObjTriangle<T>>& Fragments() { return fragments; }
		uint32 NLevel() { return nLevel; }

		void Build(ModelPointer<T>& model, uint32 leafSize, shader_sp<T> shader = nullptr, bool draw = false);
		void DebugGlDraw(Camera<T>& camera) override;
		void DebugGlBind() override;

		void GlDraw(Camera<T>& camera) override;
		void GlBind() override;

	private:
		void Init(ModelPointer<T>& model);
		void DebugCreate(shader_sp<T> shader, bool draw) override;

		void Create(uint32 leafSize, shader_sp<T> shader, bool draw) override;

	private:
		std::vector<BBoxObjTriangle<T>> fragments;
		std::vector<BBox<T>> bBoxes;
		std::vector<BVHFlatNode<T>> flatBvhTree;
		uint32 nNodes, nLeafs, leafSize, nLevel;
	};

	template<typename T>
	using objectBvh_sp = std::shared_ptr<ObjectBVH<T>>;
	template<typename T>
	using objectBvh_up = std::unique_ptr<ObjectBVH<T>>;
	template<typename T>
	using objectBvh_pt = ObjectBVH<T>*;
}

template<typename T>
void Lyra::ObjectBVH<T>::DebugGlBind()
{
	for (auto& bbox : bBoxes) {
		bbox.GlBind();
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::DebugGlDraw(Camera<T>& camera)
{
	for (auto& bbox : bBoxes) {
		bbox.GlDraw(camera);
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::GlDraw(Camera<T>& camera)
{
	for (auto& flatNode : flatBvhTree) {
		//if(flatNode.level == 8)
		flatNode.bBox.GlDraw(camera);
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::GlBind()
{
	glm::vec<3, T> color;
	ColorMapper<T> mapper;
	uint32 totalLevel = ceil(std::log2(flatBvhTree.size()));

	for (auto& flatNode : flatBvhTree) {
		T tmp = (flatNode.level + 1) / T(totalLevel + 1);
		mapper.GetGlmColor(tmp, Lyra::ColorMap::COLOR__MAGMA, color);
		flatNode.bBox.GlInit(color);
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::Build(ModelPointer<T> & model, uint32 leafSize, shader_sp<T> shader, bool draw)
{
	//std::cout << "Object BVH Build Start.....\n";
	//提取OBJ的triangle信息
	Init(model);
	Create(leafSize, shader, draw);
	//DebugCreate(shader, draw);
	//std::cout << "Object BVH Build End!!!\n";
}

template<typename T>
void Lyra::ObjectBVH<T>::Create(uint32 leafSize, shader_sp<T> shader, bool draw)
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

	nLevel = ceil(std::log2(flatBvhTree.size()));
}

template<typename T>
void Lyra::ObjectBVH<T>::DebugCreate(shader_sp<T> shader, bool draw)
{
	for (auto& frag : fragments) {
		bBoxes.push_back(frag.GetBBox(shader, draw));
	}
}

template<typename T>
void Lyra::ObjectBVH<T>::Init(ModelPointer<T> & model)
{
	auto& mesh = model->Meshes()[0];
	auto& triangles = mesh.Triangles();

	for (auto& tri : triangles) {
		fragments.push_back(BBoxObjTriangle<T>(tri.V1(), tri.V2(), tri.V3()));
	}
}