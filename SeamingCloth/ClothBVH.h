#pragma once
#include"BVH.h"
#include"TrianglePatch.h"
#include"ObjectBVH.h"
#include"CollisionResult.h"

#include<ExactCCD/rootparitycollisiontest.h>

#include<stack>

namespace Lyra
{
	template<typename T> class ClothBVH;
	using namespace rootparity;
	enum class ClothBvhCategory { CLOTH_BVH_PRE, CLOTH_BVH_POST };
}

namespace Lyra
{
	template<typename T>
	class ClothBVH
	{
	public:
		ClothBVH() = default;

		/*Interface to private data memeber*/
		std::vector<BVHFlatNode<T>>& FlatBVHTree() { return flatBvhTree; }
		std::vector<BBoxClothTriangle<T>>& Fragments() { return fragments; }

		uint32 NLevels() { return nLevels; }

		/*Interface to Build Cloth BVH*/
		void Build(std::vector<TrianglePatch<T>>& triangles, uint32 leafSize,
			ClothBvhCategory cat= ClothBvhCategory::CLOTH_BVH_PRE,
			shader_sp<T> shader=nullptr, bool draw = false);
		
		void ReBuild(std::vector<TrianglePatch<T>>& triangles, uint32 leafSize,
			ClothBvhCategory cat = ClothBvhCategory::CLOTH_BVH_PRE,
			shader_sp<T> shader = nullptr, bool draw = false);

		/*Interface to Debug*/
		void DebugGlDraw(Camera<T>& camera);
		void DebugGlBind();

		/*Interface to Draw BVH*/
		void GlDraw(Camera<T>& camera);
		void GlBind();

		/*Interface to Collision*/
		//cloth-object BVH collision
		void CollisionWithObjBVH(ObjectBVH<T>& objectBVH, CollisionResults_C2O<T>& collisionResult);
		//cloth-cloth BVH collision,用于多件cloth的互相碰撞
		void CollisionWithClothBVH(ClothBVH<T>& clothBVH, CollisionResults_C2C<T>& collisionResult);

	private:
		/*private function member to build BVH*/
		void Init(std::vector<TrianglePatch<T>>& triangles);
		void Create(uint32 leafSize, shader_sp<T> shader, bool draw,
			ClothBvhCategory cat = ClothBvhCategory::CLOTH_BVH_PRE);
		void DebugCreate(shader_sp<T> shader, bool draw);

		/*private function member to detect BVH collision*/
		void BVHCollisionDetect(std::vector<BVHFlatNode<T>>& bvh0, std::vector<BVHFlatNode<T>>& bvh1,
			std::vector<BBoxClothTriangle<T>>& fragment0, std::vector<BBoxObjTriangle<T>>& fragment1,
			CollisionResults_C2O<T>& collsionResult);

		void CollidePrimitives(BVHCollisionPair<T>& collisionPair, 
			std::vector<BBoxClothTriangle<T>>& fragment0, std::vector<BBoxObjTriangle<T>>& fragment1,
			CollisionResults_C2O<T>& collsionResult);

		bool DescendStrategy(BVHCollisionPair<T>& collisionPair);

		void Edge2EdgeCollisionDetect(BBoxClothTriangle<T>& clothTriangle, BBoxObjTriangle<T>& objTriangle,
			CollisionResults_C2O<T>& collsionResult);

		void Point2TriangleCollisionDetect(BBoxClothTriangle<T>& clothTriangle, BBoxObjTriangle<T>& objTriangle,
			CollisionResults_C2O<T>& collsionResult);

		void Edge2EdgeCollisionDetect(ClothEdge<T>& clothEdge, ObjectEdge<T>& objectEdge, 
			CollisionResults_C2O<T>& collsionResult);

		void Point2TriangleCollisionDetect(particle_pt<T> p, BBoxObjTriangle<T>& objTriangle,
			CollisionResults_C2O<T>& collsionResult);

		void Point2TriangleCollisionDetect(VertexPointer<T>& v, BBoxClothTriangle<T>& clothTriangle,
			CollisionResults_C2O<T>& collsionResult);

		bool ExactEdge2EdgeCCD(ClothEdge<T>& clothEdge, ObjectEdge<T>& objectEdge);

		bool ExactPoint2TriangleCCD(particle_pt<T> p, BBoxObjTriangle<T>& objTriangle);

		bool ExactPoint2TriangleCCD(VertexPointer<T> v, BBoxClothTriangle<T>& clothTriangle);

		void DebugTrversal();

	private:
		std::vector<BBoxClothTriangle<T>> fragments;
		std::vector<BBox<T>> bBoxes;
		std::vector<BVHFlatNode<T>> flatBvhTree;
		uint32 nNodes, nLeafs, leafSize, nLevels;
	};

	template<typename T>
	using clothBvh_sp = std::shared_ptr<ClothBVH<T>>;
	template<typename T>
	using clothBvh_up = std::unique_ptr<ClothBVH<T>>;
	template<typename T>
	using clothBvh_pt = ClothBVH<T>*;
}


template<typename T>
void Lyra::ClothBVH<T>::ReBuild(std::vector<TrianglePatch<T>>& triangles, uint32 leafSize, ClothBvhCategory cat,shader_sp<T> shader, bool draw)
{
	fragments.clear();
	bBoxes.clear();
	flatBvhTree.clear();
	nNodes = 0;
	nLeafs = 0;
	nLevels = 0;
	this->leafSize = leafSize;

	Build(triangles, leafSize, cat, shader, draw);
}

template<typename T>
void Lyra::ClothBVH<T>::Build(std::vector<TrianglePatch<T>>& triangles, uint32 leafSize, ClothBvhCategory cat, shader_sp<T> shader, bool draw)
{
	//std::cout << "Cloth BVH Build Start.....\n";
	//提取Cloth的triangle信息
	Init(triangles);
	Create(leafSize, shader, draw, cat);

	//DebugTrversal();

	/*for (uint32 i = 0; i < flatBvhTree.size(); i++) {
		if (flatBvhTree[i].rightOffset == 0) {
			std::cout << flatBvhTree[i].start << "\n";
		}
	}
	system("pause");*/
	//DebugCreate(shader, draw);
	//std::cout << "Cloth BVH Build End!!!\n";
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
void Lyra::ClothBVH<T>::Init(std::vector<TrianglePatch<T>>& triangles)
{
	for (auto& tri : triangles) {
		fragments.push_back(BBoxClothTriangle<T>(tri.X0(), tri.X1(), tri.X2()));
	}

	//std::cout << fragments.size() << "\n\n";
}

template<typename T>
void Lyra::ClothBVH<T>::DebugCreate(shader_sp<T> shader, bool draw)
{

}

template<typename T>
void Lyra::ClothBVH<T>::Create(uint32 leafSize, shader_sp<T> shader, bool draw, ClothBvhCategory cat)
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
		
		BBox<T> bb, bc;

		if (cat == ClothBvhCategory::CLOTH_BVH_PRE)
		{
			bb = fragments[startTmp].GetBBox(shader, false);
			//bc用于估计整个OBJ的质心位置,不需要渲染出来
			bc = BBox<T>(fragments[startTmp].GetCentroid(), shader, false);

			for (uint32 p = startTmp + 1; p < endTmp; p++) {
				bb.ExpandToInclude(fragments[p].GetBBox(shader, false));
				bc.ExpandToInclude(fragments[p].GetCentroid());
			}
		}
		else if (cat == ClothBvhCategory::CLOTH_BVH_POST) {
			bb = fragments[startTmp].GetPostBBox(shader, false);
			bc = BBox<T>(fragments[startTmp].GetPostCentroid(), shader, false);

			for (uint32 p = startTmp + 1; p < endTmp; p++) {
				bb.ExpandToInclude(fragments[p].GetPostBBox(shader, false));
				bc.ExpandToInclude(fragments[p].GetPostCentroid());
			}	
		}

		node.bBox = bb;

		//计算BBox的平均速度，用三角形三个顶点插出来
		vec3<T> vTmp;
		vTmp.setZero();
		for (uint32 i = 0; i < node.nPrims; i++) {
			vTmp += fragments[node.start + i].GetAverageVelocty();
			node.bBox.velocity = T(1) / node.nPrims * vTmp;
		}

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

		///////////////////////////////partition//////////////////////////////////////////
		uint32 splitDim = bc.MaxDimension();
		T splitCoord = T(1) / 2 * (bc.minCorner(splitDim) + bc.maxCorner(splitDim));

		uint32 mid = startTmp;

		if (cat == ClothBvhCategory::CLOTH_BVH_PRE)
		{
			for (uint32 i = startTmp; i < endTmp; i++) {
				if (fragments[i].GetCentroid()[splitDim] < splitCoord)
					std::swap(fragments[i], fragments[mid]);
				++mid;
			}
		}
		else if (cat == ClothBvhCategory::CLOTH_BVH_POST) {
			for (uint32 i = startTmp; i < endTmp; i++) {
				if (fragments[i].GetPostCentroid()[splitDim] < splitCoord)
					std::swap(fragments[i], fragments[mid]);
				++mid;
			}
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

	nLevels = ceil(std::log2(flatBvhTree.size()));
}

template<typename T>
void Lyra::ClothBVH<T>::CollisionWithObjBVH(ObjectBVH<T>& objectBVH, CollisionResults_C2O<T>& collisionResult)
{
	auto& objectFlatBvhTree = objectBVH.FlatBVHTree();
	auto& objectFragments = objectBVH.Fragments();

	//printf_s("%d %d\n", objectFlatBvhTree.size(), objectFragments.size());
	//system("pause");

	//暴力检测
	for (auto& clothFrag : fragments) {
		for (auto& objectFrag : objectFragments) {
			Edge2EdgeCollisionDetect(clothFrag, objectFrag, collisionResult);
			Point2TriangleCollisionDetect(clothFrag, objectFrag, collisionResult);
		}
	}

	//BVHCollisionDetect(flatBvhTree, objectFlatBvhTree, fragments, objectFragments, collisionResult);
}

template<typename T>
void Lyra::ClothBVH<T>::BVHCollisionDetect(
	std::vector<BVHFlatNode<T>>& bvh0, std::vector<BVHFlatNode<T>>& bvh1,
	std::vector<BBoxClothTriangle<T>>& fragment0, std::vector<BBoxObjTriangle<T>>& fragment1, 
	CollisionResults_C2O<T>& collsionResult)
{
	//两棵BVH Tree一起遍历
	auto& root0 = bvh0[0];
	auto& root1 = bvh1[0];

	/*printf_s("%d %d\n", bvh0.size(), bvh1.size());
	system("pause");*/

	std::stack<BVHCollisionPair<T>> s;

	BVHCollisionPair<T> root;
	root.p0Index = 0;
	root.p1Index = 0;
	root.p0 = root0;
	root.p1 = root1;

	s.push(root);

	while (!s.empty()) {
		auto& curPair = s.top();
		s.pop();

		//printf_s("%d\n", s.size());

		/*bool bBoxOverlap;
		if (curPair.p0.rightOffset == 0 || curPair.p1.rightOffset == 0)
			bBoxOverlap = curPair.p0.bBox.IsIntersectWithMovingBBox(curPair.p1.bBox);
		else
			bBoxOverlap = curPair.p0.bBox.IsIntersectWithBBox(curPair.p1.bBox);*/
		bool bBoxOverlap = curPair.p0.bBox.IsIntersectWithBBox(curPair.p1.bBox);
		//bool bBoxOverlap = curPair.p0.bBox.IsIntersectWithMovingBBox(curPair.p1.bBox);
		if (!bBoxOverlap) continue;

		//如果都是叶子节点,检查叶子节点中的triangle的相交情况
		if (curPair.p0.rightOffset == 0 && curPair.p1.rightOffset == 0) {
			CollidePrimitives(curPair, fragment0, fragment1, collsionResult);
		}
		else {
			if (DescendStrategy(curPair)) {

				uint32 posLeftChild = curPair.p0Index + 1;
				uint32 posRightChild = curPair.p0Index + curPair.p0.rightOffset;

				BVHCollisionPair<T> tmpPair;
				tmpPair.p1Index = curPair.p1Index;
				tmpPair.p1 = curPair.p1;

				tmpPair.p0Index = posRightChild;
				tmpPair.p0 = bvh0[posRightChild];
				s.push(tmpPair);

				tmpPair.p0Index = posLeftChild;
				tmpPair.p0 = bvh0[posLeftChild];
				s.push(tmpPair);
			}
			else {
				uint32 posLeftChild = curPair.p1Index + 1;
				uint32 posRightChild = curPair.p1Index + curPair.p1.rightOffset;

				//printf_s("233333\n");
				BVHCollisionPair<T> tmpPair;
				tmpPair.p0Index = curPair.p0Index;
				tmpPair.p0 = curPair.p0;

				tmpPair.p1Index = posRightChild;
				tmpPair.p1 = bvh1[posRightChild];
				s.push(tmpPair);

				tmpPair.p1Index = posLeftChild;
				tmpPair.p1 = bvh1[posLeftChild];

				s.push(tmpPair);
			}
		}
	}

}

template<typename T>
bool Lyra::ClothBVH<T>::DescendStrategy(BVHCollisionPair<T>& collisionPair)
{
	//优先遍历A树的策略
	if (collisionPair.p0.rightOffset == 0)
		return false;
	return true;
}

template<typename T>
void Lyra::ClothBVH<T>::CollidePrimitives(BVHCollisionPair<T>& collisionPair,
	std::vector<BBoxClothTriangle<T>>& fragment0, std::vector<BBoxObjTriangle<T>>& fragment1,
	CollisionResults_C2O<T>& collsionResult)
{
	//对triangle碰撞分成vertex-vertex、vertex-triangle
	//对来检查并返回collisionDetect的信息到collsionResult
	
	std::vector<BBoxClothTriangle<T>> clothTriangles;
	std::vector<BBoxObjTriangle<T>> objectTriangles;

	uint32 startPos0 = collisionPair.p0.start;
	uint32 startPos1 = collisionPair.p1.start;

	//提取可能碰撞的cloth三角形
	for (uint32 i = 0; i < collisionPair.p0.nPrims; i++) {
		clothTriangles.push_back(fragment0[startPos0 + i]);
	}
	//提取可能碰撞的object三角形
	for (uint32 i = 0; i < collisionPair.p1.nPrims; i++) {
		objectTriangles.push_back(fragment1[startPos1 + i]);
	}

	for (uint32 i = 0; i < clothTriangles.size(); i++) {
		for (uint32 j = 0; j < objectTriangles.size(); j++) {
			Edge2EdgeCollisionDetect(clothTriangles[i], objectTriangles[j], collsionResult);
			Point2TriangleCollisionDetect(clothTriangles[i], objectTriangles[j], collsionResult);
		}
	}
}

template<typename T>
void Lyra::ClothBVH<T>::Edge2EdgeCollisionDetect(BBoxClothTriangle<T>& clothTriangle, BBoxObjTriangle<T>& objTriangle,
	CollisionResults_C2O<T>& collsionResult)
{
	//edge-edge collision test
	particle_pt<T> p0 = clothTriangle.P0();
	particle_pt<T> p1 = clothTriangle.P1();
	particle_pt<T> p2 = clothTriangle.P2();

	VertexPointer<T> v0 = objTriangle.V0();
	VertexPointer<T> v1 = objTriangle.V1();
	VertexPointer<T> v2 = objTriangle.V2();

	std::vector<ClothEdge<T>> clothEdges;
	std::vector<ObjectEdge<T>> objectEdges;

	clothEdges.push_back(ClothEdge<T>(p0, p1));
	clothEdges.push_back(ClothEdge<T>(p1, p2));
	clothEdges.push_back(ClothEdge<T>(p2, p1));

	objectEdges.push_back(ObjectEdge<T>(v0, v1));
	objectEdges.push_back(ObjectEdge<T>(v1, v2));
	objectEdges.push_back(ObjectEdge<T>(v2, v1));

	for (uint32 i = 0; i < clothEdges.size(); i++) {
		for (uint32 j = 0; j < objectEdges.size(); j++) {
			Edge2EdgeCollisionDetect(clothEdges[i], objectEdges[j], collsionResult);
		}
	}
}

template<typename T>
bool Lyra::ClothBVH<T>::ExactPoint2TriangleCCD(VertexPointer<T> v, BBoxClothTriangle<T>& clothTriangle)
{
	particle_pt<T> t0 = clothTriangle.P0();
	particle_pt<T> t1 = clothTriangle.P1();
	particle_pt<T> t2 = clothTriangle.P2();

	//v是object上的点
	Vec3d pOld(v->position.x, v->position.y, v->position.z);
	Vec3d pNew(v->position.x, v->position.x, v->position.x);

	//triangle's vertex old position and new one
	Vec3d t0Old(t0->position(0), t0->position(1), t0->position(2));
	Vec3d t0New(t0->pseudoPosition(0), t0->pseudoPosition(1), t0->pseudoPosition(2));
	Vec3d t1Old(t1->position(0), t1->position(1), t1->position(2));
	Vec3d t1New(t1->pseudoPosition(0), t1->pseudoPosition(1), t1->pseudoPosition(2));
	Vec3d t2Old(t2->position(0), t2->position(1), t2->position(2));
	Vec3d t2New(t2->pseudoPosition(0), t2->pseudoPosition(1), t2->pseudoPosition(2));

	RootParityCollisionTest p2TriangleCollisionTest(pOld, t0Old, t1Old, t2Old, pNew, t0New, t1New, t2New, false);

	return p2TriangleCollisionTest.run_test();
}

template<typename T>
void Lyra::ClothBVH<T>::Point2TriangleCollisionDetect(VertexPointer<T>& v, BBoxClothTriangle<T>& clothTriangle,
	CollisionResults_C2O<T>& collsionResult)
{
	//single point-triangle pair test
	if (ExactPoint2TriangleCCD(v, clothTriangle)) {
		Vertex2Triangle_C2O_<T> vertex2Triangle;
		vertex2Triangle.v0 = v;

		vertex2Triangle.t0 = clothTriangle.P0();
		vertex2Triangle.t1 = clothTriangle.P1();
		vertex2Triangle.t2 = clothTriangle.P2();

		collsionResult.vertex2Triangle_.push_back(vertex2Triangle);
	}
}

template<typename T>
void Lyra::ClothBVH<T>::Point2TriangleCollisionDetect(BBoxClothTriangle<T>& clothTriangle, BBoxObjTriangle<T>& objTriangle,
	CollisionResults_C2O<T>& collsionResult)
{
	//point-triangle collision test
	particle_pt<T> p0 = clothTriangle.P0();
	particle_pt<T> p1 = clothTriangle.P1();
	particle_pt<T> p2 = clothTriangle.P2();

	VertexPointer<T> v0 = objTriangle.V0();
	VertexPointer<T> v1 = objTriangle.V1();
	VertexPointer<T> v2 = objTriangle.V2();

	//solve a tri-equation to decide whether point intesect with triangle
	Point2TriangleCollisionDetect(p0, objTriangle, collsionResult);
	Point2TriangleCollisionDetect(p1, objTriangle, collsionResult);
	Point2TriangleCollisionDetect(p2, objTriangle, collsionResult);

	Point2TriangleCollisionDetect(v0, clothTriangle, collsionResult);
	Point2TriangleCollisionDetect(v1, clothTriangle, collsionResult);
	Point2TriangleCollisionDetect(v2, clothTriangle, collsionResult);
}

template<typename T>
void Lyra::ClothBVH<T>::Edge2EdgeCollisionDetect(ClothEdge<T>& clothEdge, ObjectEdge<T>& objectEdge,
	CollisionResults_C2O<T>& collsionResult)
{
	if (ExactEdge2EdgeCCD(clothEdge, objectEdge)) {
		Edge2Edge_C2O<T> edge2Edge;
		edge2Edge.clothEdge0 = clothEdge;
		edge2Edge.objectEdge0 = objectEdge;
		collsionResult.edge2Edge.push_back(edge2Edge);
	}
}

template<typename T>
void Lyra::ClothBVH<T>::Point2TriangleCollisionDetect(particle_pt<T> p, BBoxObjTriangle<T>& objTriangle,
	CollisionResults_C2O<T>& collsionResult)
{
	//single point-triangle pair test
	if (ExactPoint2TriangleCCD(p, objTriangle)) {
		Vertex2Triangle_C2O<T> vertex2Triangle;
		vertex2Triangle.v0 = p;

		vertex2Triangle.t0 = objTriangle.V0();
		vertex2Triangle.t1 = objTriangle.V1();
		vertex2Triangle.t2 = objTriangle.V2();

		collsionResult.vertex2Triangle.push_back(vertex2Triangle);
	}

}

template<typename T>
bool Lyra::ClothBVH<T>::ExactEdge2EdgeCCD(ClothEdge<T>& clothEdge, ObjectEdge<T>& objectEdge)
{
	//single point-triangle test
	particle_pt<T> p0 = clothEdge.p0;
	particle_pt<T> p1 = clothEdge.p1;

	VertexPointer<T>& v0 = objectEdge.p0;
	VertexPointer<T>& v1 = objectEdge.p1;

	//cloth edge顶点的新旧位置
	Vec3d p0Old(p0->position(0), p0->position(1), p0->position(2));
	Vec3d p0New(p0->pseudoPosition(0), p0->pseudoPosition(1), p0->pseudoPosition(2));

	Vec3d p1Old(p1->position(0), p1->position(1), p1->position(2));
	Vec3d p1New(p1->pseudoPosition(0), p1->pseudoPosition(1), p1->pseudoPosition(2));

	/*std::cout << p0Old << " " << " " << p1Old << "\n\n";
	std::cout << p0New << " " << " " << p1New << "\n\n";*/

	//object edge顶点的新旧位置一致
	Vec3d v0Old(v0->position.x, v0->position.y, v0->position.z);
	Vec3d v0New(v0->position.x, v0->position.y, v0->position.z);

	Vec3d v1Old(v1->position.x, v1->position.y, v1->position.z);
	Vec3d v1New(v1->position.x, v1->position.y, v1->position.z);

	RootParityCollisionTest edge2EdgeColllsionTest(p0Old, p1Old, v0Old, v1Old, p0New, p1New, v0New, v1New, true);

	return edge2EdgeColllsionTest.run_test();

}

template<typename T>
bool Lyra::ClothBVH<T>::ExactPoint2TriangleCCD(particle_pt<T> p, BBoxObjTriangle<T>& objTriangle)
{
	VertexPointer<T> t0 = objTriangle.V0();
	VertexPointer<T> t1 = objTriangle.V1();
	VertexPointer<T> t2 = objTriangle.V2();

	//p是cloth上的点，triangle是object上的
	Vec3d pOld(p->position(0), p->position(1), p->position(2));
	Vec3d pNew(p->pseudoPosition(0), p->pseudoPosition(1), p->pseudoPosition(2));
	
	//triangle's vertex old position and new one
	Vec3d t0Old(t0->position.x, t0->position.y, t0->position.z);
	Vec3d t0New(t0->position.x, t0->position.y, t0->position.z);
	Vec3d t1Old(t1->position.x, t1->position.y, t1->position.z);
	Vec3d t1New(t1->position.x, t1->position.y, t1->position.z);
	Vec3d t2Old(t2->position.x, t2->position.y, t2->position.z);
	Vec3d t2New(t2->position.x, t2->position.y, t2->position.z);

	RootParityCollisionTest p2TriangleCollisionTest(pOld, t0Old, t1Old, t2Old, pNew, t0New, t1New, t2New, false);

	return p2TriangleCollisionTest.run_test();
}

template<typename T>
void Lyra::ClothBVH<T>::CollisionWithClothBVH(ClothBVH<T>& clothBVH, CollisionResults_C2C<T>& collisionResult)
{
	
}

template<typename T>
void Lyra::ClothBVH<T>::DebugTrversal()
{
	std::vector<bool> isVisited;
	isVisited.resize(fragments.size(), false);

	uint32 root = 0;
	std::stack<uint32> s;
	s.push(root);
	while (!s.empty()) {
		uint32 cur = s.top();
		s.pop();

		if (flatBvhTree[cur].rightOffset == 0) {
			for (uint32 i = 0; i < flatBvhTree[cur].nPrims; i++) {
				isVisited[flatBvhTree[cur].start + i] = true;
			}
		}
		else {
			s.push(cur + flatBvhTree[cur].rightOffset);
			s.push(cur + 1);
		}
	}

	for (uint32 i = 0; i < isVisited.size(); i++) {
		if (!isVisited[i]) {
			std::cout << "BVH of Cloth do not build correctly!!!\n";
			system("pause");
			exit(0);
		}
	}
}