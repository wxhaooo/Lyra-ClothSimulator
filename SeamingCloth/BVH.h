#pragma once
#include"BBox.h"
#include"BBoxTriangle.h"
#include"Shader.h"
#include"colorMapper.h"

#include<GraphicHelper/Camera.h>

#include<memory>
#include<vector>
#include<stack>
#include<utility>

namespace Lyra
{
	using namespace GraphicHelper;
	template<typename T> class BVH;
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
		uint32 level;
	};

	template<typename T>
	struct BVHFlatNode {
		BBox<T> bBox;
		uint32 start, nPrims, rightOffset;
		uint32 level;
	};

	template<typename T>
	struct BVHCollisionPair
	{
		BVHFlatNode<T> p0;
		BVHFlatNode<T> p1;
		//在vector<BVHFlatNode>中的index，用于遍历BVH树
		uint32 p0Index, p1Index;
	};

	/*template<typename T>
	using BVHCollisionPair = std::pair<BVHFlatNode<T>, BVHFlatNode<T>>;*/

	//抽象类只提供接口，不要有数据成员
	template<typename T>
	class BVH
	{
	public:
		BVH() = default;
		virtual void DebugGlDraw(Camera<T>& camera) = 0;
		virtual void DebugGlBind() = 0;

		virtual void GlDraw(Camera<T>& camera) = 0;
		virtual void GlBind() = 0;

	private:
		virtual void DebugCreate(shader_sp<T> shader, bool draw) = 0;

		virtual void Create(uint32 leafSize, shader_sp<T> shader, bool draw) = 0;
	};
}