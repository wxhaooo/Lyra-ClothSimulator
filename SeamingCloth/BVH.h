#pragma once
#include"BBox.h"
#include"BBoxTriangle.h"
#include"Shader.h"
#include"colorMapper.h"

#include<GraphicHelper/Camera.h>

#include<memory>
#include<vector>
#include<stack>

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
	class BVH
	{
	public:
		BVH() = default;
	};
}