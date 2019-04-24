#pragma once

#include"Common.h"
#include"Shader.h"
#include"ForceSwitch.h"

#include<string>
#include<vector>

namespace Lyra
{
	template<typename T> struct ClothPatchParms;
	enum ClothPatchMode;
	enum ClothPatchInitState;
}

enum Lyra::ClothPatchInitState
{
	LYRA_CLOTH_PATCH_PLANE,
	LYRA_CLOTH_PATCH_NON_PLANE
};

enum Lyra::ClothPatchMode
{
	LYRA_CLOTH_PATCH_SINGLE,
	LYRA_CLOTH_PATCH_SEAMING
};

namespace Lyra
{
	template<typename T>
	struct ClothPatchParms
	{
		//file path
		std::string path;
		//file name
		std::string name;
		//patch density
		T density;
		//stretch force factor
		T stretchingFactor;
		//shearing force factor
		T shearingFactor;
		//bending force factor
		T bendingFactor;
		//damping factor for stretch force
		T dampingStretchFactor;
		//damping factor for Shear force
		T dampingShearFactor;
		//damping factor for Bend force
		T dampingBendingFactor;
		//stretch scale in U
		T stretchScaleUDir;
		//stretch scale in V
		T stretchScaleVDir;

		T lengthRangeMin;

		T lengthRangeMax;
		//stick points
		vector_sp<uint32> stickPoints;
		//patch的模式可以不设置，没有影响
		ClothPatchMode patchMode;

		ClothPatchInitState initState;

		Shader<T> shader;

		PlaneForceSwitch planeForceSwitch;
		SpaceForceSwitch spaceForceSwitch;

		//仅用于非平面状态用来放大初始的UV
		T scale;
		//用于调节面积的影响
		T alpha;

		bool enableCollisionDetect;

		T frictionFactorForObject;
		T dampingFactorForObject;

		ClothPatchParms() { alpha = 3. / 4;  }

		ClothPatchParms<T> &operator = (ClothPatchParms<T> &parms)
		{
			path = parms.path;
			name = parms.name;

			density = parms.density;

			stretchingFactor = parms.stretchingFactor;
			shearingFactor = parms.shearingFactor;
			bendingFactor = parms.bendingFactor;
			dampingStretchFactor = parms.dampingStretchFactor;
			dampingShearFactor = parms.dampingShearFactor;
			dampingBendingFactor = parms.dampingBendingFactor;

			stretchScaleUDir = parms.stretchScaleUDir;
			stretchScaleVDir = parms.stretchScaleVDir;

			lengthRangeMin = parms.lengthRangeMin;
			lengthRangeMax = parms.lengthRangeMax;

			stickPoints = parms.stickPoints;

			patchMode = parms.patchMode;
			initState = parms.initState;

			scale = parms.scale;
			alpha = parms.alpha;

			shader = parms.shader;

			planeForceSwitch = parms.planeForceSwitch;
			spaceForceSwitch = parms.spaceForceSwitch;

			enableCollisionDetect = parms.enableCollisionDetect;

			frictionFactorForObject = parms.frictionFactorForObject;
			dampingFactorForObject = parms.dampingFactorForObject;
			return *this;
		}
	};
}
