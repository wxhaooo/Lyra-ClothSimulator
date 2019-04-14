#pragma once

#include<utility>
#include<vector>
#include"ClothPatch.h"


namespace Lyra
{
	using SeamingPair = std::pair<uint32, uint32>;
	template<typename T> struct SeamingInfo;
	template<typename T> struct SeamingParms;
}

namespace Lyra
{
	template<typename T>
	struct SeamingInfo
	{
		clothPatch_sp<T> patch1, patch2;
		std::vector<SeamingPair> seamingPairs;
	};

	template<typename T>
	struct SeamingParms
	{
		T force;
		T relax;
		T damp;
		T atten;
		T attet;

		SeamingParms()
		{
			force = 20.;
			relax = 5.;
			damp = 25.;
			atten = 1.;
			attet = 0.;
		}

		SeamingParms &operator=(SeamingParms &parms)
		{
			force = parms.force;
			relax = parms.relax;
			damp = parms.damp;
			atten = parms.atten;
			attet = parms.attet;

			return *this;
		}

		void DebugInfo() 
		{
			std::cout << "force: " << force << "\n";
			std::cout << "relax: " <<relax << "\n";
			std::cout << "damp: " << damp << "\n";
			std::cout << "atten: " << atten << "\n";
			std::cout << "attet: " << attet << "\n\n";
			system("pause");
		}
	};
}