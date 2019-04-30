#pragma once

#include<vector>
#include<memory>
#include<typeinfo>
#include<algorithm>
#include<Eigen\Dense>
#include<Eigen\Sparse>

#include<glm\gtc\matrix_transform.hpp>

//类型
namespace Lyra
{
	/*template<typename T>
	using vec2 = glm::vec<2, T, glm::defaultp>;

	template<typename T>
	using vec3 = glm::vec<3, T, glm::defaultp>;

	template<typename T>
	using vec4 = glm::vec<4, T, glm::defaultp>;*/

	template<typename T>
	using vec2 = Eigen::Matrix<T, 2, 1>;

	template<typename T>
	using vec3 = Eigen::Matrix<T, 3, 1>;

	template<typename T>
	using vec4 = Eigen::Matrix<T, 4, 1>;

	template<typename T, int N>
	using vec = Eigen::Matrix<T, N, 1>;

	template<typename T, int M, int N>
	using mat = Eigen::Matrix<T, M, N>;

	template<typename T>
	using mat2 = Eigen::Matrix<T, 2, 2>;

	template<typename T>
	using mat3 = Eigen::Matrix<T, 3, 3>;

	template<typename T>
	using mat4 = Eigen::Matrix<T, 4, 4>;

	template<typename T,int N>
	using matnxn = Eigen::Matrix<T, N, N>;

	template<typename T>
	using vector_sp = std::shared_ptr<std::vector<T>>;

	template<typename T>
	using vector_up = std::unique_ptr<std::vector<T>>;

	template<typename T>
	using vector_pt = std::vector<T> *;


	using uint32 = unsigned int;
	using uint32_sp = std::shared_ptr<uint32>;
	using uint32_up = std::unique_ptr<uint32>;
	using uint32_pt = uint32 *;
}

//常量
namespace Lyra
{
	constexpr float PI = 3.141592653f;
}

