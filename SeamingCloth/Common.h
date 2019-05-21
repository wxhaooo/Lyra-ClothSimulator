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
	/*template<typename Scalar>
	using Complex = std::complex<Scalar>;*/

	using uint32 = unsigned int;
	using uint32_sp = std::shared_ptr<uint32>;
	using uint32_up = std::unique_ptr<uint32>;
	using uint32_pt = uint32 *;
	/*template<typename Scalar>
	using vec2 = glm::vec<2, Scalar, glm::defaultp>;

	template<typename Scalar>
	using vec3 = glm::vec<3, Scalar, glm::defaultp>;

	template<typename Scalar>
	using vec4 = glm::vec<4, Scalar, glm::defaultp>;*/

	template<typename Scalar>
	using vec2 = Eigen::Matrix<Scalar, 2, 1>;

	template<typename Scalar>
	using vec3 = Eigen::Matrix<Scalar, 3, 1>;

	template<typename Scalar>
	using vec4 = Eigen::Matrix<Scalar, 4, 1>;

	template<typename Scalar, int N>
	using vec = Eigen::Matrix<Scalar, N, 1>;

	template<typename Scalar>
	using vecX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

	template<typename Scalar, int M, int N>
	using mat = Eigen::Matrix<Scalar, M, N>;

	template<typename Scalar>
	using matX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

	template<typename Scalar>
	using mat2 = Eigen::Matrix<Scalar, 2, 2>;

	template<typename Scalar>
	using mat3 = Eigen::Matrix<Scalar, 3, 3>;

	template<typename Scalar>
	using mat4 = Eigen::Matrix<Scalar, 4, 4>;

	template<typename Scalar,int N>
	using matnxn = Eigen::Matrix<Scalar, N, N>;

	template<typename Scalar>
	using vector_sp = std::shared_ptr<std::vector<Scalar>>;

	template<typename Scalar>
	using vector_up = std::unique_ptr<std::vector<Scalar>>;

	template<typename Scalar>
	using vector_pt = std::vector<Scalar> *;

	template<typename Scalar>
	using matSp = Eigen::SparseMatrix<Scalar>;
	
}

//常量
namespace Lyra
{
	constexpr float PI = 3.141592653f;
}

