#pragma once
#include<vector>
#include<fstream>
#include<iostream>
#include<memory>
#include<typeinfo>
#include<algorithm>
#include<Eigen\Dense>

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

template<typename T, int N>
using matnxn = Eigen::Matrix<T, N, N>;

template<typename T>
using vector_sp = std::shared_ptr<std::vector<T>>;

template<typename T>
using vector_up = std::unique_ptr<std::vector<T>>;

template<typename T>
using vector_pt = std::vector<T>*;

using uint32 = unsigned int;

constexpr float PI = 3.141592653f;

template<typename T>
class PlyExporter
{
private:
	std::fstream ofs;

public:
	PlyExporter() = default;
	~PlyExporter() = default;

	bool Save(std::vector<vec3<T>>& vertices, std::vector<vec2<T>>& colors,
		std::vector<vec3<T>>& normals, std::vector<vec3<uint32>>& indices,
		std::string& fileName, std::string& filePath);

public:
	bool Create(std::string& fileName, std::string& filePath);

	void DumpVertices(std::vector<vec3<T>>& vertices);
	void DumpColors(std::vector<vec2<T>>& colors);
	void DumpNormals(std::vector<vec3<T>>& normals);
	void DumpFaces(std::vector<vec3<uint32>>& indices);
};

template<typename T>
bool PlyExporter<T>::Save(std::vector<vec3<T>>& vertices, std::vector<vec2<T>>& colors,
	std::vector<vec3<T>>& normals, std::vector<vec3<uint32>>& indices,
	std::string& fileName, std::string& filePath)
{

}