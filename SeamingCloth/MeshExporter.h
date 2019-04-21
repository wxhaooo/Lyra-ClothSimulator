#pragma once
#include<vector>
#include<fstream>
#include<iostream>
#include"Common.h"

namespace Lyra
{
	template<typename T> class MeshExporter;
	enum MeshExporterSwitch;
}

enum Lyra::MeshExporterSwitch
{
	MESH_EXPORTER_ALL,
	MESH_EXPORTER_VERTICES,
	MESH_EXPORTER_VERTICES_NORMALS,
	MESH_EXPORTER_VERTICES_TEXTURECOORDS
};

namespace Lyra
{
	template<typename T>
	class MeshExporter
	{
	private:
		std::fstream ofs;
	public:
		MeshExporter() = default;
		~MeshExporter() = default;

		bool Save(std::vector<vec3<T>>& vertices, std::vector<vec2<T>>& texCoords,
			std::vector<vec3<T>>& normals, std::vector<vec3<uint32>>& indices,
			std::string& fileName, std::string& filePath, MeshExporterSwitch exporterSwitch = MeshExporterSwitch::MESH_EXPORTER_ALL);

	public:
		bool Create(std::string& fileName, std::string& filePath);

		void DumpVertices(std::vector<vec3<T>>& vertices);
		void DumpTexCoords(std::vector<vec2<T>>& texCoords);
		void DumpNormals(std::vector<vec3<T>>& normals);
		void DumpFaces(std::vector<vec3<uint32>>& indices, MeshExporterSwitch exporterSwitch = MeshExporterSwitch::MESH_EXPORTER_ALL);
	};
}

template<typename T>
bool Lyra::MeshExporter<T>::Save(std::vector<vec3<T>>& vertices, std::vector<vec2<T>>& texCoords,
	std::vector<vec3<T>>& normals, std::vector<vec3<uint32>>& indices,
	std::string& fileName, std::string& filePath, Lyra::MeshExporterSwitch exporterSwitch)
{
	bool isCreted = Create(fileName, filePath);
	if (!isCreted) return false;

	ofs << "# OBJ file:\"Cloth\"" << std::endl;
	ofs << "o Cloth" << std::endl;

	DumpVertices(vertices);

	if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_VERTICES_NORMALS ||
		exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_ALL)
		DumpNormals(normals);

	if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_VERTICES_TEXTURECOORDS ||
		exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_ALL)
		DumpTexCoords(texCoords);

	DumpFaces(indices, exporterSwitch);
	ofs.close();
	return true;
}

template<typename T>
bool Lyra::MeshExporter<T>::Create(std::string & fileName, std::string & filePath)
{
	ofs.open(filePath + fileName, std::ios::out);

	if (!ofs.is_open()) {
		std::cerr << (filePath + fileName).c_str() << " Can not be opened\n";
		return false;
	}
	return true;
}

template<typename T>
void Lyra::MeshExporter<T>::DumpVertices(std::vector<vec3<T>>& vertices)
{
	for (auto& v : vertices) {
		ofs << "v " << v(0) << " " << v(1) << " " << v(2) << std::endl;
	}
}

template<typename T>
void Lyra::MeshExporter<T>::DumpTexCoords(std::vector<vec2<T>>& texCoords)
{
	for (auto& coord : texCoords) {
		ofs << "vt " << coord(0) << " " << coord(1) << std::endl;
	}
}

template<typename T>
void Lyra::MeshExporter<T>::DumpNormals(std::vector<vec3<T>>& normals)
{
	for (auto& n : normals) {
		ofs << "vn " << n(0) << " " << n(1) << " " << n(2) << std::endl;
	}
}

template<typename T>
void Lyra::MeshExporter<T>::DumpFaces(std::vector<vec3<uint32>>& indices, Lyra::MeshExporterSwitch exporterSwitch)
{
	for (auto& index : indices) {
		//obj 文件的编号从1开始编号
		vec3<uint32> tmp = index + vec3<uint32>(1, 1, 1);
		if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_VERTICES) {
			ofs << "f " << tmp(0) << " ";
			ofs << tmp(1) << " ";
			ofs << tmp(2) << std::endl;
		}
		else if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_VERTICES_NORMALS) {
			ofs << "f " << tmp(0) << "//" << tmp(0) << " ";
			ofs << tmp(1) << "//" << tmp(1) << " ";
			ofs << tmp(2) << "//" << tmp(2) << std::endl;
		}
		else if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_VERTICES_TEXTURECOORDS) {
			ofs << "f " << tmp(0) << "/" << tmp(0) << " ";
			ofs << tmp(1) << "/" << tmp(1) << " ";
			ofs << tmp(2) << "/" << tmp(2) << std::endl;
		}
		else if (exporterSwitch == MeshExporterSwitch::MESH_EXPORTER_ALL) {
			ofs << "f " << tmp(0) << "/" << tmp(0) << "/" << tmp(0) << " ";
			ofs << tmp(1) << "/" << tmp(1) << "/" << tmp(1) << " ";
			ofs << tmp(2) << "/" << tmp(2) << "/" << tmp(2) << std::endl;
		}
	}
}
