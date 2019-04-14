#pragma once
#include"Common.h"
#include"BBox.h"
#include"Shader.h"

#include<set>

#include<GraphicHelper\Model.h>
#include<GraphicHelper\Camera.h>

namespace Lyra
{
	using TriangleIndexs = vec3<uint32>;
	template<typename T> class UniformBVH;
}

namespace Lyra
{
	template<typename T>
	class UniformBVH
	{
	public:
		void SetExternalBBox(vec3<T> &center, vec3<T> &halfExtent, Shader<T> &shader, bool draw = false);

		void BuildBBoxVH(gph::ModelPointer<float> &model, vec3<T> &subHalfExtent, Shader<T> &shader);

		void DrawSubBBoxs(gph::Camera<T> &camera);

		void DrawExternalBBox(gph::Camera<T> &camera);

		vec3<T> &CellSize() { return cellSize; }

		vec3<T> &HalfExtent() { return halfExtent; }

		vec3<T> &SubHalfExtent() { return subHalfExtent; }

		std::vector<vec3<T>> &Positions() { return positions; }

		bBox_sp<T> &ExternelBBox() { return bBox; }

		std::vector<bBox_sp<T>> &SubBBoxs() { return subBboxs; }

		std::vector<std::vector<TriangleIndexs>> &TriangleSets() { return triangleSets; }

	private:
		void UniformSpatialPartition(vec3<T> &subBboxHalfExtent, Shader<T> &shader);

		void MeshPartition(gph::ModelPointer<float> &model);
	private:
		bBox_sp<T> bBox;
		std::vector<bBox_sp<T>> subBboxs;
		//对应subBbox存放的三角形索引
		std::vector<std::vector<TriangleIndexs>> triangleSets;
		//存放顶点的位置，用来做碰撞检测用
		std::vector<vec3<T>> positions;

		bool draw;

		vec3<T> cellSize;
		vec3<T> halfExtent;
		vec3<T> subHalfExtent;
	};

	template<typename T>
	using uniformBvh_sp = std::shared_ptr<UniformBVH<T>>;
	template<typename T>
	using uniformBvh_up = std::unique_ptr<UniformBVH<T>>;
	template<typename T>
	using uniformBvh_pt = UniformBVH<T> *;
}

template<typename T>
void Lyra::UniformBVH<T>::SetExternalBBox(vec3<T> &center, vec3<T> &halfExtent,Shader<T> &shader, bool draw)
{
	this->draw = draw;
	this->halfExtent = halfExtent;
	bBox = std::make_shared<BBox<T>>();
	bBox->Init(center, halfExtent, shader, draw);
}

template<typename T>
void Lyra::UniformBVH<T>::UniformSpatialPartition(vec3<T> &subBboxHalfExtent, Shader<T> &shader)
{
	vec3<T> minCorner = bBox->minCorner;
	vec3<T> maxCorner = bBox->maxCorner;

	//minCorner + subBboxHalfExtent是最小的center
	vec3<T> minCenter = minCorner + subBboxHalfExtent;

	T deltaX = 2 * subBboxHalfExtent(0);
	T deltaY = 2 * subBboxHalfExtent(1);
	T deltaZ = 2 * subBboxHalfExtent(2);

	for (T x = minCenter(0); x < maxCorner(0); x += deltaX) {
		for (T y = minCenter(1); y < maxCorner(1); y += deltaY) {
			for (T z = minCenter(2); z < maxCorner(2); z += deltaZ) {

				vec3<T> auxCenter(x, y, z);
				subBboxs.push_back(bBox_sp<T>(new BBox<T>(auxCenter, subBboxHalfExtent, shader, draw)));
			}
		}
	}
	printf_s("subBBox size: %d\n", subBboxs.size());
}


template<typename T>
void Lyra::UniformBVH<T>::MeshPartition(gph::ModelPointer<float> &model)
{
	std::vector<TriangleIndexs> triangleSet;

	gph::Mesh<T> &mesh = model->Meshes()[0];

	auto &indices = mesh.Indices();

	for (gph::Vertex<T> &vertex : mesh.Vertices()) {

		vec3<T> tmp(vertex.position.x, vertex.position.y, vertex.position.z);
		positions.push_back(tmp);
	}

	//float scale = 1.04f;
	//glm::vec3 scaleVec(scale, scale, scale);
	//glm::mat4 scaleMat = glm::mat4(1.f);
	//scaleMat = glm::scale(scaleMat, scaleVec);

	//for (GraphicHelper::Vertex<T> &vertex : mesh.Vertices()) {
	//	//std::cout << vertex.position.x << " " << vertex.position.y << " " << vertex.position.z << "\n";
	//	glm::vec4 tmp = glm::vec4(vertex.position, 1.f) * scaleMat;
	//	glm::vec3 aux;

	//	aux.x = tmp.x;
	//	aux.y = tmp.y;
	//	aux.z = tmp.z;

	//	//std::cout << aux.x << " " << aux.y << " " << aux.z << "\n";

	//	positions.push_back(aux);
	//}

	//对每一个bbox求在该bbox内的triangles
	for (auto &bBox : subBboxs) {
		//检查所有的三角形
		for (uint32 i = 0; i < indices.size(); i += 3) {
			uint32 v1 = indices[i];
			uint32 v2 = indices[i + 1];
			uint32 v3 = indices[i + 2];

			bool isInBox = bBox->PointInBox(positions[v1])
				|| bBox->PointInBox(positions[v2])
				|| bBox->PointInBox(positions[v3]);

			if (isInBox) {
				triangleSet.push_back(TriangleIndexs(v1, v2, v3));
			}
		}
		triangleSets.push_back(triangleSet);
		triangleSet.clear();
	}

	int num = 0;
	for (auto &ss : triangleSets) {
		num += ss.size();
	}

	printf_s("total triangle number in BBox: %d\n", num);
}


template<typename T>
void Lyra::UniformBVH<T>::BuildBBoxVH(gph::ModelPointer<float> &model, vec3<T> &subHalfExtent, Shader<T> &shader)
{
	this->cellSize = cellSize;
	this->subHalfExtent = vec3<T>(halfExtent(0) / cellSize(0), halfExtent(1) / cellSize(1), halfExtent(2) / cellSize(2));
	UniformSpatialPartition(subHalfExtent,shader);
	MeshPartition(model);
}

template<typename T>
void Lyra::UniformBVH<T>::DrawSubBBoxs(gph::Camera<T> &camera)
{
	if (!bBox->IsDraw()) {
		std::cerr << "Please enable bBox draw" << "\n\n";
		system("pause");
		return;
	}
	for (auto &bbox : subBboxs) {
		bbox->GlDraw(camera);
	}
}

template<typename T>
void Lyra::UniformBVH<T>::DrawExternalBBox(gph::Camera<T> &camera)
{
	if (bBox->IsDraw())
		bBox->GlDraw(camera);
	else {
		std::cerr << "Please enable bBox draw" << "\n\n";
		system("pause");
	}
}



