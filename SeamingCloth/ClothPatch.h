#pragma once

#include"ClothPatchParms.h"
#include"TrianglePatch.h"
#include"AdjacentTrianglePatch.h"
#include"LyraFunction.h"
#include"ClothBVH.h"

#include<GraphicHelper\Model.h>
#include<GraphicHelper\Camera.h>

#include<set>
#include<memory>
#include<iostream>
#include<vector>

namespace Lyra
{
	template<typename T> class ClothPatch;

	namespace gph = GraphicHelper;
}

namespace Lyra
{
	template<typename T>
	class ClothPatch
	{
	public:
		ClothPatch() = default;

	public:
		
		/*Interface to private members*/
		ClothPatchParms<T> &Parms() { return parms; }
		std::vector<Particle<T>> &Particles() { return particles; }
		std::vector<vec3<uint32>> &TriangleIndices() { return triangleIndices; }
		std::vector<vec3<uint32>> &GlobalTriangleIndices() { return globalTriangleIndices; }
		gph::ModelPointer<T> PatchMesh() { return patchMesh; }
		std::vector<TrianglePatch<T>>& TrianglePatches() { return trianglePatches; }
		uint32_sp &Mapping() { return mapping; }
		uint32 NumberOfParticle() { return numberOfParticle; }

		uint32 SetBias(uint32 bias) { this->bias = bias; return bias; }

		uint32 Bias() { return bias; }

		/*Init Functions*/

		void LoadPatch(ClothPatchParms<T> &parms);
		void DebugInfo();
		//使用particle列表和triangleIndices列表建立trianglePatches和adjacentTrianglePatches
		//particles这里使用cloth传来的particles，但是索引用自己的索引
		void Create(std::vector<Particle<T>> &particles);

		void ReGeneratePatchInfo(std::vector<Particle<T>> &particles);

		/*Rendering Functions*/

		void GlBind();

		void GlUpdate();

		void GlDraw(Shader<T> &shader, bool lineMode);

		void Rendering(std::vector<Particle<T>> &particles,gph::Camera<T> &camera, bool lineMode);

		/*Simulation Functions*/
		void ApplyWind(vec3<T> &windVelocity, T density, T cod);

		void ApplyPlaneForce(PlaneForceSwitch &globalSwitch);

		void ApplySpaceForce(SpaceForceSwitch &spaceForceSwitch);

		/*Collision Detection Functions*/
		void CollisionDetectWithRigidbody(objectBvh_sp<T> objectBvh, CollisionResults_C2O<T>& collsionResult);

		//建立patch的BVH
		void BuildBVH(uint32 leafSize, shader_sp<T> shader, bool draw);

	private:
		//用于初始化particles和triangleIndices列表
		void Init(ClothPatchParms<T> &parms);
		//使用自己的particle和自己的triangleIndices建立trianglePatches和adjacentTrianglePatches
		//用于只有patch时的模拟
		void Create();
		//使用密度重新计算每个particle的质量
		void RevisedMass();
		//生成邻接三角形列表
		void GenerateAdjacentTrianglePatches(std::vector<Particle<T>> &particles, ClothPatchParms<T> &parms);
		//生成三角形列表
		void GenerateTrianglePatches(std::vector<Particle<T>> &particles, ClothPatchParms<T> &parms);
		

	private:
		//首次先得到particle和triangleIndices，然后在cloth里面把index换成全局的，生成一次trianglePathes和adjTriangles
		//在达到缝合条件后把triangleIndices沿缝合线重新组织一次，然后再生成新的trianglePathes和adjTriangles，后面模拟就没有问题了

		//该patch的参数
		ClothPatchParms<T> parms;
		//patch的mesh信息
		gph::ModelPointer<T> patchMesh;
		uint32 bias;
		uint32 numberOfParticle;

		/*simulation 用*/
		uint32_sp mapping;
		std::vector<Particle<T>> particles;
		//模拟用全局索引，方便缝合以及缝合后缝合点的合并
		std::vector<vec3<uint32>> globalTriangleIndices;
		std::vector<TrianglePatch<T>> trianglePatches;
		std::vector<AdjacentTrianglePatch<T>> adjacentTrianglePatches;

		/*碰撞检测和碰撞响应用*/
		ClothBVH<T> clothBvh;
		ClothBVH<T> clothBvhPost;

		/*Rendering 用*/
		//渲染用局部索引，方便引用UV和控制哪些patch渲染，哪些不渲染
		std::vector<vec3<uint32>> triangleIndices;
		std::vector<vec3<T>> positionVec;
		std::vector<vec3<T>> velocityVec;
		std::vector<vec3<T>> acclerationVec;
		std::vector<vec2<T>> uvCoordVec;
		std::vector<vec3<T>> normalVec;

		uint32_sp VAO;
		uint32_sp VBO;
		uint32_sp uvVBO;
		uint32_sp normalVBO;
		uint32_sp velocityVBO;
		uint32_sp acclerationVBO;

		uint32_sp EBO;
	};

	template<typename T>
	using clothPatch_sp = std::shared_ptr<ClothPatch<T>>;
	template<typename T>
	using clothPatch_up = std::unique_ptr<ClothPatch<T>>;
	template<typename T>
	using clothPatch_pt = ClothPatch<T> *;
}

template<typename T>
void Lyra::ClothPatch<T>::DebugInfo()
{
	std::cout << "Debug Info\n\n";
}

template<typename T>
void Lyra::ClothPatch<T>::LoadPatch(ClothPatchParms<T> &parms)
{
	this->parms = parms;

	/*auto &stickPoints = parms.stickPoints;

	for (auto &stickPoint : *(stickPoints)) {
		std::cout << stickPoint << "\n";
	}*/

	patchMesh = std::make_shared<gph::Model<T>>();
	patchMesh->LoadModel(parms.path + parms.name);

	Init(parms);

	//if (parms.patchMode == ClothPatchMode::LYRA_CLOTH_PATCH_SINGLE) {
	//	//如果是单个patch就按原来的方式初始化和渲染
	//}
	//else if (parms.patchMode == ClothPatchMode::LYRA_CLOTH_PATCH_SEAMING) {
	//	//如果是缝合模式，就只初始化particles和triangleIndices
	//}

}
template<typename T>
void Lyra::ClothPatch<T>::Init(ClothPatchParms<T> &parms)
{
	GraphicHelper::Mesh<T> &mesh = patchMesh->Meshes()[0];
	auto &vertices = mesh.Vertices();

	//初始化三角形列表
	auto &indices = mesh.Indices();
	for (uint32 i = 0; i < indices.size(); i += 3) {
		//printf_s("%d %d %d\n", indices[i], indices[i + 1], indices[i + 2]);
		triangleIndices.push_back(vec3<uint32>(indices[i], indices[i + 1], indices[i + 2]));
	}

	/*printf_s("\n\n");
	system("pause");*/

	//初始化顶点和渲染信息
	for (int i = 0; i < vertices.size(); i++) {

		auto &vertex = vertices[i];

		vec3<T> posTmp(vertex.position.x, vertex.position.y, vertex.position.z);
		vec2<T> uvTmp(vertex.texureCoords.x, vertex.texureCoords.y);
		vec2<T> uvScaleTmp = parms.scale * uvTmp;
		vec3<T> normalTmp(vertex.normal.x, vertex.normal.y, vertex.normal.z);
		vec2<T> planePosTmp(vertex.position.x, vertex.position.y);

		positionVec.push_back(posTmp);
		uvCoordVec.push_back(uvTmp);

		if (parms.initState == ClothPatchInitState::LYRA_CLOTH_PATCH_PLANE)
			particles.push_back(Particle<T>(planePosTmp, posTmp));
		else if (parms.initState == ClothPatchInitState::LYRA_CLOTH_PATCH_NON_PLANE)
			particles.push_back(Particle<T>(uvScaleTmp, posTmp));
		else {
			std::cerr << "ClothPatchInitState is not specified!\n\n";
			system("pause");
			exit(0);
		}

		velocityVec.push_back(particles[i].velocity);
		acclerationVec.push_back(particles[i].acceleration);
		normalVec.push_back(normalTmp);
	}

	//按照密度来修改每个particle的质量
	RevisedMass();

	numberOfParticle = vertices.size();
}

template<typename T>
void Lyra::ClothPatch<T>::RevisedMass()
{
	GraphicHelper::Mesh<T> &mesh = patchMesh->Meshes()[0];
	auto &indices = mesh.Indices();

	std::vector<T> triangleArea;
	for (int i = 0; i < indices.size(); i += 3) {

		uint32 ind1 = indices[i];
		uint32 ind2 = indices[i + 1];
		uint32 ind3 = indices[i + 2];

		vec2<T> x1 = particles[ind1].planeCoordinate;
		vec2<T> x2 = particles[ind2].planeCoordinate;
		vec2<T> x3 = particles[ind3].planeCoordinate;

		vec2<T> x21 = x2 - x1;
		vec2<T> x31 = x3 - x1;
		vec2<T> x23 = x2 - x3;

		/*T l21 = glm::length(x21);
		T l31 = glm::length(x31);
		T l23 = glm::length(x23);*/

		T l21 = x21.norm();
		T l31 = x31.norm();
		T l23 = x23.norm();

		T p = 1.f / 2 * (l21 + l31 + l23);

		T area = std::sqrt(p*(p - l21)*(p - l31)*(p - l23));

		triangleArea.push_back(area);
	}

	int adjTriangleNum = 0;
	std::vector<uint32> triangleIndexTmp;
	for (uint32 i = 0; i < particles.size(); i++) {
		adjTriangleNum = 0;
		//用index确定是哪些三角形
		triangleIndexTmp.clear();
		for (auto &triIndex : triangleIndices) {
			if (triIndex[0] == i || triIndex[1] == i || triIndex[2] == i) {
				triangleIndexTmp.push_back(adjTriangleNum);
			}
			adjTriangleNum++;
		}

		T totalMass = T(0);
		for (uint32 &index : triangleIndexTmp) {
			totalMass += triangleArea[index] * parms.density;
		}

		//质量如果过小就把它设定为固定值
		particles[i].mass = 1.0;/*totalMass / 3.f;*/

		//std::cout << particles[i].mass << "\n";

		if (Lyra::IsZero(particles[i].mass)) {
			std::cerr << "particle's mass is Zero!\n";
		}
	}
}
template<typename T>
void Lyra::ClothPatch<T>::Create(std::vector<Particle<T>> &particles)
{
	//设置stickPoints
	for (auto &stickPoint : *(parms.stickPoints)) {
		particles[stickPoint].movable = false;
	}
	//三角形patch列表
	GenerateTrianglePatches(particles,parms);
	//邻接三角形patch列表
	GenerateAdjacentTrianglePatches(particles,parms);

}
template<typename T>
void Lyra::ClothPatch<T>::Create()
{

}

template<typename T>
void Lyra::ClothPatch<T>::GenerateTrianglePatches(std::vector<Particle<T>> &particles,ClothPatchParms<T> &parms)
{
	for (auto &index : globalTriangleIndices) {
		trianglePatches.push_back(TrianglePatch<T>(&particles[index(0)], &particles[index(1)], &particles[index(2)],
												   index(0), index(1), index(2),
												   parms.stretchingFactor, parms.shearingFactor,
												   parms.dampingStretchFactor, parms.dampingShearFactor,
												   parms.stretchScaleUDir, parms.stretchScaleVDir,parms.alpha));
	}

	/*for (auto &triPatch : trianglePatches) {
		std::cout << triPatch.index1() <<" "<< triPatch.index2()<<" "<< triPatch.index3() << "\n";
	}*/
}

template<typename T>
void Lyra::ClothPatch<T>::GenerateAdjacentTrianglePatches(std::vector<Particle<T>> &particles,ClothPatchParms<T> &parms)
{
	T bc = parms.bendingFactor;
	T dbc = parms.dampingBendingFactor;

	for (auto iti = globalTriangleIndices.begin(); iti != globalTriangleIndices.end(); iti++) {
		for (auto itj = iti + 1; itj != globalTriangleIndices.end(); itj++) {

			std::set<uint32> tmp;		//存储不同particle的索引并且对有多少个particle进行计数
			tmp.insert((*iti)(0));
			tmp.insert((*iti)(1));
			tmp.insert((*iti)(2));
			tmp.insert((*itj)(0));
			tmp.insert((*itj)(1));
			tmp.insert((*itj)(2));

			if (tmp.size() == 4) {		//恰好构成邻接三角形
				uint32 aux[4], k = 0;
				for (auto &t : tmp) {
					aux[k++] = t;
				}

				if (((aux[0] == (*iti)(0) || aux[0] == (*iti)(1) || aux[0] == (*iti)(2))
					 && (aux[0] == (*itj)(0) || aux[0] == (*itj)(1) || aux[0] == (*itj)(2)))
					&& ((aux[1] == (*iti)(0) || aux[1] == (*iti)(1) || aux[1] == (*iti)(2))
						&& (aux[1] == (*itj)(0) || aux[1] == (*itj)(1) || aux[1] == (*itj)(2))))		//aux[0],aux[1]是共同的顶点

					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[2]],
																			   &particles[aux[3]],
																			   &particles[aux[0]],
																			   &particles[aux[1]],
																			   aux[2], aux[3], aux[0], aux[1],
																			   bc, dbc));
				else if (((aux[0] == (*iti)(0) || aux[0] == (*iti)(1) || aux[0] == (*iti)(2))
						  && (aux[0] == (*itj)(0) || aux[0] == (*itj)(1) || aux[0] == (*itj)(2)))
						 && ((aux[2] == (*iti)(0) || aux[2] == (*iti)(1) || aux[2] == (*iti)(2))
							 && (aux[2] == (*itj)(0) || aux[2] == (*itj)(1) || aux[2] == (*itj)(2))))		//aux[0],aux[2]是共同的顶点

					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[1]],
																			   &particles[aux[3]],
																			   &particles[aux[0]],
																			   &particles[aux[2]],
																			   aux[1], aux[3], aux[0], aux[2],
																			   bc, dbc));
				else if (((aux[0] == (*iti)(0) || aux[0] == (*iti)(1) || aux[0] == (*iti)(2))
						  && (aux[0] == (*itj)(0) || aux[0] == (*itj)(1) || aux[0] == (*itj)(2)))
						 && ((aux[3] == (*iti)(0) || aux[3] == (*iti)(1) || aux[3] == (*iti)(2))
							 && (aux[3] == (*itj)(0) || aux[3] == (*itj)(1) || aux[3] == (*itj)(2))))		//aux[0],aux[3]是共同的顶点

					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[1]],
																			   &particles[aux[2]],
																			   &particles[aux[0]],
																			   &particles[aux[3]],
																			   aux[1], aux[2], aux[0], aux[3],
																			   bc, dbc));
				else if (((aux[1] == (*iti)(0) || aux[1] == (*iti)(1) || aux[1] == (*iti)(2))
						  && (aux[1] == (*itj)(0) || aux[1] == (*itj)(1) || aux[1] == (*itj)(2)))
						 && ((aux[2] == (*iti)(0) || aux[2] == (*iti)(1) || aux[2] == (*iti)(2))
							 && (aux[2] == (*itj)(0) || aux[2] == (*itj)(1) || aux[2] == (*itj)(2))))		//aux[1],aux[2]是共同的顶点

					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[0]],
																			   &particles[aux[3]],
																			   &particles[aux[1]],
																			   &particles[aux[2]],
																			   aux[0], aux[3], aux[1], aux[2],
																			   bc, dbc));
				else if (((aux[1] == (*iti)(0) || aux[1] == (*iti)(1) || aux[1] == (*iti)(2))
						  && (aux[1] == (*itj)(0) || aux[1] == (*itj)(1) || aux[1] == (*itj)(2)))
						 && ((aux[3] == (*iti)(0) || aux[3] == (*iti)(1) || aux[3] == (*iti)(2))
							 && (aux[3] == (*itj)(0) || aux[3] == (*itj)(1) || aux[3] == (*itj)(2))))		//aux[1],aux[3]是共同的顶点

					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[0]],
																			   &particles[aux[2]],
																			   &particles[aux[1]],
																			   &particles[aux[3]],
																			   aux[0], aux[2], aux[1], aux[3],
																			   bc, dbc));
				else if (((aux[2] == (*iti)(0) || aux[2] == (*iti)(1) || aux[2] == (*iti)(2))
						  && (aux[2] == (*itj)(0) || aux[2] == (*itj)(1) || aux[2] == (*itj)(2)))
						 && ((aux[3] == (*iti)(0) || aux[3] == (*iti)(1) || aux[3] == (*iti)(2))
							 && (aux[3] == (*itj)(0) || aux[3] == (*itj)(1) || aux[3] == (*itj)(2))))		//aux[2],aux[3]是共同的顶点
					adjacentTrianglePatches.push_back(AdjacentTrianglePatch<T>(&particles[aux[0]],
																			   &particles[aux[1]],
																			   &particles[aux[2]],
																			   &particles[aux[3]],
																			   aux[0], aux[1], aux[2], aux[3],
																			   bc, dbc));
			}
		}
	}
}

template<typename T>
void Lyra::ClothPatch<T>::ReGeneratePatchInfo(std::vector<Particle<T>> &particles)
{
	trianglePatches.clear();
	adjacentTrianglePatches.clear();
	//三角形patch列表
	GenerateTrianglePatches(particles, parms);
	//邻接三角形patch列表
	GenerateAdjacentTrianglePatches(particles, parms);
}

template<typename T>
void Lyra::ClothPatch<T>::GlBind()
{
	VAO.reset(new uint32());
	VBO.reset(new uint32());
	uvVBO.reset(new uint32());
	EBO.reset(new uint32());
	normalVBO.reset(new uint32());
	velocityVBO.reset(new uint32());
	acclerationVBO.reset(new uint32());

	//这里的函数要特别注意，很多只有一个字母之差，很容易有bug
	glGenVertexArrays(1, VAO.get());
	glGenBuffers(1, VBO.get());
	glGenBuffers(1, EBO.get());
	glGenBuffers(1, uvVBO.get());
	glGenBuffers(1, normalVBO.get());
	glGenBuffers(1, velocityVBO.get());
	glGenBuffers(1, acclerationVBO.get());

	glBindVertexArray(*VAO);

	//顶点位置属性
	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferData(GL_ARRAY_BUFFER, positionVec.size() * sizeof(vec3<T>), &(positionVec[0]), GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	if (typeid(T) == typeid(float))
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3<T>), (void*)0);
	else if (typeid(T) == typeid(double))
		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(vec3<T>), (void*)0);

	//顶点法线属性
	glBindBuffer(GL_ARRAY_BUFFER, *normalVBO);
	glBufferData(GL_ARRAY_BUFFER, normalVec.size() * sizeof(vec3<T>), &(normalVec[0]), GL_STREAM_DRAW);
	glEnableVertexAttribArray(1);
	if (typeid(T) == typeid(float))
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3<T>), (void*)0);
	else if(typeid(T) == typeid(double))
		glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, sizeof(vec3<T>), (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangleIndices.size() * sizeof(vec3<uint32>), &(triangleIndices[0]), GL_STREAM_DRAW);

	//其他属性
	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	glBindVertexArray(0);
}

template<typename T>
void Lyra::ClothPatch<T>::GlUpdate()
{
	glBindVertexArray(*VAO);

	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, positionVec.size() * sizeof(vec3<T>), &(positionVec[0]));

	glBindVertexArray(0);
}

template<typename T>
void Lyra::ClothPatch<T>::GlDraw(Shader<T> &shader, bool lineMode)
{
	if (lineMode) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	//glPointSize(5.f);
	glBindVertexArray(*VAO);

	//glPointSize(3.f);
	glDrawElements(GL_POINTS, triangleIndices.size() * 3, GL_UNSIGNED_INT, 0);
	glDrawElements(GL_TRIANGLES, triangleIndices.size() * 3, GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

template<typename T>
void Lyra::ClothPatch<T>::Rendering(std::vector<Particle<T>> &particles, gph::Camera<T> &camera, bool lineMode)
{
	//从global particle池拉进新的数据
	for (uint32 i = 0; i < numberOfParticle; i++) {
		uint32 indTmp = mapping.get()[i];
		positionVec[i] = particles[indTmp].position;
	}

	glm::mat<4, 4, T> modelMat(1);
	glm::mat<4, 4, T> viewMat(1);
	glm::mat<4, 4, T> projectionMat(1);
	glm::mat<4, 4, T> MVP(1);

	MVP = camera.GetMVPMatrix();
	parms.shader.use();
	parms.shader.setMat4("MVP", MVP);

	GlDraw(parms.shader, lineMode);
}

template<typename T>
void Lyra::ClothPatch<T>::ApplyWind(vec3<T> &windVelocity,T density,T cod)
{

	for (auto &tri : trianglePatches) {

		vec3<T> normal = tri.Normal();

		vec3<T> surfaceVelocity = (tri.X0()->velocity + tri.X1()->velocity + tri.X2()->velocity) / 3.;

		vec3<T> relativeVelocity = surfaceVelocity - windVelocity;

		T a0 = tri.Area();

		T a = (a0 / relativeVelocity.norm())*relativeVelocity.dot(normal);

		vec3<T> wind = normal * (-0.5 * density * cod * a * std::pow(relativeVelocity.norm(), 2));

		//std::cout << wind << "\n\n";

		tri.X0()->ApplyForce(wind);
		tri.X1()->ApplyForce(wind);
		tri.X2()->ApplyForce(wind);
	}
}

template<typename T>
void Lyra::ClothPatch<T>::ApplyPlaneForce(PlaneForceSwitch &globalSwitch)
{
	bool enableStretch = parms.planeForceSwitch.enableStretchForce & globalSwitch.enableStretchForce;
	bool enableShear = parms.planeForceSwitch.enableShearForce & globalSwitch.enableShearForce;
	bool enableDampingStretch = parms.planeForceSwitch.enableDampingStretchForce & globalSwitch.enableDampingStretchForce;
	bool enableDampingShear = parms.planeForceSwitch.enableDampingShearForce & globalSwitch.enableDampingShearForce;

	for (auto &tri : trianglePatches) {
		
		if (enableStretch)
			tri.ExplicitStretchForce();

		if (enableDampingStretch)
			tri.ExplicitDampingStretchForce();

		if (enableShear)
			tri.ExplicitShearForce();

		if (enableDampingShear)
			tri.ExplicitDampingShearForce();
		
	}
}

template<typename T>
void Lyra::ClothPatch<T>::ApplySpaceForce(SpaceForceSwitch &spaceForceSwitch)
{
	bool enableBendingForce = parms.spaceForceSwitch.enableBendingForce & spaceForceSwitch.enableBendingForce;
	bool enableDampingBendingForce = parms.spaceForceSwitch.enableDampingBendingForce & spaceForceSwitch.enableDampingBendingForce;

	for (auto &adjTri : adjacentTrianglePatches) {
		if (enableBendingForce)
			adjTri.ExplicitBendingForce();

		if (enableDampingBendingForce)
			adjTri.ExplicitDampingBendingForce();
	}
}

template<typename T>
void Lyra::ClothPatch<T>::BuildBVH(uint32 leafSize, shader_sp<T> shader, bool draw)
{
	clothBvh.ReBuild(trianglePatches, leafSize, ClothBvhCategory::CLOTH_BVH_PRE, shader, draw);
	//clothBvhPost.ReBuild(trianglePatches, leafSize, ClothBvhCategory::CLOTH_BVH_POST, shader, draw);
	//clothBvh.Build(trianglePatches, leafSize, shader, draw);
}

template<typename T>
void Lyra::ClothPatch<T>::CollisionDetectWithRigidbody(objectBvh_sp<T> objectBvh, CollisionResults_C2O<T>& collsionResult)
{
	/*std::cout << clothBvh.NLevels() << " " << objectBvh->NLevel() << "\n";
	system("pause");*/
	clothBvh.CollisionWithObjBVH(*objectBvh, collsionResult);
	//clothBvhPost.CollisionWithObjBVH(*objectBvh, collsionResult);
}