#pragma once

#include"Particle.h"
#include"ClothPatch.h"
#include"SeamingInfo.h"
#include"ForceSwitch.h"
#include"Shader.h"
#include"CollisionResult.h"

#include<GraphicHelper\Camera.h>

#include<vector>
#include<iostream>

namespace Lyra
{
	template<typename T> class Cloth;
	template<typename T> struct ClothParms;
	template<typename T> struct WindParms;

	namespace gph = GraphicHelper;
}

template<typename T>
struct Lyra::WindParms
{
	vec3<T> windVelocity;
	T windDensity;
	T windCod;

	WindParms<T> &operator=(WindParms<T> &parms)
	{
		windVelocity = parms.windVelocity;
		windDensity = parms.windDensity;
		windCod = parms.windCod;

		return *this;
	}
};

template<typename T>
struct Lyra::ClothParms
{
	vec3<T> gravity;
	WindParms<T> windParms;

	PlaneForceSwitch planeForceSwitch;
	SpaceForceSwitch spaceForceSwitch;

	T seamingForceFactor;
	T seamingThreshold;

	SeamingParms<T> seamingParms;

	T delta_t;
	T frameRate;		//FPS

	bool enableWind;
	bool enableGravity;
	bool enableSeaming;
	bool enableCollisionDetect;

	//for seaming
	Shader<T> shader;
	
	//for debug
	shader_sp<T> bvhShader;

	ClothParms<T> &operator=(ClothParms<T> &parms)
	{
		gravity = parms.gravity;
		windParms = parms.windParms;

		delta_t = parms.delta_t;
		frameRate = parms.frameRate;
		seamingForceFactor = parms.seamingForceFactor;

		seamingThreshold = parms.seamingThreshold;

		enableWind = parms.enableWind;
		enableGravity = parms.enableGravity;
		enableCollisionDetect = parms.enableCollisionDetect;

		planeForceSwitch = parms.planeForceSwitch;
		spaceForceSwitch = parms.spaceForceSwitch;

		seamingParms = parms.seamingParms;
		enableSeaming = parms.enableSeaming;

		shader = parms.shader;
		bvhShader = parms.bvhShader;

		return *this;
	}
};

//缝合放在Cloth里
namespace Lyra
{
	template<typename T>
	class Cloth
	{
	public:
		Cloth() = default;

	private:
		std::vector<clothPatch_sp<T>> patches;
		std::vector<Particle<T>> particles;

		std::vector<SeamingInfo<T>> seamingInfo;

		std::vector<vec3<T>> seamingPointsVec;

		ClothParms<T> parms;
		bool isSeaming;

		T elapseTime;

		uint32 numberOfParticle;

		//渲染缝合线用
		uint32_sp VAO;
		uint32_sp VBO;

	public:
		/*Init Functions*/
		bool AddPatch(clothPatch_sp<T> &patch);
		bool AddPatches(std::vector<clothPatch_sp<T>> &patches);
		bool AddSeamingInfo(SeamingInfo<T> &info);
		bool AddMultiSeamingInfo(std::vector<SeamingInfo<T>> &seamingInfo);
		bool Integrate(ClothParms<T>& parms);
		/*Simulation Functions*/
		void ApplyWind();
		void ApplyGravity();
		void ApplyPlaneForce();
		void ApplySpaceForce();
		void ApplyExternalForce();
		void ApplyInternalForce();
		void Simulate(Lyra::objectBvh_sp<T> objectBvh);
		void Seaming();
		/*Rendering Functions*/
		void Rendering(gph::Camera<T> &camera, bool lineMode);
		void GlBind();
		void GlUpdate();
		void GlDrawBvh(gph::Camera<T>& camera)
		{
			for (auto& patch : patches) {
				patch->GlDrawBvh(camera);
			}
		}
		/*Debug Functions*/
		void DebugSimulate(shader_sp<T> shader);
		void SimulationInfo();
		/*Auxiliary Functions*/
		void SaveClothMesh(std::string fileName, std::string& filePath,
			MeshExporterSwitch exportSwitch = MESH_EXPORTER_ALL);

	private:
		//Simulation
		void AdvanceStep();
		void UpdatePosition();
		void UpdateMiddleVelocity();
		void UpdatePesudoPosition();
		void UpdateMiddleVelocityWithVelcoityVerlet();
		void UpdatePesudoPositionWithVelocityVerlet();
		void UpdatePesudoVelocityWithVelocityVerlet();

		void UpdateDampingForceMiddleVelocity(VelocityUpdate updateCat);

		void ApplyDampingForceImplicit(VelocityUpdate updateCat);
		void UpdatePesudoPositionImplicit();
		void ApplyInternalForceExplicit();

		//用于cloth-cloth碰撞时用
		void EstimateMiddleVelocity();
		bool CheckSeamingCondition();
		void UpdatePatches();
		void UpdateSeamingPosition();
		//Rendering
		void GlBindSeaming();
		void GlUpdateSeaming();
		void GlRenderSeaming(gph::Camera<T> &camera);
		void GenerateParticleNormal();
		
		void BuildPatchBVH(uint32 leafSize = 4, shader_sp<T> shader = nullptr, bool draw = false);

		void GramentSimulate();
		void PatchSimulate(Lyra::objectBvh_sp<T> objectBvh);
		void ImplicitPatchSimulate(Lyra::objectBvh_sp<T> objectBvh);

		void CollisionDetectWithRigidbody(objectBvh_sp<T> objectBvh, CollisionResults_C2O<T>& collsionResult);
		void CollisionDetectWithOtherCloth();
		void CollisionDetectWithSelf();

		void SimpleCollisionResponseWithRigidbody(CollisionResults_C2O<T>& collsionResult,T fricationFactor,T dampingFactor);
		void SimpleEdge2EdgeResponseWithRigidbody(std::vector<Edge2Edge_C2O<T>>& edge2Edges, T fricationFactor, T dampingFactor);
		void SimplePoint2TriangleResponseWithRigidbody(std::vector<Vertex2Triangle_C2O<T>>& v2Triangles,T fricationFactor, T dampingFactor);

		void DebugUpdatePosition();
		void DebugCollisionResponse(CollisionResults_C2O<T>& collsionResult);
	};

	template<typename T>
	using cloth_sp = std::shared_ptr<Cloth<T>>;
	template<typename T>
	using cloth_up = std::unique_ptr<Cloth<T>>;
	template<typename T>
	using cloth_pt = Cloth<T> *;
}


template<typename T>
bool Lyra::Cloth<T>::AddPatch(clothPatch_sp<T> &patch)
{
	//把patch加入cloth中
	auto &parms = patch->Parms();
	/*std::cout << "AdddPatchFunc:\n";
	std::cout << "name: " << parms.name << "\n\n";*/
	patches.push_back(patch);

	return true;
}

template<typename T>
bool Lyra::Cloth<T>::AddPatches(std::vector<clothPatch_sp<T>> &patches)
{
	for (auto &patch : patches) {
		AddPatch(patch);
	}

	return true;
}

template<typename T>
bool Lyra::Cloth<T>::AddSeamingInfo(SeamingInfo<T> &info)
{
	/*std::cout << "AddSeamingInfo:\n\n";

	for (auto &pair : info.seamingPairs) {
		std::cout << pair.first << " " << pair.second << "\n";
	}

	std::cout << "\n";*/
	seamingInfo.push_back(info);
	return true;
}

template<typename T>
bool Lyra::Cloth<T>::AddMultiSeamingInfo(std::vector<SeamingInfo<T>> &seamingInfo)
{
	for (auto &info : seamingInfo) {
		AddSeamingInfo(info);
	}
	return true;
}

template<typename T>
bool Lyra::Cloth<T>::Integrate(ClothParms<T> &parms)
{
	isSeaming = false;
	this->parms = parms;
	elapseTime = T(0);

	uint32 bias = 0;
	uint32 count = 0;

	for (clothPatch_sp<T> &patch : patches) {
		numberOfParticle += patch->NumberOfParticle();
	}

	//printf_s("%d\n", numOfParticle);

	for (clothPatch_sp<T> &patch : patches) {
		//统一组织particles
		auto &ps = patch->Particles();
		for (auto &p : ps) {
			particles.push_back(p);
			count++;
		}
		//统一索引
		auto &indices = patch->TriangleIndices();
		auto &globalIndices = patch->GlobalTriangleIndices();
		auto &mapping = patch->Mapping();
		//记录映射关系
		mapping.reset(new uint32[ps.size()]);
		for (auto &index : indices) {
			 vec3<uint32> tmp = index + vec3<uint32>(bias, bias, bias);
			 globalIndices.push_back(tmp);
			 //局部映射到全局
			 mapping.get()[index(0)] = tmp(0);
			 mapping.get()[index(1)] = tmp(1);
			 mapping.get()[index(2)] = tmp(2);
		}

		/*for (uint32 i = 0; i < ps.size(); i++) {
			std::cout << mapping.get()[i] << "\n";
		}
		std::cout << "\n";
		system("pause");*/

		auto &parms = patch->Parms();
		auto &stickPoints = parms.stickPoints;

		//重新索引stickpoints
		for (auto &stickPoint : *(stickPoints)) {

			/*std::cout << stickPoint;*/
			stickPoint += bias;
			particles[stickPoint].movable = false;
			//std::cout << " " << stickPoint << "\n";
		}

		//为了缝合点的重新索引
		patch->SetBias(bias);
		bias += count;
	}

	//std::cout<<particles.size();

	//重新组织缝合信息
	for (auto &info : seamingInfo) {

		uint32 bias1 = info.patch1->Bias();
		uint32 bias2 = info.patch2->Bias();

		std::vector<SeamingPair> &seamingPairs = info.seamingPairs;

		for (auto &pair : seamingPairs) {
			pair.first += bias1;
			pair.second += bias2;

			//渲染缝合线需要的信息
			seamingPointsVec.push_back(particles[pair.first].position);
			seamingPointsVec.push_back(particles[pair.second].position);
		}
	}

	//对每一个patch，按重新索引后的位置建立三角形列表和邻接三角形列表
	for (clothPatch_sp<T> &patch : patches) {
		patch->Create(particles);
	}

	//Build BVH
	BuildPatchBVH();

	//生成每个patch的顶点法线用于渲染
	GenerateParticleNormal();

	//Output simulation information
	SimulationInfo();

	//之后就可以进行模拟了
	return true;
}

template<typename T>
void Lyra::Cloth<T>::SimulationInfo()
{
	std::cout << "######################Cloth Simulation Info######################\n\n";
	std::cout << "patch number: " << patches.size() << "\n";
	std::cout << "Simulation Category: ";
	if(parms.enableSeaming)
		std::cout << "GRAMENT\n";
	else
		std::cout << "PATCH\n";
	std::cout << "Enable Collsiion Detection: ";
	if (parms.enableCollisionDetect)
		std::cout << "True\n";
	else
		std::cout << "False\n";
}

template<typename T>
void Lyra::Cloth<T>::BuildPatchBVH(uint32 leafSize,shader_sp<T> shader,bool draw)
{
	for (auto& patch : patches) {
		patch->BuildBVH(leafSize, shader, draw);
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdatePesudoVelocityWithVelocityVerlet()
{
	for (auto& p : particles) {
		if (p.movable)
		p.UpdatePseudoVelocityWithVelocityVerlet(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdatePesudoPositionWithVelocityVerlet()
{
	for (auto& p : particles) {
		if (p.movable)
		p.UpdatePseudoPositionWithVelocityVerlet(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdatePosition()
{
	for (auto &p : particles) {
		p.UpdatePosition(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdateDampingForceMiddleVelocity(VelocityUpdate updateCat)
{
	for (auto& p : particles) {
		if (p.movable) {
			if (updateCat == VelocityUpdate::MIDDLE_VELOCITY)
				p.middleVelocity += p.velocity;
			else if (updateCat == VelocityUpdate::PSEUDO_VELOCITY)
				p.pseudoVelocity += p.middleVelocity;
		}
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyDampingForceImplicit(VelocityUpdate updateCat)
{
	for (auto& patch : patches) {
		patch->ApplyDampingPlaneForceImplicit(parms.planeForceSwitch, parms.delta_t, updateCat);
		//patch->ApplyDampingSpaceForceImplicit(parms.spaceForceSwitch, parms.delta_t);
	}

	UpdateDampingForceMiddleVelocity(updateCat);
}

template<typename T>
void Lyra::Cloth<T>::UpdatePesudoPositionImplicit()
{
	for (auto& p : particles) {
		if (p.movable)
			p.UpdatePesudoPositionImplicit(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyInternalForceExplicit()
{
	for (auto& patch : patches) {
		patch->ApplyPlaneForceExplicit(parms.planeForceSwitch);
		//patch->ApplySpaceForceExplicit(parms.spaceForceSwitch);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyGravity()
{
	for (auto &p : particles) {
		if (p.movable)
			p.ApplyGravity(parms.gravity);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyWind()
{
	vec3<T> windVelocity = parms.windParms.windVelocity;
	T density = parms.windParms.windDensity;
	T cod = parms.windParms.windCod;

	for (auto &patch : patches) {
		patch->ApplyWind(windVelocity,density,cod);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyExternalForce()
{
	//重力统一加，防止缝合点被加几次重力
	if (parms.enableGravity)
		ApplyGravity();
	//风力分开加
	if (parms.enableWind)
		ApplyWind();
}

template<typename T>
void Lyra::Cloth<T>::ApplyPlaneForce()
{
	for (auto &patch : patches) {
		patch->ApplyPlaneForce(parms.planeForceSwitch);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplySpaceForce()
{
	for (auto &patch : patches) {
		patch->ApplySpaceForce(parms.spaceForceSwitch);
	}
}

template<typename T>
void Lyra::Cloth<T>::ApplyInternalForce()
{
	ApplyPlaneForce();

	ApplySpaceForce();
}

template<typename T>
void Lyra::Cloth<T>::UpdateSeamingPosition()
{
	seamingPointsVec.clear();

	for (auto &info : seamingInfo) {

		std::vector<SeamingPair> &seamingPairs = info.seamingPairs;

		for (auto &pair : seamingPairs) {

			//渲染缝合线需要的信息
			seamingPointsVec.push_back(particles[pair.first].position);
			seamingPointsVec.push_back(particles[pair.second].position);
		}
	}
}

template<typename T>
void Lyra::Cloth<T>::DebugSimulate(shader_sp<T> shader)
{
	BuildPatchBVH(1, shader, true);
}

template<typename T>
void Lyra::Cloth<T>::GramentSimulate()
{
	//BuildPatchBVH(1);
	ApplyInternalForce();
	if (isSeaming)
		ApplyExternalForce();

	Seaming();

	UpdatePosition();

	if (!isSeaming) {
		UpdateSeamingPosition();
		isSeaming = CheckSeamingCondition();
	}

	//@(todo:)IMEX solver
	//ApplyExternalForce();
	//ApplyInternalForce();

	////seaming必须放在其他所有力之后根据加速度来修正
	//if (!isSeaming) {
	//	Seaming();
	//} else {
	//	//ApplyInternalForce();
	//	ApplyExternalForce();
	//	//std::cout << "施加外力:重力、风力\n";
	//}
	//
	//UpdatePosition();

	//if (!isSeaming) {
	//	UpdateSeamingPosition();
	//	isSeaming = CheckSeamingCondition();
	//	if (isSeaming) {
	//		UpdatePatches();
	//	}
	//}

	elapseTime += parms.delta_t;
	//printf_s("%f\n", elapseTime);
}

template<typename T>
void Lyra::Cloth<T>::CollisionDetectWithRigidbody(objectBvh_sp<T> objectBvh,CollisionResults_C2O<T>& collsionResult)
{
	for (auto& patch : patches) {
		patch->CollisionDetectWithRigidbody(objectBvh, collsionResult);
	}
}

template<typename T>
void Lyra::Cloth<T>::CollisionDetectWithOtherCloth()
{

}

template<typename T>
void Lyra::Cloth<T>::CollisionDetectWithSelf()
{

}

template<typename T>
void Lyra::Cloth<T>::SimpleEdge2EdgeResponseWithRigidbody(std::vector<Edge2Edge_C2O<T>>& edge2Edges, T fricationFactor, T dampingFactor)
{
	//利用middle velocity 计算 pseudo velocity
	//每个particle只响应一次
	//std::cout << edge2Edges.size() << "\n";
	std::set<particle_pt<T>> tmp;
	for (auto& edge : edge2Edges) {

		auto& cp0 = edge.clothEdge0.p0;
		auto& cp1 = edge.clothEdge0.p1;

		auto& op0 = edge.objectEdge0.p0;
		auto& op1 = edge.objectEdge0.p1;

		vec3<T> v0 = cp1->position - cp0->position;
		glm::vec<3, T> v1Tmp = op1->position - op0->position;
		vec3<T> v1(v1Tmp.x, v1Tmp.y, v1Tmp.z);

		//std::cout << v0.dot(v1) << "\n";

		vec3<T> normal = v0.cross(v1).normalized();
		/////////////////////v0顶点的响应/////////////////////////////
		if (tmp.find(cp0) == tmp.end())
		{
			vec3<T> p0v = cp0->middleVelocity;
			//法线速度
			vec3<T> p0vn = p0v.dot(normal) * normal;
			/*if (p0vn.dot(normal) < 0)
				p0vn = - p0vn;*/
			//切线速度
			vec3<T> p0vt = (p0v - p0vn);
			//std::cout << p0vt.norm() << "\n";
			if (p0vt.norm() >= fricationFactor * p0vn.norm()) {
				//std::cout << "2333\n";
				cp0->pseudoVelocity = p0vt - fricationFactor * p0vn.norm() * p0vt.normalized()
					- dampingFactor * p0vn;
			}
			else {
				//std::cout << "2333\n";
				cp0->pseudoVelocity = -dampingFactor * p0vn;
			}
			//std::cout << parms.delta_t << "\n";
			cp0->pseudoPosition = cp0->position + cp0->pseudoVelocity * parms.delta_t;
			tmp.insert(cp0);
		}
		
		/////////////////////v1顶点的响应/////////////////////////////
		if (tmp.find(cp1) == tmp.end()) {
			vec3<T> p1v = cp1->middleVelocity;
			//法线速度
			vec3<T> p1vn = p1v.dot(normal) * normal;
			/*if (p1vn.dot(normal) < 0)
				p1vn = -p1vn;*/
			/*if (p1vn.dot(normal) > 0)
				std::cout << "Emmmm\n";*/
				//切线速度
			vec3<T> p1vt = (p1v - p1vn);

			//std::cout << p1vt.norm() << "\n";
			if (p1vt.norm() >= fricationFactor * p1vn.norm()) {
				//std::cout << "2333\n";
				cp1->pseudoVelocity = p1vt - fricationFactor * p1vn.norm() * p1vt.normalized()
					- dampingFactor * p1vn;
			}
			else {
				//std::cout << "2333\n";
				cp1->pseudoVelocity = -dampingFactor * p1vn;
			}
			//std::cout << parms.delta_t << "\n";
			cp1->pseudoPosition = cp1->position + cp1->pseudoVelocity * parms.delta_t;
			tmp.insert(cp1);
		}	
	}

	//std::cout << tmp.size() << "\n";
}

template<typename T>
void Lyra::Cloth<T>::SimplePoint2TriangleResponseWithRigidbody(std::vector<Vertex2Triangle_C2O<T>>& v2Triangles, T fricationFactor, T dampingFactor)
{
	//利用 middle velocity 计算 pseudo velocity
	//每个particle只响应一次
	std::set<particle_pt<T>> tmp;
	for(auto& v2t:v2Triangles){
		vec3<T> v = v2t.v0->middleVelocity;

		if (tmp.find(v2t.v0) != tmp.end())
			continue;
	
		tmp.insert(v2t.v0);

		glm::vec<3, T> t0Pos = v2t.t0->position;
		glm::vec<3, T> t1Pos = v2t.t1->position;
		glm::vec<3, T> t2Pos = v2t.t2->position;

		glm::vec<3, T> t01Pos = t1Pos - t0Pos;
		glm::vec<3, T> t02Pos = t2Pos - t0Pos;

		glm::vec<3, T> normalTmp = glm::normalize(glm::cross(t01Pos, t02Pos));

		vec3<T> normal(normalTmp.x, normalTmp.y, normalTmp.z);
		//法线速度
		vec3<T> vn = v.dot(normal) * normal;
		if (vn.dot(normal) > 0)
			vn = -vn;
		//切线速度
		vec3<T> vt = (v - vn);
		//std::cout << vt.norm() << "\n";
		if (vt.norm() >= fricationFactor * vn.norm()) {
			//std::cout << "2333\n";
			v2t.v0->pseudoVelocity = vt - fricationFactor * vn.norm() * vt.normalized()
				- dampingFactor * vn;
		}
		else {
			//std::cout << "2333\n";
			v2t.v0->pseudoVelocity = -dampingFactor * vn;
		}
		//std::cout << parms.delta_t << "\n";
		v2t.v0->pseudoPosition = v2t.v0->position + v2t.v0->pseudoVelocity * parms.delta_t;
	}

	//std::cout << tmp.size() << "\n";
}

template<typename T>
void Lyra::Cloth<T>::SimpleCollisionResponseWithRigidbody(CollisionResults_C2O<T>& collsionResult,T fricationFactor, T dampingFactor)
{
	auto& edgeResults = collsionResult.edge2Edge;
	auto& v2TriangleResults = collsionResult.vertex2Triangle;
	auto& v2TriangleResults_ = collsionResult.vertex2Triangle_;

	//对v2TriangleResult_暂且不做响应
	SimpleEdge2EdgeResponseWithRigidbody(edgeResults, fricationFactor, dampingFactor);
	SimplePoint2TriangleResponseWithRigidbody(v2TriangleResults, fricationFactor, dampingFactor);

}

template<typename T>
void Lyra::Cloth<T>::DebugCollisionResponse(CollisionResults_C2O<T>& collsionResult)
{
	auto& edgeResults = collsionResult.edge2Edge;
	auto& v2TriangleResults = collsionResult.vertex2Triangle;
	auto& v2TriangleResults_ = collsionResult.vertex2Triangle_;

	for (auto& edge : edgeResults) {
		edge.clothEdge0.p0->isCollide = true;
		edge.clothEdge0.p1->isCollide = true;

		//std::cout << edge.clothEdge0.p0->pseudoPosition - edge.clothEdge0.p0->position << "\n";
	}

	for (auto& v2t : v2TriangleResults){
		v2t.v0->isCollide = true;
	}

	for (auto& v2t_ : v2TriangleResults_) {
		v2t_.t0->isCollide = true;
		v2t_.t1->isCollide = true;
		v2t_.t2->isCollide = true;
	}
}

template<typename T>
void Lyra::Cloth<T>::DebugUpdatePosition()
{
	for (auto& p : particles) {
		if (!p.isCollide) {
			p.UpdatePosition(parms.delta_t);
			//system("pause");
		}
	}
}

template<typename T>
void Lyra::Cloth<T>::EstimateMiddleVelocity()
{
	for (auto& p : particles) {
		if (p.movable)
			p.EstimateMiddleVelocity(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::AdvanceStep()
{
	for (auto& p : particles) {
		if (p.movable)
			p.AdvanceStep();
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdateMiddleVelocityWithVelcoityVerlet()
{
	for (auto& p : particles) {
		if(p.movable)
		p.UpdateMiddleVelocityWithVelocityVerlet(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::UpdateMiddleVelocity()
{
	for (auto& p : particles) {
		if (p.movable)
			p.UpdateMiddleVelocity(parms.delta_t);
	}
}

template<typename T>
void Lyra::Cloth<T>::ImplicitPatchSimulate(Lyra::objectBvh_sp<T> objectBvh)
{
	T friction = patches[0]->Parms().frictionFactorForObject;
	T damping = patches[0]->Parms().dampingFactorForObject;

	ApplyDampingForceImplicit(VelocityUpdate::MIDDLE_VELOCITY);
	UpdatePesudoPositionImplicit();
	ApplyExternalForce();
	ApplyInternalForceExplicit();
	UpdateMiddleVelocity();
	ApplyDampingForceImplicit(VelocityUpdate::PSEUDO_VELOCITY);

	//估计中间速度用于碰撞响应
	EstimateMiddleVelocity();

	if (parms.enableCollisionDetect) {

		CollisionResults_C2O<T> collisionResults_C2O;
		CollisionResults_C2C<T> collisionResults_C2C;

		BuildPatchBVH(1, parms.bvhShader, false);
		CollisionDetectWithRigidbody(objectBvh, collisionResults_C2O);

		if (collisionResults_C2O.edge2Edge.size() != 0
			|| collisionResults_C2O.vertex2Triangle.size() != 0
			|| collisionResults_C2O.vertex2Triangle_.size() != 0) {
			SimpleCollisionResponseWithRigidbody(collisionResults_C2O, friction, damping);
		}
	}

	AdvanceStep();
}

template<typename T>
void Lyra::Cloth<T>::PatchSimulate(objectBvh_sp<T> objectBvh)
{
	T friction = patches[0]->Parms().frictionFactorForObject;
	T damping = patches[0]->Parms().dampingFactorForObject;

	//计算t+0.5 * \delta t时刻的速度用于计算damping force
	UpdateMiddleVelocityWithVelcoityVerlet();
	//先计算下一时刻的位置用于碰撞检测
	UpdatePesudoPositionWithVelocityVerlet();
	//计算合力得到下一时刻的加速度
	ApplyInternalForce();
	ApplyExternalForce();
	//根据上面的信息获得下一时刻的速度
	UpdatePesudoVelocityWithVelocityVerlet();
	//估计中间速度用于碰撞响应
	EstimateMiddleVelocity();

	if (parms.enableCollisionDetect) {

		CollisionResults_C2O<T> collisionResults_C2O;
		CollisionResults_C2C<T> collisionResults_C2C;

		BuildPatchBVH(1, parms.bvhShader, false);
		CollisionDetectWithRigidbody(objectBvh, collisionResults_C2O);

		if (collisionResults_C2O.edge2Edge.size() != 0 
			|| collisionResults_C2O.vertex2Triangle.size() != 0
			|| collisionResults_C2O.vertex2Triangle_.size() != 0) {
			SimpleCollisionResponseWithRigidbody(collisionResults_C2O, friction, damping);
		}
	}
	AdvanceStep();
}

template<typename T>
void Lyra::Cloth<T>::UpdatePesudoPosition()
{
	for (auto& p : particles) {
		p.UpdatePseudoPosition(parms.delta_t);

		//std::cout << p.pseudoPosition << "\n";
	}
}

template<typename T>
void Lyra::Cloth<T>::Simulate(Lyra::objectBvh_sp<T> objectBvh)
{	
	if (parms.enableSeaming)
	{
		//整件缝合衣物的模拟
		if (patches.size() > 1)
			GramentSimulate();
		else if (patches.size() < 2) {
			std::cerr << "Number of patch is less than 2\n";
			exit(0);
		}
	}
	else
	{
		//单件布块的模拟,不接受多块不相关布块的模拟
		if (patches.size() != 1) {
			std::cerr << "Number of patch is less than 1,Simulator can't go on\n";
			exit(0);
		}

		ImplicitPatchSimulate(objectBvh);//PatchSimulate(objectBvh);
	}

	//生成新的顶点法线用于渲染
	GenerateParticleNormal();

	elapseTime += parms.delta_t;

	//std::cout << elapseTime << "\n";
}

template<typename T>
void Lyra::Cloth<T>::SaveClothMesh(std::string fileName, std::string& filePath,
	MeshExporterSwitch exportSwitch)
{
	char number[1000];
	for (uint32 i = 0; i < patches.size(); i++) {
		sprintf_s(number, "patch_%d_", i);
		fileName = std::string(number) + fileName;
		patches[i]->SaveMesh(fileName, filePath, exportSwitch);
	}
}

template<typename T>
void Lyra::Cloth<T>::Seaming()
{
	T force = parms.seamingParms.force;
	T relax = parms.seamingParms.relax;
	T damp = parms.seamingParms.damp;
	T atten = parms.seamingParms.atten;
	T attet = parms.seamingParms.attet;

	for (auto &sps : seamingInfo) {
		for (auto &sp : sps.seamingPairs) {


			auto &p1 = particles[sp.first];
			auto &p2 = particles[sp.second];

			T p1MassInv = 1./ p1.mass;
			T p2MassInv = 1./ p2.mass;

			//std::cout << p1MassInv <<" "<< p2MassInv << "\n";

			vec3<T> paPos = p1.position;
			vec3<T> pbPos = p2.position;

			vec3<T> paVel = p1.velocity;
			vec3<T> pbVel = p2.velocity;

			vec3<T> paAcc = p1.acceleration;
			vec3<T> pbAcc = p2.acceleration;

			//std::cout << paAcc << " " << pbAcc << "\n\n";

			vec3<T> relPos = pbPos - paPos;
			vec3<T> relVel = pbVel - paVel;
			vec3<T> relAcc = pbAcc - paAcc;

			//std::cout << relAcc <<"\n\n";

			vec3<T> relPosI = relPos.normalized();

			vec3<T> relVelN = relVel.dot(relPosI) * relPosI;
			vec3<T> relVelT = relVel - relVelN;
			vec3<T> relAccN = relAcc.dot(relPosI) * relPosI;
			vec3<T> relAccT = relAcc - relAccN;

			//std::cout << relPos << "\n\n";
			//std::cout << relVelN << "\n\n";
			//std::cout << relVelT << "\n\n";
			//std::cout << relAccN << "\n\n";
			//std::cout << relAccT << "\n\n";

			vec3<T> deltaAcc = (force * relax) * relPos + 
				(force + relax)*relVelN + damp * relVelT + 
				atten * relAccN + attet * relAccT;

			//std::cout << deltaAcc << "\n\n";

			vec3<T> paDeltaAcc = p1MassInv / (p1MassInv + p2MassInv)*deltaAcc;
			vec3<T> pbDeltaAcc = - p2MassInv / (p1MassInv + p2MassInv)*deltaAcc;

		/*	p1.acceleration = paDeltaAcc;
			p2.acceleration = pbDeltaAcc;*/

			p1.CorrectAcceleration(paDeltaAcc);
			p2.CorrectAcceleration(pbDeltaAcc);

			////////////////////////////基于elastic force的模型（已证明没啥用的模型）////////////////////////////////////
			/*auto &p1 = particles[sp.first];
			auto &p2 = particles[sp.second];

			vec3<T> p1Pos = p1.position;
			vec3<T> p2Pos = p2.position;
			vec3<T> disVec = p2Pos - p1Pos;
			vec3<T> seamingForce = parms.seamingForceFactor * disVec;

			if (disVec.norm() > parms.seamingThreshold) {
				p1.ApplyForce(seamingForce);
				p2.ApplyForce(-seamingForce);
			} else {
				p1.ClearVelocity();
				p2.ClearVelocity();
			}*/

			//std::cout << seamingForce << "\n\n";

			//std::cout << disVec.norm() << "\n\n";

			//std::cout << p1.position << "\n\n" << p2.position << "\n\n";
		}
	}
	                                                                                                                                                                                                                                                                                                   
	//system("pause");
}

template<typename T>
bool Lyra::Cloth<T>::CheckSeamingCondition()
{
	if (elapseTime > parms.seamingParms.force / parms.frameRate * T(1.3))
		 return true;
	return false;
	//for (auto &sps : seamingInfo) {
	//	for (auto &sp : sps.seamingPairs) {

	//		auto &p1 = particles[sp.first];
	//		auto &p2 = particles[sp.second];

	//		vec3<T> p1Pos = p1.position;
	//		vec3<T> p2Pos = p2.position;
	//		T distance = (p2Pos - p1Pos).norm();

	//		//std::cout << distance << "\n";

	//		//现在用的模型没法支持所有点在距离非常近的时候才触发缝合条件
	//		if (distance > parms.seamingThreshold) {
	//			return false;
	//		}
	//	}
	//}

	//printf("isSeaming is True\n");
	////system("pause");
	//return true;
}

template<typename T>
void Lyra::Cloth<T>::UpdatePatches()
{

	clothPatch_sp<T> patch1, patch2;

	//按缝合信息先置换index
	for (auto &info : seamingInfo) {

		patch1 = info.patch1;
		patch2 = info.patch2;

		std::vector<vec3<uint32>> &indices = patch2->GlobalTriangleIndices();
		uint32_sp &mapping = patch2->Mapping();

		uint32 patch2BiasTmp = patch2->Bias();

		for (auto &pair : info.seamingPairs) {
			uint32 ind1 = pair.first;
			uint32 ind2 = pair.second;

			uint32 ind11 = ind1;
			uint32 ind22 = ind2 - patch2BiasTmp;
			//printf_s("%d %d\n", ind11, ind22);
			for (auto &index : indices) {
				//printf_s("%d %d %d\n", index(0), index(1), index(2));
				if (index(0) == ind2) {
					//global index - bias = local index
					//ind1 is global index
					mapping.get()[index(0) - patch2BiasTmp] = ind1;
					//for rendering
					index(0) = ind1;
				}
				else if (index(1) == ind2) {
					mapping.get()[index(1) - patch2BiasTmp] = ind1;
					index(1) = ind1;
					//printf_s("%d %d\n", index(1), ind1);
				}
				else if (index(2) == ind2) {
					mapping.get()[index(2) - patch2BiasTmp] = ind1;
					index(2) = ind1;
					//printf_s("%d %d\n", index(2), ind1);
				}

				//particles[ind1].velocity.setZero();
			}
		}

		//system("pause");
		patch2->ReGeneratePatchInfo(particles);
	}

	printf("seaming done\n");
}

template<typename T>
void Lyra::Cloth<T>::GlRenderSeaming(gph::Camera<T> &camera)
{
	glm::mat<4, 4, T> MVP(1);

	MVP = camera.GetMVPMatrix();
	parms.shader.use();
	parms.shader.setMat4("MVP", MVP);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glBindVertexArray(*VAO);

	glDrawArrays(GL_LINES, 0, seamingPointsVec.size());

	glBindVertexArray(0);
}

template<typename T>
void Lyra::Cloth<T>::Rendering(gph::Camera<T> &camera, bool lineMode)
{
	GenerateParticleNormal();

	for (auto& patch : patches) {
		patch->Rendering(particles, camera, lineMode);
	}

	if (!isSeaming && parms.enableSeaming)
		GlRenderSeaming(camera);
}

template<typename T>
void Lyra::Cloth<T>::GenerateParticleNormal()
{
	for (auto& patch : patches) {
		patch->GenerateParticleNormal(particles);
	}
}

template<typename T>
void Lyra::Cloth<T>::GlBindSeaming()
{
	VAO.reset(new uint32());
	VBO.reset(new uint32());

	glGenVertexArrays(1, VAO.get());
	glGenBuffers(1, VBO.get());

	glBindVertexArray(*VAO);

	//顶点位置属性
	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferData(GL_ARRAY_BUFFER, seamingPointsVec.size() * sizeof(vec3<T>), &(seamingPointsVec[0]), GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	if (typeid(T) == typeid(float))
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3<T>), (void*)0);
	else if (typeid(T) == typeid(double))
		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(vec3<T>), (void*)0);

	glBindVertexArray(0);
}

template<typename T>
void Lyra::Cloth<T>::GlBind()
{
	for (auto& patch : patches) {
		patch->GlBind();
	}

	if(parms.enableSeaming)
		GlBindSeaming();
}

template<typename T>
void Lyra::Cloth<T>::GlUpdateSeaming()
{
	glBindVertexArray(*VAO);

	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, seamingPointsVec.size() * sizeof(vec3<T>), &(seamingPointsVec[0]));

	glBindVertexArray(0);
}

template<typename T>
void Lyra::Cloth<T>::GlUpdate()
{
	for (auto& patch : patches) {
		patch->GlUpdate();
	}

	if (!isSeaming && parms.enableSeaming)
		GlUpdateSeaming();
}

