#pragma once
#include"Common.h"
#include"LyraFunction.h"
#include"Shader.h"
#include"LyraRandom.h"

#include<GraphicHelper\Camera.h>

#include<vector>

/*Wating to Test Functions*/

//bool IsIntersectWithBBox(BBox<T>& bBox);

namespace Lyra
{
	template<typename T> class BBox;
}

namespace gph = GraphicHelper;

namespace Lyra
{
	template<typename T>
	class BBox
	{
	public:
		BBox() = default;
		BBox(vec3<T> &center, vec3<T> &halfExtent, shader_sp<T> shader, bool draw = false);
		BBox(const vec3<T> &minCornerCoord, const vec3<T> &maxCornerCoord, shader_sp<T> shader, bool draw = false);
		BBox(const vec3<T>& p, shader_sp<T> shader, bool draw = false);

		//////////////////////////////Functions to build BVH//////////////////////////////////////
		void ExpandToInclude(const vec3<T>& p);
		void ExpandToInclude(const BBox<T>& bBox);
		uint32 MaxDimension() const;
		T SurfaceArea() const;
		/////////////////////////////Collision Detection//////////////////////////////////////////
		bool IsPointInBox(vec3<T>& point);
		bool IsIntersectWithBBox(BBox<T>& bBox);
		/////////////////////////////Visualize BBox///////////////////////////////////////////////
		void InitDraw(glm::vec<3, T>& color);
		void InitDraw();
		void GlBind();
		void GlUpdate();
		void GlDraw(gph::Camera<T>& camera);
		bool IsDraw() { return draw; }
		//Init Draw Info and Bind Vertex Buffer
		void GlInit(glm::vec<3, T>& color) { InitDraw(color); GlBind(); }

	private:
		///////////////////////////Init BBox//////////////////////////////////////////////////////
		void Init(const vec3<T>& minCornerCoord, const vec3<T>& maxCornerCoord, shader_sp<T> shader, bool draw = false);
		void Init(vec3<T>& center, vec3<T>& halfExtent, shader_sp<T> shader, bool draw = false);
	public:
		vec3<T> center;
		vec3<T> halfExtent;

		vec3<T> minCorner;
		vec3<T> maxCorner;

		//Shader<T> shader;
		shader_sp<T> shader;
	private:
		uint32_sp VAO;
		uint32_sp VBO;
		uint32_sp EBO;

		bool draw;
		glm::vec3 RGB;

		std::vector<vec3<T>> positions;
		std::vector<vec4<uint32>> indices;
	};

	template<typename T>
	using bBox_sp = std::shared_ptr<BBox<T>>;
	template<typename T>
	using bBox_up = std::unique_ptr<BBox<T>>;
	template<typename T>
	using bBox_pt = BBox<T> *;
}

template<typename T>
Lyra::BBox<T>::BBox(vec3<T> &center, vec3<T> &halfExtent, shader_sp<T> shader, bool draw)
{
	Init(center, halfExtent, shader, draw);
}

template<typename T>
Lyra::BBox<T>::BBox(const vec3<T>& minCornerCoord, const vec3<T>& maxCornerCoord, shader_sp<T> shader, bool draw)
{
	Init(minCornerCoord, maxCornerCoord, shader, draw);
}

template<typename T>
Lyra::BBox<T>::BBox(const vec3<T>& p, shader_sp<T> shader, bool draw)
{
	Init(p, p, shader, draw);
}

template<typename T>
uint32 Lyra::BBox<T>::MaxDimension() const
{
	uint32_t result = 0;
	if (halfExtent(1) > halfExtent(0)) {
		result = 1;
		if (halfExtent(2) > halfExtent(1)) result = 2;
	}
	else if (halfExtent(2) > halfExtent(0)) result = 2;

	return result; 
}

template<typename T>
T Lyra::BBox<T>::SurfaceArea() const
{
	vec3<T> extent = T(2) * halfExtent;
	return T(2)* (extent(0) * extent(2) + extent(0) * extent(1) + extent(1) * extent(2));
}

template<typename T>
void Lyra::BBox<T>::ExpandToInclude(const vec3<T>& p)
{
	minCorner = Min(minCorner, p);
	maxCorner = Max(maxCorner, p);

	halfExtent = (maxCorner - minCorner) * T(1) / 2;
	center = maxCorner - halfExtent;
}

template<typename T>
void Lyra::BBox<T>::ExpandToInclude(const BBox<T>& bBox)
{
	minCorner = Min(minCorner, bBox.minCorner);
	maxCorner = Max(maxCorner, bBox.maxCorner);

	halfExtent = (maxCorner - minCorner) * T(1) / 2;
	center = maxCorner - halfExtent;
}

template<typename T>
bool Lyra::BBox<T>::IsIntersectWithBBox(BBox<T>& bBox)
{
	if (maxCorner(0) < bBox.minCorner(0) || minCorner(0) > bBox.maxCorner(0)) return false;
	if (maxCorner(1) < bBox.minCorner(1) || minCorner(1) > bBox.maxCorner(1)) return false;
	if (maxCorner(2) < bBox.minCorner(2) || minCorner(2) > bBox.maxCorner(2)) return false;

	return true;
}

template<typename T>
bool Lyra::BBox<T>::IsPointInBox(vec3<T> &point)
{
	if (point[0] >= minCorner[0] && point[1] >= minCorner[1] && point[2] >= minCorner[2]
		&& point[0] <= maxCorner[0] && point[1] <= maxCorner[1] && point[2] <= maxCorner[2])
		return true;

	return false;
}

template<typename T>
void Lyra::BBox<T>::Init(const vec3<T>& minCornerCoord, const vec3<T>& maxCornerCoord, shader_sp<T> shader, bool draw)
{
	this->draw = draw;
	this->shader = shader;

	this->minCorner = minCornerCoord;
	this->maxCorner = maxCornerCoord;

	halfExtent = (maxCornerCoord - minCornerCoord) * T(1) / 2;

	center = maxCornerCoord - halfExtent;

	if (draw) {
		InitDraw();
		GlBind();
	}
}

template<typename T>
void Lyra::BBox<T>::Init(vec3<T> &center, vec3<T> &halfExtent, shader_sp<T> shader, bool draw)
{
	this->draw = draw;
	this->center = center;
	this->halfExtent = halfExtent;

	minCorner = center - halfExtent;
	maxCorner = center + halfExtent;

	this->shader = shader;

	if (draw) {	
		InitDraw();
		GlBind();
	}
}

template<typename T>
void Lyra::BBox<T>::InitDraw()
{
	glm::vec<3, T> color = ColorGenerator<T>();
	InitDraw(color);
}

template<typename T>
void Lyra::BBox<T>::InitDraw(glm::vec<3, T>& color)
{
	vec3<T> tmp;

	float deltaX = halfExtent(0);
	float deltaY = halfExtent(1);
	float deltaZ = halfExtent(2);

	//0(-,-,+)
	tmp = center + vec3<T>(-deltaX, -deltaY, deltaZ);
	positions.push_back(tmp);
	//1(+,-,+)
	tmp = center + vec3<T>(deltaX, -deltaY, deltaZ);
	positions.push_back(tmp);
	//2(+,+,+)
	tmp = center + vec3<T>(deltaX, deltaY, deltaZ);
	positions.push_back(tmp);
	//3(-,+,+)
	tmp = center + vec3<T>(-deltaX, deltaY, deltaZ);
	positions.push_back(tmp);
	//4(-,-,-)
	tmp = center + vec3<T>(-deltaX, -deltaY, -deltaZ);
	positions.push_back(tmp);
	//5(+,-,-)
	tmp = center + vec3<T>(deltaX, -deltaY, -deltaZ);
	positions.push_back(tmp);
	//6(+,+,-)
	tmp = center + vec3<T>(deltaX, deltaY, -deltaZ);
	positions.push_back(tmp);
	//7(-,+,-)
	tmp = center + vec3<T>(-deltaX, deltaY, -deltaZ);
	positions.push_back(tmp);

	indices.push_back(vec4<uint32>(0, 1, 2, 3));
	indices.push_back(vec4<uint32>(4, 5, 6, 7));
	indices.push_back(vec4<uint32>(3, 2, 6, 7));
	indices.push_back(vec4<uint32>(0, 1, 5, 4));

	RGB = color;
}

template<typename T>
void Lyra::BBox<T>::GlBind()
{
	VAO.reset(new uint32());
	VBO.reset(new uint32());
	EBO.reset(new uint32());

	glGenVertexArrays(1, VAO.get());
	glGenBuffers(1, VBO.get());
	glGenBuffers(1, EBO.get());

	glBindVertexArray(*VAO);

	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(vec3<T>), &(positions[0]), GL_STREAM_DRAW);
	glEnableVertexAttribArray(0);
	if (typeid(T) == typeid(float))
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3<T>), (void*)0);
	else if (typeid(T) == typeid(double))
		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(vec3<T>), (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(vec4<uint32>), &(indices[0]), GL_STREAM_DRAW);

	glBindVertexArray(0);
}

template<typename T>
void Lyra::BBox<T>::GlUpdate()
{
	glBindVertexArray(*VAO);

	glBindBuffer(GL_ARRAY_BUFFER, *VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, positions.size() * sizeof(vec3<T>), &(positions[0]));

	glBindVertexArray(0);
}

template<typename T>
void Lyra::BBox<T>::GlDraw(gph::Camera<T> &camera)
{
	glm::mat4 MVP = camera.GetMVPMatrix();
	
	shader->use();
	shader->setMat4("MVP", MVP);
	shader->setVec3("RGB", RGB);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glBindVertexArray(*VAO);

	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(sizeof(vec4<T>)));
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(2 * sizeof(vec4<T>)));
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(3 * sizeof(vec4<T>)));

	glBindVertexArray(0);
}