#pragma once
#include"Common.h"
#include"LyraFunction.h"
#include"Shader.h"

#include<GraphicHelper\Camera.h>

#include<vector>

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
		BBox(vec3<T> &center, vec3<T> &halfExtent, Shader<T> &shader, bool draw = false);

		bool PointInBox(vec3<T> &point);
		void Init(vec3<T> &center, vec3<T> &halfExtent,Shader<T> &shader, bool draw = false);
		void InitDraw();
		void GlBind();
		void GlUpdate();
		void GlDraw(gph::Camera<T> &camera);
		bool IsDraw() { return draw; }
	public:
		vec3<T> center;
		vec3<T> halfExtent;

		vec3<T> minCorner;
		vec3<T> maxCorner;

		Shader<T> shader;
	private:
		uint32_sp VAO;
		uint32_sp VBO;
		uint32_sp EBO;

		bool draw;

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
Lyra::BBox<T>::BBox(vec3<T> &center, vec3<T> &halfExtent, Shader<T> &shader, bool draw)
{
	Init(center, halfExtent, shader, draw);
}

template<typename T>
bool Lyra::BBox<T>::PointInBox(vec3<T> &point)
{
	if (point[0] >= minCorner[0] && point[1] >= minCorner[1] && point[2] >= minCorner[2]
		&& point[0] <= maxCorner[0] && point[1] <= maxCorner[1] && point[2] <= maxCorner[2])
		return true;

	return false;
}

template<typename T>
void Lyra::BBox<T>::Init(vec3<T> &center, vec3<T> &halfExtent, Shader<T> &shader, bool draw)
{
	this->center = center;
	this->halfExtent = halfExtent;
	this->draw = draw;

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

	shader.use();
	shader.setMat4("MVP", MVP);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glBindVertexArray(*VAO);

	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(sizeof(vec4<T>)));
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(2 * sizeof(vec4<T>)));
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (GLvoid*)(3 * sizeof(vec4<T>)));

	glBindVertexArray(0);
}