#pragma once

#include"Common.h"
#include<memory>

namespace Lyra
{
	template<typename T> struct Particle;
}

namespace Lyra
{
	
}

namespace Lyra
{
	template<typename T>
	struct Particle
	{
		T mass;
		bool movable;
		bool isCollide;

		vec2<T> planeCoordinate;
		//t时刻的位置
		vec3<T> position;
		//t-\delta t时刻的位置
		vec3<T> prePosition;
		//t+\delta t时刻的位置
		vec3<T> pseudoPosition;
		//t时刻的速度
		vec3<T> velocity;
		//t-\delta t时刻的速度
		vec3<T> preVelocity;
		//t+\delta t时刻的速度
		vec3<T> pseudoVelocity;
		//t时刻的加速度
		vec3<T> acceleration;
		//t-\delta t时刻的加速度
		vec3<T> preAccleration;
		//t+\delta t时刻的加速度
		vec3<T> pseudoAccleration;
		

		Particle() = default;
		~Particle() = default;

		Particle(vec2<T> &planeCoord, vec3<T> &wordPos, T mass = 1.0f, bool mv = true);

		void ApplyForce(vec3<T> force);

		void UpdatePseudoPosition(T delta_t);

		void DebugUpdatePosition(T delta_t);

		void UpdatePosition(T delta_t);

		void ClearVelocity() { velocity.setZero(); }

		void CorrectAcceleration(vec3<T> &deltaAcc) { acceleration += deltaAcc; }

	};

	template<typename T>
	using particle_sp = std::shared_ptr<Particle<T>>;
	template<typename T>
	using particle_up = std::unique_ptr<Particle<T>>;
	template<typename T>
	using particle_pt = Particle<T> *;
}

template<typename T>
Lyra::Particle<T>::Particle(vec2<T> &planeCoord, vec3<T> &worldPos, T mass, bool mv)
{
	planeCoordinate = planeCoord;

	position = worldPos;
	prePosition = worldPos;
	pseudoPosition = worldPos;

	velocity.setZero();
	preVelocity.setZero();
	pseudoVelocity.setZero();

	acceleration.setZero();
	preAccleration.setZero();
	pseudoAccleration.setZero();
	
	movable = mv;
	isCollide = false;

	//collisionInfo.isCollided = false;
	this->mass = mass;
}

template<typename T>
void Lyra::Particle<T>::ApplyForce(vec3<T> force)
{
	acceleration += force / mass;
}

template<typename T>
void Lyra::Particle<T>::UpdatePosition(T delta_t)
{
	//velocity verlet积分
	/*T delta_t2 = delta_t * delta_t;
	vec3<T> tmp = position;
	position = position + velocity * delta_t + T(1) / 2 * preAccleration * delta_t2;
	prePosition = tmp;
	//这里的accleration是t时刻的，所以结果会不对
	velocity = velocity + (preAccleration + acceleration) / T(2) * delta_t;
	preAccleration = acceleration;
	acceleration.setZero();*/

	//common verlet积分
	T delta_t2 = delta_t * delta_t;
	vec3<T> tmp = position;
	position = position + position - prePosition + acceleration * delta_t2;
	//这里估计的是t时刻的速度
	//velocity = (position - prePosition) / (T(2) * delta_t);
	prePosition = tmp;
	//这里估计的是t+\delta t时刻的速度
	velocity = (position - prePosition)/delta_t;
	//velocity = (position - prePosition);
	acceleration.setZero();
}

template<typename T>
void Lyra::Particle<T>::DebugUpdatePosition(T delta_t)
{
	
}

template<typename T>
void Lyra::Particle<T>::UpdatePseudoPosition(T delta_t)
{
	//估计下一时刻的位置和速度用于碰撞响应
	pseudoPosition.setZero();
	pseudoVelocity.setZero();

	T delta_t2 = delta_t * delta_t;
	pseudoPosition = position + position - prePosition + acceleration * delta_t2;
	//pseudoPosition = vec3<T>(position(0), -1000.f, position(2));
	//std::cout << pseudoPosition - position << "\n";
	pseudoVelocity = (pseudoPosition - position) / delta_t;
}