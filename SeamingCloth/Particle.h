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

		vec2<T> planeCoordinate;
		//t时刻的位置
		vec3<T> position;
		vec3<T> prePosition;
		vec3<T> pseudoPosition;
		vec3<T> velocity;
		vec3<T> acceleration;
		vec3<T> preAccleration;

		Particle() = default;
		~Particle() = default;

		Particle(vec2<T> &planeCoord, vec3<T> &wordPos, T mass = 1.0f, bool mv = true);

		void ApplyForce(vec3<T> force);

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
	velocity = vec3<T>(0., 0., 0.);
	acceleration = vec3<T>(0., 0., 0.);
	preAccleration = vec3<T>(0., 0., 0.);

	movable = mv;

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
	acceleration.setZero();
}