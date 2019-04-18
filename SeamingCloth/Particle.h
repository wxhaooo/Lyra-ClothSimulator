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
		//tʱ�̵�λ��
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
	//velocity verlet����
	/*T delta_t2 = delta_t * delta_t;
	vec3<T> tmp = position;
	position = position + velocity * delta_t + T(1) / 2 * preAccleration * delta_t2;
	prePosition = tmp;
	//�����accleration��tʱ�̵ģ����Խ���᲻��
	velocity = velocity + (preAccleration + acceleration) / T(2) * delta_t;
	preAccleration = acceleration;
	acceleration.setZero();*/

	//common verlet����
	T delta_t2 = delta_t * delta_t;
	vec3<T> tmp = position;
	position = position + position - prePosition + acceleration * delta_t2;
	//������Ƶ���tʱ�̵��ٶ�
	//velocity = (position - prePosition) / (T(2) * delta_t);
	prePosition = tmp;
	//������Ƶ���t+\delta tʱ�̵��ٶ�
	velocity = (position - prePosition)/delta_t;
	acceleration.setZero();
}