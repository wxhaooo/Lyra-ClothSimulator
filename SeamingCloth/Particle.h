#pragma once

#include"Common.h"
#include"LyraFunction.h"
#include<memory>

namespace Lyra
{
	template<typename T> struct Particle;
	enum VelocityUpdate { PSEUDO_VELOCITY, MIDDLE_VELOCITY, CURRENT_VELOCITY, PRE_VELOCITY };
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
		T massInv;
		bool movable;
		bool isCollide;

		vec2<T> planeCoordinate;
		//tʱ�̵�λ��
		vec3<T> position;
		//t-\delta tʱ�̵�λ��
		vec3<T> prePosition;
		//t+\delta tʱ�̵�λ��
		vec3<T> pseudoPosition;
		//tʱ�̵��ٶ�
		vec3<T> velocity;
		//��ʱ��t+\litteʱ�̵��ٶ�
		vec3<T> velocityTmp;
		//t-\delta tʱ�̵��ٶ�
		vec3<T> preVelocity;
		//t+0.5 * \delta tʱ�̵��ٶ�
		//����ٶ���implicit�����ге��������ã���һ�����÷�������û���ˣ����Ը���
		vec3<T> middleVelocity;
		//t+\delta tʱ�̵��ٶ�
		vec3<T> pseudoVelocity;
		//tʱ�̵ļ��ٶ�
		vec3<T> acceleration;
		//t-\delta tʱ�̵ļ��ٶ�
		vec3<T> preAccleration;
		//t+\delta tʱ�̵ļ��ٶ�
		vec3<T> pseudoAccleration;
		
		Particle() = default;
		~Particle() = default;

		Particle(vec2<T> &planeCoord, vec3<T> &wordPos, T mass = 1.0f, bool mv = true);

		void ApplyGravity(vec3<T> acc);

		void AdvanceStep();

		void ApplyForce(vec3<T> force);

		void UpdatePseudoPosition(T delta_t);

		void EstimateMiddleVelocity(T delta_t);

		void UpdateMiddleVelocityWithVelocityVerlet(T delta_t);

		void UpdatePseudoPositionWithVelocityVerlet(T delta_t);

		void UpdatePseudoVelocityWithVelocityVerlet(T delta_t);

		void UpdatePesudoVelocity(T delta_t);

		void UpdateMiddleVelocity(T delta_t);

		void UpdatePesudoPositionImplicit(T delta_t);

		void DebugUpdatePosition(T delta_t);

		//common verlet and estimate velocity
		void UpdatePosition(T delta_t);

		//velocity verlet
		void UpdatePositionWithVelocityVerlet(T delta_t);

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
	pseudoPosition.setZero();

	velocity.setZero();
	velocityTmp.setZero();
	preVelocity.setZero();
	middleVelocity.setZero();
	pseudoVelocity.setZero();

	acceleration.setZero();
	preAccleration.setZero();
	pseudoAccleration.setZero();
	
	movable = mv;
	isCollide = false;

	this->mass = mass;
}

template<typename T>
void Lyra::Particle<T>::AdvanceStep()
{
	//������û����ײ��Ӧ����һʱ�̵���Ϣ����ǰ����µ�pseudoXXX����
	//����ֻ��Ҫ��pseudoXXX���µ�XXX�Ϳ�����

	/*std::cout << preVelocity(1) << "\n";
	std::cout << velocity(1) << "\n\n";*/

	prePosition = position;
	position = pseudoPosition;
	pseudoPosition.setZero();

	preVelocity = velocity;
	velocity = pseudoVelocity;
	velocityTmp = pseudoVelocity;

	middleVelocity.setZero();
	pseudoVelocity.setZero();

	preAccleration = acceleration;
	acceleration = pseudoAccleration;
	pseudoAccleration.setZero();
	//IMEX��ʱ����Ҫ��accleration��0
	acceleration.setZero();
}

template<typename T>
void Lyra::Particle<T>::ApplyGravity(vec3<T> acc)
{
	if (!IsZero(massInv)) {
		acceleration += acc;
	}
	//pseudoAccleration += acc;
}

template<typename T>
void Lyra::Particle<T>::ApplyForce(vec3<T> force)
{
	acceleration += (massInv * force);
	//pseudoAccleration += force / mass;
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
	prePosition = tmp;
	//������Ƶ���t+\delta tʱ�̵��ٶ�
	velocity = (position - prePosition)/delta_t;
	//velocity = (position - prePosition);
	acceleration.setZero();
}

template<typename T>
void Lyra::Particle<T>::EstimateMiddleVelocity(T delta_t)
{
	middleVelocity = T(1) / delta_t * (pseudoPosition - position);
}

template<typename T>
void Lyra::Particle<T>::UpdatePesudoVelocity(T delta_t) {
	//std::cout << (0.5 * delta_t * acceleration).norm() << "\n";
	pseudoVelocity = middleVelocity + 0.5 * delta_t * acceleration;
}

template<typename T>
void Lyra::Particle<T>::UpdateMiddleVelocity(T delta_t)
{
	middleVelocity = velocity + 0.5 * delta_t * acceleration;
}

template<typename T>
void Lyra::Particle<T>::UpdatePesudoPositionImplicit(T delta_t)
{
	pseudoPosition = position + delta_t * middleVelocity;
}

template<typename T>
void Lyra::Particle<T>::UpdateMiddleVelocityWithVelocityVerlet(T delta_t)
{
	middleVelocity = velocity + 0.5 * acceleration * delta_t;
}

template<typename T>
void Lyra::Particle<T>::UpdatePseudoPositionWithVelocityVerlet(T delta_t)
{
	//������ײ�����
	//pseudoPosition = position + velocity * delta_t + 0.5 * acceleration * delta_t * delta_t;
	pseudoPosition = position + middleVelocity * delta_t;
}

template<typename T>
void Lyra::Particle<T>::UpdatePseudoVelocityWithVelocityVerlet(T delta_t)
{
	//���ڼ�����һʱ�̵��ٶ�
	//pseudoVelocity = velocity + 0.5 * (acceleration + pseudoAccleration) * delta_t;
	pseudoVelocity = middleVelocity + 0.5 * pseudoAccleration * delta_t;
}

template<typename T>
void Lyra::Particle<T>::UpdatePositionWithVelocityVerlet(T delta_t)
{
	//ֻ������һ֡�µ����ݣ�������force
	//���������ײ��Ӧ�����¼�����һʱ�̵�position��velocity���Ҹ���
	pseudoPosition = position + velocity * delta_t + 0.5 * acceleration * delta_t * delta_t;
	pseudoVelocity = velocity + 0.5 * (acceleration + pseudoAccleration) * delta_t;

	prePosition = position;
	position = pseudoPosition;
	pseudoPosition.setZero();

	preVelocity = velocity;
	velocity = pseudoVelocity;
	pseudoVelocity.setZero();

	preAccleration = acceleration;
	acceleration = pseudoAccleration;
	pseudoAccleration.setZero();
}

template<typename T>
void Lyra::Particle<T>::DebugUpdatePosition(T delta_t)
{
	
}

template<typename T>
void Lyra::Particle<T>::UpdatePseudoPosition(T delta_t)
{
	//������һʱ�̵�λ�ú��ٶ�������ײ��Ӧ
	pseudoPosition.setZero();
	pseudoVelocity.setZero();

	T delta_t2 = delta_t * delta_t;
	pseudoPosition = position + position - prePosition + acceleration * delta_t2;
	//pseudoPosition = vec3<T>(position(0), -1000.f, position(2));
	//std::cout << pseudoPosition - position << "\n";
	pseudoVelocity = (pseudoPosition - position) / delta_t;
}