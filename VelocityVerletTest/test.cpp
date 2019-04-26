#include "pch.h"

#include"../SeamingCloth/Particle.h"

using namespace Lyra;
using namespace std;

using type = float;

// velocity verlet method test example
//���ڲ���velocity verlet ��delta_t���׶��ٺ���
//condition:
//force == const
//initial position/velocity == vec3(ZERO,ZERO,ZERO)
TEST(ParticleTest, VelocityVerlet)
{
	type totalTime = 15.f;
	type delta_t = 0.004f;
	type mass = 0.3f;
	vec2<type> uvPos(0.f, 0.f);
	vec3<type> pos(0.f, 0.f, 0.f);
	vec3<type> force(1.f, 0.f, 0.f);

	Particle<type> p(uvPos, pos, mass);

	uint32 count = ceil(totalTime / delta_t);

	std::cout << count << "\n\n";

	for (uint32 i = 0; i < count; i++) {
		p.UpdateMiddleVelocityWithVelocityVerlet(delta_t);
		p.UpdatePseudoPositionWithVelocityVerlet(delta_t);
		p.ApplyForce(force);
		p.UpdatePseudoVelocityWithVelocityVerlet(delta_t);
		p.AdvanceStep();
	}

	vec3<type> truePos = 0.5 * force / mass * totalTime * totalTime;
	vec3<type> trueVel = force / mass * totalTime;

	//����ʵ�ٶ�/λ�õ����
	std::cout << p.position - truePos << "\n\n";
	std::cout << p.velocity - trueVel << "\n\n";
}