#include "pch.h"

#include"../SeamingCloth/Particle.h"
#include"../SeamingCloth/BBox.h"
#include"../SeamingCloth/Shader.h"

#include<glad\glad.h>
#include<glad\glad.c>

#include<GraphicHelper\Camera.h>

#include<string>

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

using namespace Lyra;
using namespace std;
using namespace GraphicHelper;


TEST(particleClass, UpdataPosition)
{
	particle_up<double> p = std::make_unique<Particle<double>>();
	vec3<double> tmpAcc(2.f, 3.f, 4.f);
	double delta_t = 0.001f;
	EXPECT_EQ(vec3<double>(0.f, 0.f, 0.f), p->acceleration);
	p->CorrectAcceleration(tmpAcc);
	EXPECT_EQ(tmpAcc, p->acceleration);
	vec3<double> tmp = p->acceleration * delta_t * delta_t;
	p->UpdatePosition(delta_t);
	EXPECT_EQ(tmp, p->position) << tmp << "\n\n" << p->position << "\n\n";
}

TEST(BBoxClass, RunNormal)
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1024, 1024, "Cloth Simulation", NULL, NULL);

	if (window == NULL) {
		std::cerr << "creating windows failed\n";
		glfwTerminate();
		system("pause");
	}

	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		system("pause");
	}

	std::string bboxVertexShaderPath = "./shader/bboxVertexShader.vs";
	std::string bboxFragmentShaderPath = "./shader/bboxFragmentShader.fs";

	Shader<float> shader(bboxVertexShaderPath.c_str(), bboxFragmentShaderPath.c_str());
	bBox_sp<float> bBox = std::make_shared<BBox<float>>();

	vec3<float> center(1., 2., 3.);
	vec3<float> halfExent(4, 5, 6);
	bBox->Init(center, halfExent, shader, true);

	EXPECT_EQ(bBox->center, center) << bBox->center << "\n\n" << center << "\n\n";
	EXPECT_EQ(bBox->halfExtent, halfExent) << bBox->halfExtent << "\n\n" << halfExent << "\n\n";
}