#include "pch.h"

#include"../SeamingCloth/BBox.h"
#include"../SeamingCloth/BBoxTriangle.h"

#include<random>
#include<ctime>

#include<glad/glad.h>
#include<glad/glad.c>

using namespace Lyra;
using namespace std;

std::string vertexShaderPath = "../SeamingCloth/shader/bboxVertexShader.vs";
std::string fragmentShaderPath = "../SeamingCloth/shader/bboxFragmentShader.fs";

constexpr uint32 MAXN = 10;
constexpr float MAXF = 10e6;

default_random_engine e(time(0));
uniform_int_distribution<uint32> u(0, MAXN);
uniform_real_distribution<float> v(0, MAXF);


TEST(BBoxClass, FoundationData)
{
	int screenWidth = 1024;
	int screenHeight = 1024;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(screenWidth, screenHeight, "Cloth Simulation", NULL, NULL);

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

	const vec3<float> minCornerCoord1(7, 6, 5);
	const vec3<float> maxCornerCoord1(15, 15, 15);
	const vec3<float> minCornerCoord2(6, 6, 6);
	const vec3<float> maxCornerCoord2(16, 16, 16);

	Shader<float> shader(vertexShaderPath.c_str(), fragmentShaderPath.c_str());
	BBox<float> bbox1(minCornerCoord1, maxCornerCoord1, shader, false);
	BBox<float> bbox2(minCornerCoord2, maxCornerCoord2, shader, true);

	
	glm::vec3 x1(u(e), u(e), u(e));
	glm::vec3 x2(u(e), u(e), u(e));
	glm::vec3 x3(u(e), u(e), u(e));

	VertexPointer<float> v1, v2, v3;
	v1 = std::make_shared<Vertex<float>>();
	v2 = std::make_shared<Vertex<float>>();
	v3 = std::make_shared<Vertex<float>>();

	v1->position = x1;
	v2->position = x2;
	v3->position = x3;

	BBoxObjTriangle<float> objTriangle(v1, v2, v3);

	//BBoxTriangle<float> boxTriangle;

	//objTriangle.GetBBox(shader, true);

	/*std::cout << objTriangle.V2()->position.x << " " << objTriangle.V2()->position.y << " "<<
		objTriangle.V2()->position.z << "\n\n";*/

	BBox<float> bBox = objTriangle.GetBBox(shader, true);
	
	std::cout << x1.x << " " << x1.y << " " << x1.z << "\n\n";
	std::cout << x2.x << " " << x2.y << " " << x2.z << "\n\n";
	std::cout << x3.x << " " << x3.y << " " << x3.z << "\n\n";

	std::cout << bBox.center << "\n\n";
	std::cout << bBox.halfExtent << "\n\n";
	std::cout << bBox.minCorner << "\n\n";
	std::cout << bBox.maxCorner << "\n\n";


	//EXPECT_TRUE(bbox.IsDraw());
}