#include"Cloth.h"
#include"ClothPatch.h"
#include"ParmSwitch.h"
#include"Shader.h"
#include"BBox.h"
#include"UniformBVH.h"

#include"ObjectBVH.h"
#include"ClothBVH.h"

#include<iostream>
#include<cassert>

#include<GraphicHelper\Camera.h>

#include<glad\glad.h>
#include<GLFW\glfw3.h>
#include<glad\glad.c>

/*当前存在的问题：
1. 无法匹配真实参数（模型问题）
2. 对非rest state的布料没法缝合（积分方法问题）
3. damping引起的偶尔崩溃（积分方法问题）
*/

//live share test

using seamingPair = Lyra::SeamingPair;
namespace gph = GraphicHelper;

void GlfwReshapeCallbackFunc(GLFWwindow* window, int width, int height);
void GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods);
void GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos);
void GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods);
void GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

std::string modelVertexShaderPath = "./shader/modelVertexShader.vs";
std::string modelFragmentShaderPath = "./shader/modelFragmentShader.fs";
std::string model_1_FragmentShaderPath = "./shader/model_1_FragmentShader.fs";
std::string seamingFragmentShaderPath = "./shader/seamingFragmentShader.fs";
std::string bboxVertexShader = "./shader/bboxVertexShader.vs";
std::string bboxFragmentShader = "./shader/bboxFragmentShader.fs";

std::string bodyVertexShaderPath = "./shader/bodyVertexShader.vs";
std::string bodyGeometryShaderPath = "./shader/bodyGeometryShader.gs";
std::string bodyFragmentShaderPath = "./shader/bodyFragmentShader.fs";

std::string bodyPath = "./model/rock/rock.obj";
//std::string bodyPath = "./patch/UvObj_Origin_Back_15.obj";
gph::Camera<float> camera;

int main()
{
	int screenWidth = 1024;
	int screenHeight = 1024;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow *window = glfwCreateWindow(screenWidth, screenHeight, "Cloth Simulation", nullptr, nullptr);

	if (window == nullptr) {
		std::cerr << "creating windows failed\n";
		glfwTerminate();
		system("pause");
		return false;
	}

	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		system("pause");
		return false;
	}

	glfwSetFramebufferSizeCallback(window, GlfwReshapeCallbackFunc);
	glfwSetKeyCallback(window, GlfwKeyBoardCallbackFunc);
	glfwSetCursorPosCallback(window, GlfwCursorPosCallbackFunc);
	glfwSetMouseButtonCallback(window, GlfwMouseStateCallbackFunc);
	glfwSetScrollCallback(window, GlfwScrollCallback);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	glm::mat4 modelMat(1);
	glm::mat4 viewMat(1);
	glm::mat4 projectionMat(1);
	glm::mat4 MVP(1);

	modelMat = glm::translate(modelMat, glm::vec3(-0.f, -0.f, 0.f));
	modelMat = glm::rotate(modelMat, Angle2Radian(-30), glm::vec3(0.f, 1.f, 0.f));

	Lyra::Shader<float> sharedShader =  Lyra::Shader<float>(modelVertexShaderPath.c_str(), modelFragmentShaderPath.c_str());

	gph::CameraSetting<float> setting;

	{
		//glm::rotate(modelMat, 90.f/180*PI, glm::vec3(1.f, 0.f, 0.f));
		//如果相机位置正好和上向量(0.f,1.f,0.f)平行，那么就会出问题
		glm::vec3 cameraPosition(-0.f, 10.0f, 150.0f);
		glm::vec3 cameraTarget(0.f, 0.f, 0.f);
		setting.cameraMode = gph::CameraMode::CAMERA_PROJECTION;
		setting.cameraPosition = cameraPosition;
		setting.cameraTarget = cameraTarget;
		setting.scrollSensitivity = 0.3f;
		setting.keyboardSensitivity = 0.3f;
		setting.mouseSensitivity = 0.05f;
		setting.maxPitchRate = 0.004f;
		setting.maxHeadingRate = 0.004f;
		setting.nearClipPlane = 1.f;
		setting.farClipPlane = 1500.f;
		setting.viewportWidth = screenWidth;
		setting.viewportHeight = screenHeight;
		setting.screenWidth = screenWidth;
		setting.screenHeight = screenHeight;
		setting.FOV = 45.f;
	}

	//////////////////////////////body setting///////////////////////////////////////////////////
	gph::Shader bodyShader(bodyVertexShaderPath.c_str(), bodyFragmentShaderPath.c_str());
	Lyra::Shader<float> bvhShader = Lyra::Shader<float>(bboxVertexShader.c_str(), bboxFragmentShader.c_str());

	gph::ModelPointer<float> body = std::make_shared<gph::Model<float>>();
	body->LoadModel(bodyPath);

	///////////////////////////////BVH Build////////////////////////////////////////////////////
	/*Lyra::objectBvh_up<float> objBVH= std::make_unique<Lyra::ObjectBVH<float>>();
	objBVH->Build(body, 1, bvhShader, false);*/
	/*Lyra::uniformBvh_up<float> bvh = std::make_unique<Lyra::UniformBVH<float>>();

	Lyra::vec3<float> center(0.f, -65.f, 5.f);
	Lyra::vec3<float> halfExtend(32.f, 90.f, 25.f);
	Lyra::vec3<float> cellSize(10, 10, 10);

	bvh->SetExternalBBox(center, halfExtend, bvhShader, drawSubBBox);
	bvh->BuildBBoxVH(body, cellSize, bvhShader);*/
	
	///////////////////////////////cloth setting///////////////////////////////////////////////////

	Lyra::cloth_up<float> cloth = std::make_unique<Lyra::Cloth<float>>();

	Lyra::clothPatch_sp<float> clothPatch0 = std::make_shared<Lyra::ClothPatch<float>>();
	Lyra::clothPatch_sp<float> clothPatch1 = std::make_shared<Lyra::ClothPatch<float>>();

	Lyra::ClothPatchParms<float> parms0;
	Lyra::ClothPatchParms<float> parms1;
	Lyra::SeamingInfo<float> seamingInfo1;
	Lyra::ClothParms<float> clothParms;

	std::vector<uint32> stickPoints0 = { /*7,8,9*/ };
	std::vector<uint32> stickPoints1 = { /*4,5,6*/ };
	//patch0 Parms
	{
		parms0.path = "./patch/";
		//parms0.name = "UvObj_texture_front.obj";
		parms0.name = "UvObj_Origin.obj";
		//parms0.name = "Square_Front.obj";
		parms0.shader = Lyra::Shader<float>(modelVertexShaderPath.c_str(), modelFragmentShaderPath.c_str());
		parms0.patchMode = Lyra::ClothPatchMode::LYRA_CLOTH_PATCH_SEAMING;
		//parms0.initState = Lyra::ClothPatchInitState::LYRA_CLOTH_PATCH_PLANE;
		parms0.initState = Lyra::ClothPatchInitState::LYRA_CLOTH_PATCH_NON_PLANE;
		parms0.stickPoints.reset(&(stickPoints0));
		///scale小了会导致plane force变小，结果系统更稳定，需要确定一个合适的scale
		///这个scale理论上是非线性的，现在用线性的替代，必然导致force计算出现问题[19-04-11]
		parms0.scale = 150.f;
		parms0.density = 1.0f;

		///刚性强会使solver崩掉，刚性不够则没法缝合，
		///@(todo_1)为什么刚性弱无法缝合？？？
		///@(do_1)问题解决，可以缝合，积分方法问题
		parms0.stretchingFactor = 10000.f;
		parms0.shearingFactor = 500.f;
		parms0.bendingFactor = 60.237942e-6;
		///damping stretch引起的问题，可能是solver不能解刚性过大的系统
		parms0.dampingStretchFactor = 0.1f;
		parms0.dampingShearFactor = 0.1f;
		parms0.dampingBendingFactor = 0.0001f;
		parms0.stretchScaleUDir = 1.f;
		parms0.stretchScaleVDir = 1.f;

		/*parms0.stretchingFactor = 5000.f;
		parms0.shearingFactor = 500.f;
		parms0.bendingFactor = 0.01f;
		parms0.dampingStretchFactor = 1.f / 25;
		parms0.dampingShearFactor = 1.f / 50;
		parms0.dampingBendingFactor = 0.0001f;
		parms0.stretchScaleUDir = 1.f;
		parms0.stretchScaleVDir = 1.f;*/

		parms0.planeForceSwitch.enableStretchForce = true;
		parms0.planeForceSwitch.enableShearForce = true;
		parms0.planeForceSwitch.enableDampingStretchForce = true;
		parms0.planeForceSwitch.enableDampingShearForce = true;

		parms0.spaceForceSwitch.enableBendingForce = true;
		parms0.spaceForceSwitch.enableDampingBendingForce = true;

		parms0.enableCollisionDetect = true;
	}
	//patch1 Parms
	{
		parms1.path = "./patch/";
		//parms1.name = "UvObj_texture_back.obj";
		parms1.name = "UvObj_Origin_Back_15.obj";
		//parms1.name = "Square_Back.obj";
		parms1.shader = Lyra::Shader<float>(modelVertexShaderPath.c_str(), model_1_FragmentShaderPath.c_str());
		parms1.patchMode = Lyra::ClothPatchMode::LYRA_CLOTH_PATCH_SEAMING;
		//parms1.initState = Lyra::ClothPatchInitState::LYRA_CLOTH_PATCH_PLANE;
		parms1.initState = Lyra::ClothPatchInitState::LYRA_CLOTH_PATCH_NON_PLANE;
		parms1.stickPoints.reset(&(stickPoints1));
		parms1.scale = 150.f;
		parms1.density = 1.0f;

		///@(todo_2)增强在高刚性参数情况下solver的稳定性
		///1.IMEX？？？
		///2.Implicit Solver???
		///@(do_2)问题解决：积分方法的问题04-11-19
		parms1.stretchingFactor = 10000.f;
		parms1.shearingFactor = 500.f;
		parms1.bendingFactor = 60.237942e-6;
		parms1.dampingStretchFactor = 0.1f;
		parms1.dampingShearFactor = 0.1f;
		parms1.dampingBendingFactor = 0.0001f;
		parms1.stretchScaleUDir = 1.f;
		parms1.stretchScaleVDir = 1.f;

		/*parms1.stretchingFactor = 5000.f;
		parms1.shearingFactor = 500.f;
		parms1.bendingFactor = 0.01f;
		parms1.dampingStretchFactor = 1.f / 25;
		parms1.dampingShearFactor = 1.f / 50;
		parms1.dampingBendingFactor = 0.0001f;
		parms1.stretchScaleUDir = 1.f;
		parms1.stretchScaleVDir = 1.f;*/

		parms1.planeForceSwitch.enableStretchForce = true;
		parms1.planeForceSwitch.enableShearForce = true;
		parms1.planeForceSwitch.enableDampingStretchForce = true;
		parms1.planeForceSwitch.enableDampingShearForce = true;

		parms1.spaceForceSwitch.enableBendingForce = true;
		parms1.spaceForceSwitch.enableDampingBendingForce = true;

		parms1.enableCollisionDetect = true;
	}
	//seaming Info
	{
		//使用houdini来确定缝合的时候务必保证mesh里面没有退化的三角形和孤立点，防止被assimp清理掉
		seamingInfo1.patch1 = clothPatch0;
		seamingInfo1.patch2 = clothPatch1;
		///plane clean scanned mesh
		{
		//	seamingInfo1.seamingPairs = {
		///*seamingPair(2555,222),seamingPair(114,220),seamingPair(116,221),seamingPair(2364,529),
		//seamingPair(2368,2470),seamingPair(2369,2472), 
		//	seamingPair(169,161),seamingPair(168,162),seamingPair(2374,2364),seamingPair(2568,2366),
		//seamingPair(2379,2370),seamingPair(354,488),*/ };
			/*seamingPair(252,217),seamingPair(120,214),seamingPair(118,216),seamingPair(251,100),
			seamingPair(107,101),seamingPair(105,211),seamingPair(247,210),seamingPair(245,94),
			seamingPair(243,208),seamingPair(244,209),};*/
		}
		///2x2 plane mesh
		{
		//seamingInfo1.seamingPairs = {
		/////2x2 不同mesh结构（正常）
		///*seamingPair(268,134),seamingPair(48,235),seamingPair(7,27),seamingPair(8,234),
		//seamingPair(267,231),seamingPair(265,229),seamingPair(266,228),seamingPair(264,232),
		//seamingPair(263,227),seamingPair(262,218),

		//seamingPair(252,217),seamingPair(120,214),seamingPair(118,216),seamingPair(251,100),
		//seamingPair(107,101),seamingPair(105,211),seamingPair(247,210),seamingPair(245,94),
		//seamingPair(243,208),seamingPair(244,209),*/
		//	///2x2集中点
		///*seamingPair(268,252),seamingPair(48,252),seamingPair(7,252),seamingPair(8,252),
		//seamingPair(267,252),seamingPair(265,252),seamingPair(266,252),seamingPair(264,252),
		//seamingPair(263,252),seamingPair(262,252),

		//seamingPair(252,252),seamingPair(120,252),seamingPair(118,252),seamingPair(251,252),
		//seamingPair(107,252),seamingPair(105,252),seamingPair(247,252),seamingPair(245,252),
		//seamingPair(243,252),*/
		/////2x2错位上边
		///*seamingPair(268,48),seamingPair(48,7),seamingPair(7,8),seamingPair(8,267),
		//seamingPair(267,265),seamingPair(265,266),seamingPair(266,264),seamingPair(264,263),
		//seamingPair(263,262),seamingPair(262,252),

		//seamingPair(252,120),seamingPair(120,118),seamingPair(118,251),seamingPair(251,107),
		//seamingPair(107,105),seamingPair(105,247),seamingPair(247,245),seamingPair(245,243),
		//seamingPair(243,244),*/
		/////*2x2 上边*/
		///*seamingPair(268,268),seamingPair(48,48),seamingPair(7,7),seamingPair(8,8),
		//seamingPair(267,267),seamingPair(265,265),seamingPair(266,266),seamingPair(264,264),
		//seamingPair(263,263),seamingPair(262,262),

		//seamingPair(252,252),seamingPair(120,120),seamingPair(118,118),seamingPair(251,251),
		//seamingPair(107,107),seamingPair(105,105),seamingPair(247,247),seamingPair(245,245),
		//seamingPair(243,243),seamingPair(244,244),*/
		//};
		}
		///100x100 plane mesh
		{
		//	seamingInfo1.seamingPairs = {
		///*100x100 下边*/
		/*seamingPair(290,290),seamingPair(287,287),seamingPair(282,282),seamingPair(280,280),
		seamingPair(283,283),seamingPair(192,192),seamingPair(278,278),seamingPair(217,217),
		seamingPair(215,215),seamingPair(214,214),

		seamingPair(216,216),seamingPair(187,187),seamingPair(11,11),seamingPair(10,10),
		seamingPair(212,212),seamingPair(211,211),seamingPair(210,210),seamingPair(207,207),
		seamingPair(205,205),seamingPair(72,72),seamingPair(204,204),*/
		///*100x100 上边*/
		///*seamingPair(262,262),seamingPair(163,163),seamingPair(161,161),seamingPair(157,157),
		//seamingPair(8,8),seamingPair(6,6),seamingPair(260,260),seamingPair(258,258),
		//seamingPair(259,259),seamingPair(250,250),

		//seamingPair(251,251),seamingPair(249,249),seamingPair(41,41),seamingPair(42,42),
		//seamingPair(248,248),seamingPair(247,247),seamingPair(122,122),seamingPair(238,238),
		//seamingPair(242,242),seamingPair(241,241),seamingPair(239,239)*/
		//	};
		}
		///cleaned mesh
		{
		
		/*seamingInfo1.seamingPairs = {
			seamingPair(2550,388),seamingPair(359,954),seamingPair(358,953),
		seamingPair(2366,952),seamingPair(2365,951),seamingPair(2361,950),seamingPair(2551,949),

		seamingPair(171,488),seamingPair(170,470),seamingPair(2371,471),seamingPair(2564,464),
		seamingPair(2376,465),seamingPair(176,500),seamingPair(175,510),seamingPair(2552,511) };*/
		/*seamingInfo1.seamingPairs = {
			seamingPair(2550,388),seamingPair(103,957),seamingPair(359,956),seamingPair(358,955),
		seamingPair(2366,954),seamingPair(2365,953),seamingPair(2361,952),seamingPair(2360,951),
		seamingPair(118,950),seamingPair(116,949),

		seamingPair(171,488),seamingPair(170,489),seamingPair(2371,472),seamingPair(2564,470),
		seamingPair(2376,471),seamingPair(356,464),seamingPair(355,465),seamingPair(2368,467),
		seamingPair(2370,501),seamingPair(176,514)};*/

		seamingInfo1.seamingPairs = {
			seamingPair(2550,2461),seamingPair(103,2277),seamingPair(359,506),
		seamingPair(358,504),seamingPair(2366,502),seamingPair(2365,2385),seamingPair(2361,2381),
		seamingPair(118,2379),seamingPair(116,162),seamingPair(2551,161),//左肩

		seamingPair(2552,245),seamingPair(176,244),seamingPair(2370,2313),
		seamingPair(2368,2493),seamingPair(355,2491),seamingPair(356,2489),seamingPair(2376,2485),
		seamingPair(2371,543),seamingPair(170,220),seamingPair(171,222),//右肩

		seamingPair(201,309),seamingPair(203,307),seamingPair(2386,500),
		seamingPair(362,493),seamingPair(361,494),seamingPair(366,2289),seamingPair(367,267),
		seamingPair(2558,263),seamingPair(371,264),seamingPair(2330,491),//右侧

		seamingPair(2328,489),seamingPair(374,487),seamingPair(2378,485),
		seamingPair(2565,2341),seamingPair(2566,2340),seamingPair(2379,483),seamingPair(377,481),
		seamingPair(379,477),seamingPair(381,478),seamingPair(2380,2378),//右侧

		seamingPair(2567,2375),seamingPair(2385,2373),seamingPair(382,476),
		seamingPair(2559,473),seamingPair(386,469),seamingPair(388,470),seamingPair(390,2360),
		seamingPair(2560,467),seamingPair(393,465),seamingPair(395,463),//右侧

		seamingPair(399,2438),seamingPair(398,2437),seamingPair(397,2432),
		seamingPair(2512,459),seamingPair(2509,457),seamingPair(2506,455),seamingPair(2504,451),
		seamingPair(2500,452),seamingPair(2499,380),	//右侧

		seamingPair(2291,91),seamingPair(2292,89),seamingPair(354,542),
		seamingPair(353,536),seamingPair(2298,538),seamingPair(2299,535),seamingPair(350,533),
		seamingPair(349,532),seamingPair(2557,530),	//左侧

		seamingPair(345,510),seamingPair(2338,508),seamingPair(2339,2298),
		seamingPair(2332,2296),seamingPair(2333,2295),seamingPair(2345,2441),seamingPair(2342,2440),
		seamingPair(2563,2445),seamingPair(342,512),seamingPair(2284,511),//左侧

		seamingPair(2282,517),seamingPair(337,519),seamingPair(338,520),
		seamingPair(2434,2290),seamingPair(2432,2292),seamingPair(2428,523),seamingPair(336,526),
		seamingPair(334,527),seamingPair(2556,2386),seamingPair(331,376),
		seamingPair(257,373),seamingPair(2555,374)//左侧
		};
		}
		///raw mesh
		{
			/*seamingInfo1.seamingPairs = {
				seamingPair(2639,336),seamingPair(184,668),seamingPair(667,185),seamingPair(645,2435),
			seamingPair(648,398),seamingPair(646,2431),seamingPair(639,393),seamingPair(622,394),
			seamingPair(621,2443),seamingPair(579,2437),seamingPair(578,2438),seamingPair(581,180),
			seamingPair(605,179)
			};*/
		}
	}
	//clothParms
	{
		clothParms.delta_t = 0.001f;
		clothParms.frameRate = 25.f;
		clothParms.gravity = Lyra::vec3<float>(0.f, -9.8f, 0.f);
		clothParms.windParms.windVelocity = Lyra::vec3<float>(0.f, 0.f, 3.f);
		clothParms.windParms.windCod = 1.;
		clothParms.windParms.windDensity = 0.5;
		////废弃的seaming method
		//clothParms.seamingForceFactor = 100.f;
		//clothParms.seamingThreshold = 0.1f;
		//new seaming method
		//这种方法需要快速缝合，因为damping掉了内力的加速度，使的出现了一些artifacts
		//force由帧率确定，指定大概多少帧的时候基本满足缝合,这个值确定什么时间点缝合,一般在后半段，前半段不要设置
		clothParms.seamingParms.force = 20.f;
		//relex由帧率决定，指定大概多少帧的时候把延伸方向的velocity damping掉，这个值应该比force小，以便剩余的加速度damping掉
		clothParms.seamingParms.relax = 5.f;
		//damp由帧率决定，指定大概多少帧的时候把水平方向的velocity damping掉，这个值应该和force相当，为了让找到对应点
		clothParms.seamingParms.damp = 25.f;
		//设为1.f的时候舍弃内力形成的延伸方向的加速度，设为0.f的时候保留内力形成的延伸方向的加速度
		clothParms.seamingParms.atten = 1.f;
		//设为1.f的时候舍弃内力形成的水平方向的加速度，设为0.f的时候保留内力形成的水平方向的加速度
		clothParms.seamingParms.attet = 0.f;

		clothParms.enableGravity = true;
		clothParms.enableWind = false;

		clothParms.planeForceSwitch.enableStretchForce = true;
		clothParms.planeForceSwitch.enableShearForce = true;
		//没有damping的话会导致拉伸的时候后面的点停不下来
		clothParms.planeForceSwitch.enableDampingStretchForce = true;
		clothParms.planeForceSwitch.enableDampingShearForce = false;

		clothParms.spaceForceSwitch.enableBendingForce = true;
		clothParms.spaceForceSwitch.enableDampingBendingForce = true;

		//for seaming
		clothParms.shader = Lyra::Shader<float>(modelVertexShaderPath.c_str(), seamingFragmentShaderPath.c_str());
	}

	clothPatch0->LoadPatch(parms0);
	clothPatch1->LoadPatch(parms1);

	cloth->AddPatch(clothPatch0);
	cloth->AddPatch(clothPatch1);
	cloth->AddSeamingInfo(seamingInfo1);
	cloth->Integrate(clothParms);

	Lyra::ClothBVH<float> clothBvh;
	clothBvh.Build(clothPatch0, 1, bvhShader);

	clothBvh.GlBind();

	cloth->GlBind();

	body->GlBind();

	//objBVH->GlBind();

	camera.Init(setting, modelMat);

	glEnable(GL_DEPTH_TEST);
	int i = 0;
	while (!glfwWindowShouldClose(window)) {
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		camera.Update();

		MVP = camera.GetMVPMatrix();
		modelMat = camera.GetModelMatrix();
		viewMat = camera.GetViewMatrix();
		projectionMat = camera.GetProjectMatrix();

		cloth->GlUpdate();
		//cloth->Simulate();
		
		cloth->Rendering(camera, lineMode);

		bodyShader.use();
		bodyShader.setMat4("MVP", MVP);
		body->GlDrawModel(bodyShader, lineMode);

		//bvh->DrawExternalBBox(camera);
		//bvh->DrawSubBBoxs(camera);
		//objBVH->GlDraw(camera);

		clothBvh.GlDraw(camera);

		glfwSwapBuffers(window);
		glfwPollEvents();

		//system("pause");
	}
	glfwTerminate();
	return 0;
}

void GlfwReshapeCallbackFunc(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void GlfwKeyBoardCallbackFunc(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	//WASDQE 用来控制移动
	switch (key) {
	case GLFW_KEY_W:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_FRONT);
		break;
	case GLFW_KEY_S:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_BACK);
		break;
	case GLFW_KEY_A:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_LEFT);
		break;
	case GLFW_KEY_D:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_RIGHT);
		break;
	case GLFW_KEY_Q:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_DOWN);
		break;
	case GLFW_KEY_E:
		camera.Move(GraphicHelper::CameraMovement::CAMERA_UP);
		break;
	case GLFW_KEY_LEFT_CONTROL:
		{
			if (action == GLFW_PRESS)
				camera.SetLeftControlPressState(true);
			else if (action == GLFW_RELEASE) {
				camera.SetLeftControlPressState(false);
				//禁止绕target旋转
				camera.SetRotateState(false);
			}
		}
		break;
	case GLFW_KEY_ESCAPE:
		glfwTerminate();
		break;
	default:
		break;
	}

	if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
		drawVelocity = !drawVelocity;

	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		bodyLine = !bodyLine;
	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		clothLine = !clothLine;

	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
		system("pause");


	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
		drawNormal = !drawNormal;
	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
		lineMode = !lineMode;
	if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) {
		hasWind = !hasWind;
		if (hasWind)
			std::cout << "wind turn on\n";
		else
			std::cout << "wind turn off\n";
	}

}

void GlfwCursorPosCallbackFunc(GLFWwindow* window, double xpos, double ypos)
{
	camera.SetMousePosition(xpos, ypos);
}

void GlfwMouseStateCallbackFunc(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		camera.SetMouseMoveState(true);

		//左鼠标按下左ctrl按下，建立旋转标记并且初始化起始点的位置,同时禁止自身姿态调整
		if (camera.LeftControlState()) {
			camera.SetRotateState(true);

			camera.SetMouseMoveState(false);

			double xPos, yPos;
			glfwGetCursorPos(window, &xPos, &yPos);
			//记录鼠标点击时的位置
			camera.SetLastMousePosition(xPos, yPos);
		}
	} else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		camera.SetMouseMoveState(false);
		//左鼠标松开，禁止绕target旋转
		camera.SetRotateState(false);
	}
}

void GlfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
	//滚轮上滚
	if (yoffset > 0.0) {
		camera.Move(GraphicHelper::CameraMovement::CAMERA_UP);

		if (camera.LeftControlState())
			camera.Move(GraphicHelper::CameraMovement::CAMERA_FRONT);
	}
	//滚轮下滚
	else if (yoffset < 0.0) {
		camera.Move(GraphicHelper::CameraMovement::CAMERA_DOWN);
		if (camera.LeftControlState())
			camera.Move(GraphicHelper::CameraMovement::CAMERA_BACK);
	}
}