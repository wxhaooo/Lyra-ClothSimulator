#include "pch.h"
#include<Eigen/Dense>

using namespace Eigen;

TEST(EigenLinearSolverTest, ColPivHouseholderQRTest)
{
	MatrixXf AMat = MatrixXf::Random(4, 3);
	MatrixXf bVec = MatrixXf::Random(4, 1);

	/*AMat << 1, -2, 3, 5, 8, 6, 9, 7, 6, 4, -5, 1;
	bVec << 1, 3, 5, 2;*/

	std::cout << AMat << "\n";
	std::cout << bVec << "\n";

	MatrixXf xVec = AMat.colPivHouseholderQr().solve(bVec);

	std::cout << (AMat * xVec - bVec).norm() / bVec.norm();
}