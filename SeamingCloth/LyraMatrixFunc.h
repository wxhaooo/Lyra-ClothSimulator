#pragma once
#include"Common.h"
#include<iostream>
#include<vector>

namespace Lyra
{
	enum SparseSolverCategory {
		//direct method
		SOLVER_LLT, SOLVER_LDLT, SOLVER_LU, SOLVER_QR,
		//iterative method
		SOLVER_CG, SOLVER_LEAST_SQUARE_CG, SOLVER_BiCG_STAB
	};

	template<typename T>
	using triplet = Eigen::Triplet<T>;

	template<typename T>
	using tripletVector = std::vector<triplet<T>>;

	template<typename T>
	void InsertMat3x3IntoSparseMat(mat<T, 3, 3>& smallMat, matSp<T>& spMat,  uint32 row, uint32 col);

	template<typename T>
	void SetSparseIdentityMatrix(matSp<T>& identityMatSp, uint32 rows, uint32 cols);

	template<typename T>
	void SetSparseMatrixFromTripletVector(tripletVector<T>& triplets, matSp<T>& spMat);

	template<typename T>
	void SolveSparseLinearSystem(vecX<T>& xVec, matSp<T>& AMat, vecX<T>& bVec,
		SparseSolverCategory category);
	
	//AMat must be self-adjoint, otherwise, solution could not right.
	template<typename T>
	void SolveSparseLinearSystemByCG(vecX<T>& xVec, matSp<T>& AMat, vecX<T>& bVec);

	//AMat is arbitrary and rectangle
	template<typename T>
	void SolveSparseLinearSystemByQR(vecX<T>& xVec, matSp<T>& AMat, vecX<T>& bVec);

	/*template<typename T>
	void SolveDenseLinearSystem(vecX<T>& xVec, matX<T>& AMat, vecX<T>& bVec);*/

	template<typename T>
	using cgSolverSp = Eigen::ConjugateGradient<Eigen::SparseMatrix<T>,
						Eigen::Upper | Eigen::Lower>;

	template<typename T>
	using qrSolverSp = Eigen::SparseQR<Eigen::SparseMatrix<T>,
						Eigen::COLAMDOrdering<int>>;
}

namespace Lyra
{
	template<typename T>
	void SetSparseIdentityMatrix(matSp<T>& identityMatSp, uint32 rows, uint32 cols)
	{
		identityMatSp.setZero();

		uint32 matSize = rows * cols;
		identityMatSp.resize(rows, cols);

		uint32 entry = std::min(rows, cols);

		for (uint32 i = 0; i < entry; i++) {
			identityMatSp.insert(i, i) = T(1);
		}

		identityMatSp.makeCompressed();

		//std::cout << identityMatSp << "\n";
	}

	template<typename T>
	void SetSparseMatrixFromTripletVector(tripletVector<T>& triplets, matSp<T>& spMat)
	{
		if (spMat.size() == 0) {
			//if sparse matrix is 0x0,auto generate appropriate sparse matrix
			int maxRows = -1, maxCols = -1;
			for (auto& trip : triplets) {
				if (trip.row() > maxRows)
					maxRows = trip.row();
				if (trip.col() > maxCols)
					maxCols = trip.col();
			}
			//if triplets is also empty,return error information
			if (maxRows == -1 || maxCols == -1) {
				std::cerr << "size of sparse matrix is 0x0!!!!\n";
				return;
			}

			spMat.resize(maxRows + 1, maxCols + 1);
			spMat.setZero();
		}
		spMat.setZero();
		spMat.setFromTriplets(triplets.begin(), triplets.end());
	}

	template<typename T>
	void SolveSparseLinearSystem(vecX<T>& xVec, matSp<T>& AMat, vecX<T>& bVec,
		SparseSolverCategory category)
	{
		if (category == SparseSolverCategory::SOLVER_CG)
			SolveSparseLinearSystemByCG(xVec, AMat, bVec);
		else if (category == SparseSolverCategory::SOLVER_QR)
			SolveSparseLinearSystemByQR(xVec, AMat, bVec);

		//other solver waits to add
	}

	template<typename T>
	void SolveSparseLinearSystemByCG(vecX<T>& xVec, matSp<T>& AMatSp, vecX<T>& bVec)
	{
		cgSolverSp<T> cgSolver;
		cgSolver.compute(AMatSp);
		if (cgSolver.info() != Eigen::Success) {
			std::cerr << "decomposition failed\n";
			return;
		}
		xVec = cgSolver.solve(bVec);
	}

	template<typename T>
	void SolveSparseLinearSystemByQR(vecX<T>& xVec, matSp<T>& AMat, vecX<T>& bVec)
	{
		qrSolverSp<T> qrSolver;
		qrSolver.compute(AMat);
		if (qrSolver.info() != Eigen::Success) {
			std::cerr << " QR decomposition failed\n";
			return;
		}
		xVec = qrSolver.solve(bVec);
	}

	template<typename T>
	void InsertMat3x3IntoSparseMat(mat<T, 3, 3>& smallMat, matSp<T>& spMat, uint32 row, uint32 col)
	{
		for (uint32 i = 0; i < 3; i++) {
			for (uint32 j = 0; j < 3; j++) {
				spMat.coeffRef(row + i, col + j) += smallMat(i, j);
			}
		}
		//spMat.makeCompressed();
	}
}