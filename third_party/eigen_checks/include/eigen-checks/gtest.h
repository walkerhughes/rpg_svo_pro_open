#pragma once
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <cmath>

// These macros return boolean expressions so they can be used inside EXPECT_TRUE()
#define EIGEN_MATRIX_EQUAL(MatrixA, MatrixB) \
  (MatrixA).isApprox((MatrixB))

#define EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB) \
  (MatrixA).isApprox((MatrixB), 1e-10)

#define EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision) \
  (MatrixA).isApprox((MatrixB), (Precision))

#define EXPECT_NEAR_EIGEN(MatrixA, MatrixB, Precision) \
  EXPECT_TRUE((MatrixA).isApprox((MatrixB), (Precision))) \
      << "MatrixA:\n" << (MatrixA) << "\nMatrixB:\n" << (MatrixB) \
      << "\nPrecision: " << (Precision)

#define EXPECT_ZERO_EIGEN(Matrix, Precision) \
  EXPECT_TRUE((Matrix).isZero((Precision))) \
      << "Matrix:\n" << (Matrix) << "\nExpected zero with precision: " << (Precision)
