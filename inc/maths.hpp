#pragma once

#include <vector>
#include <cstddef>
#include <iostream>
#include <cmath>

using Matrix = std::vector<std::vector<double> >;
using Vector = std::vector<double>;

Matrix setRotationX(Matrix& matrix, double phi);
Matrix setRotationY(Matrix& matrix, double theta);
Matrix setRotationZ(Matrix& matrix, double psi);
Matrix multiply(const Matrix& A, const Matrix& B);
Vector multiplyMatrixVector(const Matrix& matrix, const Vector& vector);
Matrix identityMatrix(size_t n);
Matrix matrixScalar(Matrix matrix, const double scalar);
Matrix mergeMatrixVertical(const Matrix m1, const Matrix m2);
Matrix transpose(const Matrix matrix);
Matrix diagonalMatrix(const Vector& diagonal);
Matrix addMatrix(const Matrix& A, const Matrix& B);
Vector addVectors(const Vector& a, const Vector& b);
void printMatrix(Matrix matrix);
void printVector(Vector vector);
