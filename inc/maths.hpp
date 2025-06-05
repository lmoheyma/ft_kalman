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
void printMatrix(Matrix matrix);
void printVector(Vector vector);
