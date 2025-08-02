#include "../inc/maths.hpp"

Matrix setRotationX(Matrix& matrix, double phi) {
    matrix[0][0] = 1; matrix[0][1] = 0;             matrix[0][2] = 0;
    matrix[1][0] = 0; matrix[1][1] = std::cos(phi); matrix[1][2] = std::sin(phi);
    matrix[2][0] = 0; matrix[2][1] = -std::sin(phi);matrix[2][2] = std::cos(phi);
    return matrix;
}

Matrix setRotationY(Matrix& matrix, double theta) {
    matrix[0][0] = std::cos(theta);     matrix[0][1] = 0;   matrix[0][2] = -std::sin(theta);
    matrix[1][0] = 0;                   matrix[1][1] = 1;   matrix[1][2] = 0;
    matrix[2][0] = std::sin(theta);     matrix[2][1] = 0;   matrix[2][2] = std::cos(theta);
    return matrix;
}

Matrix setRotationZ(Matrix& matrix, double psi) {
    matrix[0][0] = std::cos(psi);   matrix[0][1] = std::sin(psi);   matrix[0][2] = 0;
    matrix[1][0] = -std::sin(psi);  matrix[1][1] = std::cos(psi);   matrix[1][2] = 0;
    matrix[2][0] = 0;               matrix[2][1] = 0;               matrix[2][2] = 1;
    return matrix;
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    size_t n = A.size();
    size_t m = B[0].size();
    size_t p = B.size();
    Matrix result(n, std::vector<double>(m, 0.0));

    for (size_t i = 0; i < n; ++i) {
        for (size_t k = 0; k < p; ++k) {
            double temp = A[i][k];
            for (size_t j = 0; j < m; ++j) {
                result[i][j] += temp * B[k][j];
            }
        }
    }
    return result;
}

Vector multiplyMatrixVector(const Matrix& matrix, const Vector& vector) {
    Vector result(matrix.size(), 0.0);
    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = 0; j < vector.size(); ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
    return result;
}

Matrix identityMatrix(size_t n) {
    Matrix result(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            if (i == j) result[i][j] = 1.0;
        }
    }
    return result;
}

Matrix matrixScalar(Matrix matrix, const double scalar) {
    for (size_t i = 0; i < matrix.size(); i++) {
        for (size_t j = 0; j < matrix[0].size(); j++) {
            matrix[i][j] *= scalar;
        }
    }
    return matrix;
}

Matrix mergeMatrixVertical(const Matrix m1, const Matrix m2) {
    Matrix mergedMatrix(m1.size()+m2.size(), std::vector<double>(m1.size(), 0.0));

    for (size_t i = 0; i < m1.size(); i++) {
        for (size_t j = 0; j < m1[0].size(); j++) {
            mergedMatrix[i][j] = m1[i][j];
            mergedMatrix[i + m1.size()][j] = m2[i][j];
        }
    }
    return mergedMatrix;
}

Matrix transpose(const Matrix matrix) {
    Matrix transposedMatrix(matrix[0].size(), std::vector<double>(matrix.size(), 0.0));

    for (size_t i = 0; i < matrix.size(); i++) {
        for (size_t j = 0; j < matrix[0].size(); j++) {
            transposedMatrix[j][i] = matrix[i][j];
        }
    }
    return transposedMatrix;
}

Matrix diagonalMatrix(const Vector& diagonal) {
    size_t n = diagonal.size();
    Matrix mat(n, Vector(n, 0.0));

    for (size_t i = 0; i < n; ++i)
        mat[i][i] = diagonal[i];

    return mat;
}

Matrix addMatrix(const Matrix& A, const Matrix& B) {
    size_t rows = A.size();
    size_t cols = A[0].size();
    Matrix result(rows, std::vector<double>(cols, 0.0));

    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = A[i][j] + B[i][j];

    return result;
}

Vector addVectors(const Vector& a, const Vector& b) {
    if (a.size() != b.size())
        throw std::invalid_argument("Vectors must have the same size");

    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + b[i];

    return result;
}

Matrix inverseMatrix(const Matrix& m) {
    size_t n = m.size();
    if (n != m[0].size())
        throw std::invalid_argument("La matrice doit être carrée");
    Matrix inv(n, std::vector<double>(n, 0.0));
    double det = 0.0;
    if (n == 2) {
        det = m[0][0]*m[1][1] - m[0][1]*m[1][0];
        if (std::abs(det) < 1e-12)
            throw std::runtime_error("Matrice non inversible");
        inv[0][0] =  m[1][1] / det;
        inv[0][1] = -m[0][1] / det;
        inv[1][0] = -m[1][0] / det;
        inv[1][1] =  m[0][0] / det;
        return inv;
    } else if (n == 3) {
        det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
        if (std::abs(det) < 1e-12)
            throw std::runtime_error("Matrice non inversible");
        inv[0][0] =  (m[1][1]*m[2][2] - m[1][2]*m[2][1]) / det;
        inv[0][1] = -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) / det;
        inv[0][2] =  (m[0][1]*m[1][2] - m[0][2]*m[1][1]) / det;
        inv[1][0] = -(m[1][0]*m[2][2] - m[1][2]*m[2][0]) / det;
        inv[1][1] =  (m[0][0]*m[2][2] - m[0][2]*m[2][0]) / det;
        inv[1][2] = -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) / det;
        inv[2][0] =  (m[1][0]*m[2][1] - m[1][1]*m[2][0]) / det;
        inv[2][1] = -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) / det;
        inv[2][2] =  (m[0][0]*m[1][1] - m[0][1]*m[1][0]) / det;
        return inv;
    } else if (n == 4) {
        for (int i = 0; i < 4; ++i) {
            double minor_det =
                m[1][(i+1)%4] * (m[2][(i+2)%4]*m[3][(i+3)%4] - m[2][(i+3)%4]*m[3][(i+2)%4]) -
                m[1][(i+2)%4] * (m[2][(i+1)%4]*m[3][(i+3)%4] - m[2][(i+3)%4]*m[3][(i+1)%4]) +
                m[1][(i+3)%4] * (m[2][(i+1)%4]*m[3][(i+2)%4] - m[2][(i+2)%4]*m[3][(i+1)%4]);
            det += (i%2==0 ? 1 : -1) * m[0][i] * minor_det;
        }
        if (std::abs(det) < 1e-12)
            throw std::runtime_error("Matrice non inversible");
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                Matrix minor(3, std::vector<double>(3));
                int mi = 0;
                for (int ii = 0; ii < 4; ++ii) {
                    if (ii == i) continue;
                    int mj = 0;
                    for (int jj = 0; jj < 4; ++jj) {
                        if (jj == j) continue;
                        minor[mi][mj] = m[ii][jj];
                        ++mj;
                    }
                    ++mi;
                }
                double minor_det =
                    minor[0][0]*(minor[1][1]*minor[2][2] - minor[1][2]*minor[2][1])
                  - minor[0][1]*(minor[1][0]*minor[2][2] - minor[1][2]*minor[2][0])
                  + minor[0][2]*(minor[1][0]*minor[2][1] - minor[1][1]*minor[2][0]);
                inv[j][i] = ((i+j)%2==0 ? 1 : -1) * minor_det / det;
            }
        }
        return inv;
    } else {
        throw std::invalid_argument("uniquement 2x2, 3x3 et 4x4");
    }
}

void printMatrix(Matrix matrix) {
    for (size_t i = 0; i < matrix.size(); i++) {
        for (size_t j = 0; j < matrix[0].size(); j++) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void printVector(Vector vector) {
    std::vector<double>::iterator vit;
    for (vit = vector.begin(); vit != vector.end(); ++vit) {
        std::cout << *vit << std::endl;
    }
}
