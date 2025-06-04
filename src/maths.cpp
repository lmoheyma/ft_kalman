#include <vector>
#include <cstddef>

using Matrix = std::vector<std::vector<double>>;

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
