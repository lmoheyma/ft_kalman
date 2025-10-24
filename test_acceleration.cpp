// Test pour vérifier la transformation de l'accélération
#include <iostream>
#include <vector>
#include <cmath>
#include "inc/maths.hpp"

int main() {
    // Données de votre exemple
    std::vector<double> acc = {-0.00828404, -0.012101, -0.0139619};
    std::vector<double> dir = {-0.00624298, -0.00394704, 0.00408142};
    
    std::cout << "Accélération brute : ";
    for (auto a : acc) std::cout << a << " ";
    std::cout << "\n";
    
    std::cout << "Direction (radians) : ";
    for (auto d : dir) std::cout << d << " ";
    std::cout << "\n\n";
    
    // Transformation avec rotation
    Matrix Rx(3, std::vector<double>(3, 0.0));
    Matrix Ry(3, std::vector<double>(3, 0.0));
    Matrix Rz(3, std::vector<double>(3, 0.0));
    
    setRotationX(Rx, dir[0]);
    setRotationY(Ry, dir[1]);
    setRotationZ(Rz, dir[2]);
    
    Matrix R = multiply(Rz, Ry);
    R = multiply(R, Rx);
    
    std::vector<double> acc_transformed = multiplyMatrixVector(R, acc);
    
    std::cout << "Accélération transformée : ";
    for (auto a : acc_transformed) std::cout << a << " ";
    std::cout << "\n";
    
    // Norme
    double norm_before = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    double norm_after = std::sqrt(acc_transformed[0]*acc_transformed[0] + 
                                   acc_transformed[1]*acc_transformed[1] + 
                                   acc_transformed[2]*acc_transformed[2]);
    
    std::cout << "Norme avant : " << norm_before << "\n";
    std::cout << "Norme après : " << norm_after << "\n";
    
    return 0;
}
