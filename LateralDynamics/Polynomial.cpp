//
//  Polynomial.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#include "Polynomial.hpp"

Polynomial::Polynomial(){}; //Default Constructor


Polynomial::Polynomial(Eigen::ArrayXd polyPairs){
    this -> polyPairs = polyPairs;
}

void Polynomial::printPolynomial(){
    std::string plusSign;
    bool first = true;
    //Iterate through coefficients, assign them from largest degress
    std::string res = "";
    
    for(long long int i = polyPairs.size() - 1;i >= 0;--i){
        plusSign = first ? "" : "+";
        res += plusSign + " " + std::to_string(polyPairs(polyPairs.size()- 1 - i)) + "s^" + std::to_string(i) + " ";
        first = false;
    }
    std::cout << res << std::endl;
}

//Operator Overloads
Polynomial Polynomial::operator+(Polynomial const &b){
    auto A = polyPairs;
    auto B = b.polyPairs;
    
    long long int i =A.size() - 1,j = B.size() - 1;
    
    if(i>j)
    {std::swap(A,B);std::swap(i,j);}
    
    Eigen::ArrayXd newPairs = B;
    
    //We add from the lower degrees to the higher degrees. This means adding from the last index to the first index of the smaller array, a.
    while(i > -1)
    {
        
        newPairs(j--) += A(i--);
    }
    Polynomial C (newPairs);
    return C;
}
Polynomial Polynomial::operator-(Polynomial const&b){
    auto A = polyPairs;
    auto B = b.polyPairs;
    if(A.size() == B.size())
        return Polynomial(A - B);
    
    long long int i =A.size() - 1,j = B.size() - 1;
    long long int z = i;
    Eigen::ArrayXd newPairs(z+1);newPairs.setZero();
    
    
    if(j > i)
    {newPairs.resize(j);z = j+1;}
    
    while(i > -1 && j > -1)
    {
        newPairs[z--] = A[i--] - B[j--];
    }
    while(i > -1)
        newPairs[z--] = A[i--];
    while(j > -1)
        newPairs[z--] = B[j--];
    
    Polynomial C (newPairs);
    return C;
}
Polynomial Polynomial::operator *(Polynomial const &b){
    //Multiplication affects both coefficients and degrees
    auto A = polyPairs;
    auto B = b.polyPairs;
    if(A.size() == B.size()) //Eigen Operation Valid
        return Polynomial(A*B);
    
  
    long long int i = A.size() - 1, j = B.size() - 1;
    if(i > j)
    {std::swap(A,B);std::swap(i,j);}
    
    Eigen::ArrayXd C(j + i + 1);C.setZero();
 
    for(long long int x =0 ; x < j + 1; ++x){
        for(long long int y = 0; y < i + 1; ++y){
            C(x + y) += A(y) * B(x);
        }
    }
    Polynomial D (C);
    return D;
}

//Base on long division
//NOT COMPLETE
/*
Polynomial Polynomial::operator/(Polynomial const&b){
    auto A = polyPairs;
    auto B = b.polyPairs;
    Eigen::ArrayXd newPairs;
    Polynomial C (newPairs);
    return C;
}
*/
double Polynomial::polyParser(double s){
    //Parse out our polynomial
    double finalVal = 0.0;
    auto A = polyPairs;
    for(int a =0; a < A.size();++a){
        finalVal += A(a) * std::pow(s,A.size() - a); // Degree = Size - Index
    }
    return finalVal;
}
