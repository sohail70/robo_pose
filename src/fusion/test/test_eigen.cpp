// MIT License
//
// Copyright (c) 2023 Soheil Espahbodi Nia
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include<iostream>
#include<gtest/gtest.h>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<eigen3/unsupported/Eigen/AutoDiff>
TEST(Eigen, simple)
{
    // // ASSERT_EQ(1,1);
    // Eigen::VectorXd v1; //3 by 1 vector: instead of X we can put a number but we put X for any dimension that we specify
    // v1 = Eigen::VectorXd(3,1);
    
    // v1<<6,7,8;
    // std::cout<<v1<<"\n";
    // std::cout<<v1.sum()<<" "<<v1.prod()<<" "<<v1.mean()<<" "<<v1.minCoeff()<<" "<<v1.norm()<<" " <<v1.size()<<" \n";
    // Eigen::VectorXd v2(3,1);
    // v2<<1,1,1;
    // std::cout<<v1.dot(v2)<<"\n";

    // Eigen::MatrixXd A(2,2); //2by2 matrix
    // Eigen::MatrixXd B(2,2); //2by2 matrix
    // A<<1,2,3,4; //fill the matrix row wise
    // B(0,0) = 2; B(0,1) = 3 ; B(1,0) = 4; B(1,1) = 5;
    // Eigen::MatrixXd C = A+B;
    // std::cout<<C<<"\n";
    // Eigen::MatrixXd D = A.inverse();
    // std::cout<<D<<"\n";
    // double Adet;
    // Adet = A.determinant();
    // std::cout<<Adet<<"\n";

    // Eigen::VectorXcd eVals = A.eigenvalues(); //why Xcd ? X for any dimensional you want, c for complex because eigen values can be complex and d for double
    // std::cout<<eVals<<"\n";

    // Eigen::MatrixXd E = A*B;
    // std::cout<<E<<"\n";

    // Eigen::MatrixXd matrixZeros;
    // matrixZeros.setZero(5,4);
    // std::cout<<matrixZeros<<"\n";

    // Eigen::MatrixXd matrixOnes;
    // matrixOnes.setOnes(4,3);
    // std::cout<<matrixOnes<<"\n";

    // Eigen::MatrixXd matrixIdentity;
    // matrixIdentity = Eigen::MatrixXd::Identity(4,4); //i don't know why matrixIdentity.Identity(4,4) gave me a core dump
    // std::cout<<matrixIdentity<<"\n";

    // Eigen::MatrixXd matrixVpartition = matrixIdentity.block(0,0,2,3); //The block of identity matrix above 0 to 2 and 0 to 3  : injoori yad begir ke boro be top left cornet va grab kun 2 rows and 3 columns
    // std::cout<<matrixVpartition<<"\n";

    // Eigen::MatrixXd At;
    // At = matrixVpartition.transpose();
    // std::cout<<At<<"\n";

    ////////////////////////AutoDiff//////////////////
    //   // Note different initialization
    // Eigen::AutoDiffScalar<Eigen::Vector2d> x(8.0, Eigen::Vector2d(1,0)), y;
    // Eigen::AutoDiffJacobian

    // y = x*x;

    // std::cout << "x = " << x << "\n"
    //           << "y = " << y << "\n"
    //           << "y' = " << y.derivatives()[0] << "\n";
    /////////////////////////////////////////////////
    std::cout<<"Computing psuedo inverse becaues zero in the diagonal causes nans \n";
    Eigen::MatrixXd mat;
    mat.setZero(6,6);
    mat(0,0) = 2;
    mat(1,1) = 0;
    mat(2,2) = 1;
    mat(3,3) = 0;
    mat(4,4) = 1;
    mat(5,5) = 0;
    // Eigen::MatrixXd mat2 = mat.block<3,3>(3,3);
    std::cout<<mat.completeOrthogonalDecomposition().pseudoInverse();

}