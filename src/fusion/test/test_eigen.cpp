#include<iostream>
#include<gtest/gtest.h>
#include<Eigen/Dense>



TEST(Eigen, simple)
{
    // ASSERT_EQ(1,1);
    Eigen::VectorXd v1; //3 by 1 vector: instead of X we can put a number but we put X for any dimension that we specify
    v1 = Eigen::VectorXd(3,1);
    
    v1<<6,7,8;
    std::cout<<v1<<"\n";
    std::cout<<v1.sum()<<" "<<v1.prod()<<" "<<v1.mean()<<" "<<v1.minCoeff()<<" "<<v1.norm()<<" " <<v1.size()<<" \n";
    Eigen::VectorXd v2(3,1);
    v2<<1,1,1;
    std::cout<<v1.dot(v2)<<"\n";

    Eigen::MatrixXd A(2,2); //2by2 matrix
    Eigen::MatrixXd B(2,2); //2by2 matrix
    A<<1,2,3,4; //fill the matrix row wise
    B(0,0) = 2; B(0,1) = 3 ; B(1,0) = 4; B(1,1) = 5;
    Eigen::MatrixXd C = A+B;
    std::cout<<C<<"\n";
    Eigen::MatrixXd D = A.inverse();
    std::cout<<D<<"\n";
    double Adet;
    Adet = A.determinant();
    std::cout<<Adet<<"\n";

    Eigen::VectorXcd eVals = A.eigenvalues(); //why Xcd ? X for any dimensional you want, c for complex because eigen values can be complex and d for double
    std::cout<<eVals<<"\n";

    Eigen::MatrixXd E = A*B;
    std::cout<<E<<"\n";

    Eigen::MatrixXd matrixZeros;
    matrixZeros.setZero(5,4);
    std::cout<<matrixZeros<<"\n";

    Eigen::MatrixXd matrixOnes;
    matrixOnes.setOnes(4,3);
    std::cout<<matrixOnes<<"\n";

    Eigen::MatrixXd matrixIdentity;
    matrixIdentity = Eigen::MatrixXd::Identity(4,4); //i don't know why matrixIdentity.Identity(4,4) gave me a core dump
    std::cout<<matrixIdentity<<"\n";

    Eigen::MatrixXd matrixVpartition = matrixIdentity.block(0,0,2,3); //The block of identity matrix above 0 to 2 and 0 to 3  : injoori yad begir ke boro be top left cornet va grab kun 2 rows and 3 columns
    std::cout<<matrixVpartition<<"\n";

    Eigen::MatrixXd At;
    At = matrixVpartition.transpose();
    std::cout<<At<<"\n";
}