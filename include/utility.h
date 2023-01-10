#include<vector>
#include <iostream>
#include <cmath>

std::vector<std::vector<double> > MatrixTranspose(std::vector<std::vector<double> >& Matrix);
bool Inverse3x3(std::vector<std::vector<double> >& a, std::vector<std::vector<double> >& Inverse);
std::vector<std::vector<double> >  MatrixMultiplication(std::vector<std::vector<double> >& A, std::vector<std::vector<double> >& B);

double* EulerAnglesFromQuaternion(double *q);
double* rotateVectorFromQuaternion(double* v,double* q);
double* createQuaternionFromEulerAngles(double Yaw, double Pitch, double Roll);


void showMatrix(std::vector<std::vector<double> >& Matrix);