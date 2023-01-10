#include "../include/utility.h"


std::vector<std::vector<double> > MatrixTranspose(std::vector<std::vector<double> >& Matrix)
{
	// Transpose of a Matrix rows x columns will be a matrix having columns x rows
	// Initialization of Transpose matrix


	std::vector<std::vector<double> > Transpose(Matrix[0].size(), std::vector<double>(Matrix.size()));


	for (int i = 0; i < Transpose.size(); i++)
	{
		for (int j = 0; j < Transpose[0].size(); j++)
		{
			Transpose[i][j] = Matrix[j][i];
		}
	}

	return Transpose;

}

bool Inverse3x3(std::vector<std::vector<double> >& a, std::vector<std::vector<double> >& Inverse)
{
	//Inverse of a 3x3 matrix
	float determinant;

	std::vector<std::vector<double> > cofac_matrix(3, std::vector<double>(3));
	determinant = a[0][0] * a[1][1] * a[2][2] + a[0][1] * a[1][2] * a[2][0]
		+ a[0][2] * a[1][0] * a[2][1] - (a[0][2] * a[1][1] * a[2][0] +
			a[0][1] * a[1][0] * a[2][2] + a[0][0] * a[1][2] * a[2][1]);
	//std::cout << "determinat is:" << determinant << '\n';

	if (determinant == 0)
	{
		//Matrix cannot be inverted
		return false;
	}



	cofac_matrix[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * 1 / determinant;
	cofac_matrix[1][0] = -(a[0][1] * a[2][2] - a[0][2] * a[2][1]) * 1 / determinant;
	cofac_matrix[2][0] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * 1 / determinant;
	cofac_matrix[0][1] = -(a[1][0] * a[2][2] - a[1][2] * a[2][0]) * 1 / determinant;
	cofac_matrix[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * 1 / determinant;
	cofac_matrix[2][1] = -(a[0][0] * a[1][2] - a[1][0] * a[0][2]) * 1 / determinant;
	cofac_matrix[0][2] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) * 1 / determinant;
	cofac_matrix[1][2] = -(a[0][0] * a[2][1] - a[2][0] * a[0][1]) * 1 / determinant;
	cofac_matrix[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) * 1 / determinant;

	Inverse = MatrixTranspose(cofac_matrix);

	for (int i = 0; i < Inverse.size(); i++)
	{
		for (int j = 0; j < Inverse[0].size(); j++)
		{
			if (Inverse[i][j] == -0.0f)
			{
				Inverse[i][j] = 0;
			}
		}
	}
	return true;

}

std::vector<std::vector<double> > MatrixMultiplication(std::vector<std::vector<double> >& A, std::vector<std::vector<double> >& B)
{
	//Function that implements matrix multiplication

	//Getting rows and columns of the 2 matrices:

	//(NxM)x(MxP) matrix multiplication result will be a NxP matrix


	if (int(A[0].size()) != int(B.size()))
	{
		//Throw an error!
		std::cout << "Cannot perform matrix multiplication! Matrix size missmatch!";
		std::cout << A[0].size() << '\t' << B.size() << '\n';
	}

	//Initialization of the result matrix

	else
	{
		std::vector<std::vector<double> > Resulting_Matrix(A.size(), std::vector<double>(B[0].size()));


		//Start performing matrix multiplication

		for (int i = 0; i < A.size(); i++)
		{
			for (int j = 0; j < B[0].size(); j++)
			{
				Resulting_Matrix[i][j] = 0;
				for (int k = 0; k < A[0].size(); k++)
				{

					Resulting_Matrix[i][j] += A[i][k] * B[k][j];
				}

			}
		}

		return Resulting_Matrix;
	}
}

//Create quaternion from Euler angles
double* createQuaternionFromEulerAngles(double Yaw, double Pitch, double Roll)
{
    double q[4];

    //Create an array containing the components of the quaternion q:
    q[0] = sin(Roll/2) * cos(Pitch/2) * cos(Yaw/2) - cos(Roll/2) * sin(Pitch/2) * sin(Yaw/2);
    q[1] = cos(Roll/2) * sin(Pitch/2) * cos(Yaw/2) + sin(Roll/2) * cos(Pitch/2) * sin(Yaw/2);
    q[2] = cos(Roll/2) * cos(Pitch/2) * sin(Yaw/2) - sin(Roll/2) * sin(Pitch/2) * cos(Yaw/2);
    q[3] = cos(Roll/2) * cos(Pitch/2) * cos(Yaw/2) + sin(Roll/2) * sin(Pitch/2) * sin(Yaw/2);

    return q;
}



double* rotateVectorFromQuaternion(double* v,double* q)
{
    // v:   Vector i want to rotate;
    // q:   Quaternion

    double rotated_vector[3];

    //Start assigning to rotate vector array it's new components:

    rotated_vector[0] = (1-2*q[1]*q[1]-2*q[2]*q[2])*v[0] + 2*(q[0]*q[1] + q[3]*q[2])*v[1] +
            2*(q[0]*q[2] - q[3]*q[1])*v[2];

    rotated_vector[1] = 2*(q[0]*q[1] -q[3]*q[2])*v[0] + (1 - 2*q[0]*q[0] -2*q[2]*q[2])*v[1] +
            2*(q[1]*q[2] + q[3]*q[0])*v[3];

    rotated_vector[2] = 2*(q[0]*q[2] + q[3]*q[1])*v[0] + 2*(q[1]*q[2] -q[3]*q[0])*v[1] +
            (1-2*q[0]*q[0] - 2*q[1]*q[1])*v[2];


    return rotated_vector;


}

//Get Euler angles from quaternion
double* EulerAnglesFromQuaternion(double *q)
{
    double EulerAngles[3];


    //Roll:
    EulerAngles[0] = atan2(+2.0 * (q[3] * q[0] + q[1] * q[2]), +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1]));

    //Pitch:
    EulerAngles[1] = asinf(-2.0*(q[1]*q[3]-q[0]*q[2]));

    //Yaw
    EulerAngles[2] = atan2f(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]);

    return EulerAngles;
    
}



void showMatrix(std::vector<std::vector<double> >& Matrix)
{
    std::cout << "[";
    for (int i=0; i < Matrix.size(); i++)
    {
        std::cout << "[";
        for (int j=0; j < Matrix[i].size(); j++)
        {
            std::cout << Matrix[i][j] << "\t";
        }
        std::cout << "]\n";
    }


}