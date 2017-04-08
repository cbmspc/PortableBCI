//ArduinoMatrixMathDouble.cpp
//#if defined(ARDUINO) && (ARDUINO >= 100)
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//#include "HardwareSerial.h"
#include <math.h>
#include "ArduinoMatrixMathDouble.h"


void CholeskyDecomp(double *B, int n)
{
	//Cholesky Decomp
	//where B = L * L'
	//replace B with L
	
	int i, j, k;

	for (i = 0; i < n; i++)
	{
		B[i*n + i] = B[i*n + i];
		for (j = 0; j < i; j++)
		{
			B[i*n + j] = B[i*n + j];
			for (k = 0; k < j; k++)
			{
				B[i*n + j] -= B[i*n + k] * B[j*n + k];
			}
			B[i*n + j] /= B[j*n + j];
			B[j*n + i] = 0.0;
			B[i*n + i] -= pow(B[i*n + j],2.0);
		}
		B[i*n + i] = sqrt(B[i*n + i]);
	}
}

void LDLDecomp(double *B, double *D, int n)
{
	//LDL Decomp --- fixed on 04/07/2017 but untested
	//where B = L * D * L'
	//replace B with L, D (vector) with D
	
	float V[n], s;
	int i, j, k;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < i; j++)
		{
			V[j] = B[i*n + j] * D[j];
		}
		
		s = 0.0;
		for (j = 0; j < i; j++)
		{
			s += B[i*n + j] * V[j];
		}
		V[i] = B[i*n + i] - s;
		D[i] = V[i];
		
		for (k = 1; i+k < n; k++)
		{
			s = 0.0;
			for (j = 0; j < i; j++)
			{
				s += B[(i+k)*n + j] * V[j];
			}
			B[(i+k)*n + i] = (B[(i+k)*n + i] - s) / V[i];
		}
		
		B[i*n + i] = 1.0;
		for (j = 0; j < i; j++)
		{
			B[j*n + i] = 0.0;
		}
	}
}

void det(double *A, double d, int n)
{
	double B[n*n];
	double D[n];
	int i, j;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			B[i*n + j] = A[i*n + j];
		}
	}

	LDLDecomp(B, D, n);

	d = B[0] * D[0] * B[0];
	for (i = 1; i < n; i++)
	{
		d *= B[i*n + i] * D[i] * B[i*n + i];
	}

}

void LowerTriangularInverse(double *L, int n)
{
	//Solve L * L^-1 = I
	//for L^-1 by
	//replacing L with L^-1

	int i, j, k;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < i; j++)
		{
			L[i*n + j] = -L[i*n + j] * L[j*n + j]; // use float s if this doesnt work
			for (k = j + 1; k < i; k++)
			{
				L[i*n + j] -= L[i*n + k] * L[k*n + j];
			}
			L[i*n + j] /= L[i*n + i];
			L[j*n + i] = 0.0;
		}
		L[i*n + i] = 1.0 / L[i*n + i];
	}
}

void MatrixMultiply(double *A, double *B, double *C, int n, int m, int p)
{
	// C is n x p
	// A is n x m
	// B is m x p

	int i, j, k;

	for (i = 0; i<n; i++)
	{
		//single row from A
		for (k = 0; k<p; k++)
		{
			//C[i*p + k] = 0.0;
			C[i*p + k] = A[i*m] * B[k];
			//for (j = 0; j<m; j++)
			for (j = 1; j<m; j++)
			{
				C[i*p + k] += A[i*m + j] * B[j*p + k];
			}
		}
	}
}

void MatrixTranspose(double* A, double* AT, int n, int m)
{
	// A = input matrix (n x m)
	// AT = output matrix = the transpose of AT (m x n)
	int i, j;
	for (i = 0; i < n; i++)
		for (j = 0; j < m; j++)
			AT[j*n + i] = A[i*m + j];
}

void MatrixTransposeSquare(double *A, int n)
{
	// A is n x n

	int i, j;
	double el;

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < i; j++)
		{
			el = A[i*n + j];
			A[i*n + j] = A[j*n + i];
			A[j*n + i] = el;
		}
	}
}

void EigSolve(double *A, double *eig, double &l, int n)
{
	// A is nxn
	int i, j, it;
	bool repeat;
	double eig_tmp[n], l_n[1], l_d[1];

	// initialize eigenvector
	eig[0] = 1.0 / sqrt((double)n);
	for (i = 1; i < n; i++)
	{
		eig[i] = eig[0];
	}

	it = 0;
	do {
		it++;

		// calculate the matrix-by-vector product A*eig
		MatrixMultiply(A, eig, eig_tmp, n, n, 1);

		// calculate the length of the resultant vector
		Normalize(eig_tmp, n);

		// normalize b to unit vector for next iteration
		repeat = false;
		for (i = 0; i < n; i++)
		{
			if (fabs(eig[i] - eig_tmp[i]) > 0.0001 && it<100)
			{
				repeat = true;
			}
			eig[i] = eig_tmp[i];
		}
	} while (repeat);

	MatrixMultiply(A, eig, eig_tmp, n, n, 1);

	MatrixMultiply(eig, eig_tmp, l_n, 1, n, 1);

	MatrixMultiply(eig, eig, l_d, 1, n, 1);
	
	l = l_n[0] / l_d[0];

}

void Normalize(double *w, int n)
{
	// Vector normalization of w

	int i;
	double norm;

	norm = 0.0;
	for (i = 0; i < n; i++)
	{
		norm += pow(w[i],2.0);
	}
	norm = sqrt(norm);

	for (i = 0; i < n; i++)
	{
		w[i] /= norm;
	}
}


void GeneralizedSelfAdjointEigSolver(double* A, double* B, double* w, int n)//Sb, Sw, w, n
{
	double C[n*n], w_tmp[n], l, D[n];

	CholeskyDecomp(B, n); // L where L * L^T = B
	//LDLDecomp(B, D, n); //where B = L * D * L', replace B with L, D (vector) with D --- LDLDecomp fixed 04/07/2017 but untested

	LowerTriangularInverse(B, n); // L^-1

	MatrixMultiply(B, A, C, n, n, n); //L^-1 * A, output C

	MatrixTransposeSquare(B, n); // (L^-1)^T

	MatrixMultiply(C, B, A, n, n, n); //L^-1 * A * (L^-1)^T, output A

	EigSolve(A, w_tmp, l, n);

	MatrixMultiply(B, w_tmp, w, n, n, 1); // eigvecs = (L^-1)^T * eigvecs

	Normalize(w, n);
}

void PCA(double *S, int &nk, int n)
{
	// given covariance matrix S, produce first nk principal components
	// that explain > 99.9% of variance

	int i, j, k;
	double S_tmp[n*n], l, s, w[n], W[n*n], WW[n*n];

	// initialize S_tmp
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			S_tmp[i*n + j] = S[i*n + j];
			S[i*n + j] = 0.0;
		}
	}

	nk = 0;
	s = 0.0;

	EigSolve(S_tmp, w, l, n); // get first eigenvec/eigenval
	do
	{
		// fill nkth column of S with nkth principal component
		for (i = 0; i < n; i++)
		{
			S[i*n + nk] = w[i];
		}

		nk++;

		// prepare S_tmp for next principal component
		MatrixMultiply(w, w, W, n, 1, n);

		MatrixMultiply(W, W, WW, n, n, n);

		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				S_tmp[i*n + j] += l*WW[i*n + j] - 2*l*W[i*n + j];
			}
		}

		// check if enough variance is already explained by nk components, by getting nk+1 eigenvalue
		s += l;
		EigSolve(S_tmp, w, l, n);

	} while ((nk < n) && (s / (s + (n-nk)*l) <= 0.997));
}

void LDA(double *mu1, double *mu2, double *S1, double *S2, double *w, int n1, int n2, int n)
{
	// DOES NOT change S1 and S2 (covariance matrices of class 1 and 2)
	// useful output is w
	// utilizes a PCA step to ensure nondegenerate matrix !!if enabled below!!

	int i, j, k;
	int ntot = n1 + n2;
	double p1 = (double)n1 / (double)ntot;
	double p2 = (double)n2 / (double)ntot;
	double d, mu[n], Sw[n*n], Sb[n*n];

	// calculate overall mean
	for (j = 0; j < n; j++) {
		mu[j] = p1*mu1[j] + p2*mu2[j];
	}

	// calculate within-class scattering matrix
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			Sw[i*n + j] = p1*S1[i*n + j] + p2*S2[i*n + j];
		}
	}

	// calculate between-class scattering matrix
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			Sb[i*n + j] = p1*mu1[i] * mu1[j] + p2*mu2[i] * mu2[j] - mu[i] * mu[j];
		}
	}
	
	// check if Sw is rank deficient: if it's not use lda, if it is run PCA
	// to enable PCA step, replace "if (true)" with the 2 lines below 
	
	//det(Sw, d, n);
	//if (d > .0000000001) // 10^-10
	if (true)
	{
		// run lda
		GeneralizedSelfAdjointEigSolver(Sb, Sw, w, n);
	}
	else 
	{
		// run PCA
		double S[n*n];

		// calculate total covariance matrix (S)
		for (i = 0; i < n; i++)
		{
			for (j = 0; j < n; j++)
			{
				S[i*n + j] = Sw[i*n + j] + Sb[i*n + j];
			}
		}

		int nk = 0;
		PCA(S, nk, n);

		double K[n*nk], KT[nk*n];

		for (i = 0; i < n; i++)
		{
			for (j = 0; j < nk; j++)
			{
				K[i*nk + j] = S[i*n + j];
			}
		}

		MatrixTranspose(K, KT, n, nk);

		// project data to principal components before running lda
		double Sk_tmp[n*nk], Swk[nk*nk], Sbk[nk*nk], wk[nk];

		MatrixMultiply(Sw, K, Sk_tmp, n, n, nk); //Swk
		MatrixMultiply(KT, Sk_tmp, Swk, nk, n, nk); //Swk

		MatrixMultiply(Sb, K, Sk_tmp, n, n, nk); //Sbk
		MatrixMultiply(KT, Sk_tmp, Sbk, nk, n, nk); //Sbk

		// run lda
		GeneralizedSelfAdjointEigSolver(Sbk, Swk, wk, nk);

		// find projection from original space that incorporates 
		// principal components and lda vector
		MatrixMultiply(K, wk, w, n, nk, 1);

	}

}
