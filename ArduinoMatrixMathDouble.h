//ArduinoMatrixMathDouble.h
//#if defined(ARDUINO) && ARDUINO >= 100
//#include "Arduino.h"
//#else
//#include "WProgram.h"
//#endif
//#include "HardwareSerial.h" 
#ifndef ARDUINOMATRIXMATHDOUBLE_H_INCLUDED
#define ARDUINOMATRIXMATHDOUBLE_H_INCLUDED


void CholeskyDecomp(double *B, int n);
void LDLDecomp(double *B, double *D, int n);
void det(double *A, double d, int n);
void LowerTriangularInverse(double *L, int n);
void MatrixMultiply(double *A, double *B, double *C, int n, int m, int p);
void MatrixTranspose(double* A, double* AT, int n, int m);
void MatrixTransposeSquare(double *A, int n);
void EigSolve(double *A, double *eig, double &l, int n);
void Normalize(double *w, int n);
void GeneralizedSelfAdjointEigSolver(double* A, double* B, double* w, int n);
void PCA(double *S, int &nk, int n);
void LDA(double *mu1, double *mu2, double *S1, double *S2, double *w, int n1, int n2, int n);

#endif // ARDUINOMATRIXMATHDOUBLE_H_INCLUDED
