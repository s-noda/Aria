#include <iostream>
#include <complex>
#include <Eigen/Dense>

extern "C"{
  void calc_eigen(int n, double* mat, double* peigenval, double* neigenval, double* peigenvector, double* neigenvector){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n,n);
    for (int y=0 ; y<n ; y++ ){
      for ( int x=0; x<n; x++ ){
	A(y,x) = mat[x+y*n];
      }
    }
    // std::cout << A << std::endl ;
    //
    Eigen::EigenSolver<Eigen::MatrixXd> s(A);
    // Eigen::EigenSolver< std::complex<double> >::EigenvectorsType val = s.eigenvalues();
    // Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType vec = s.eigenvectors();
    for ( int i=0; i<n ; i++ ){
      std::complex<double> val = s.eigenvalues()(i);
      peigenval[i] = (double)val.real() ;
      neigenval[i] = (double)val.imag() ;
      //
      for ( int j=0; j<n ; j++ ){
	peigenvector[i*n+j] = s.eigenvectors().col(i)(j).real() ;
	neigenvector[i*n+j] = s.eigenvectors().col(i)(j).imag() ;
      }
    }
  }
}
