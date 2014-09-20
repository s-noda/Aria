#include <iostream>
#include <complex>
#include <Eigen/Dense>
#include <Eigen/LU>

extern "C"{
  void calc_eigen(int n, double* mat, double* peigenval, double* neigenval, double* peigenvector, double* neigenvector){
    Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(mat,n,n);
    Eigen::EigenSolver<Eigen::MatrixXd> s(A);
    for ( int i=0; i<n ; i++ ){
      std::complex<double> val = s.eigenvalues()(i);
      peigenval[i] = (double)val.real() ;
      neigenval[i] = (double)val.imag() ;
      for ( int j=0; j<n ; j++ ){
	peigenvector[i*n+j] = s.eigenvectors().col(i)(j).real() ;
	neigenvector[i*n+j] = s.eigenvectors().col(i)(j).imag() ;
      }
    }
  }
}

extern "C"{
  void calc_inverse_matrix(int n, double* mat, double* ret){
    Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(mat,n,n);
    Eigen::Map<Eigen::MatrixXd>(ret,n,n) = A.inverse();
  }
}

extern "C"{
  double calc_determinant(int n, double* mat){
    Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(mat,n,n);
    double ret ;
    ret = A.determinant();
    return ret;
  }
}
