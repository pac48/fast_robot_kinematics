#include "chrono"
#include "iostream"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <immintrin.h>
#include <x86intrin.h>


enum OP {
  ROT,
  TRANS
};


//template<OP T>
//inline void func(Eigen::Vector<double, 16> &sum_matrix);
//
//template<>
//inline void func<OP::ROT>(Eigen::Vector<double, 16> &sum_matrix) {
////  sum_matrix.array() = sum_matrix.array().sqrt();
//  //    sum_matrix.array() = sum_matrix.array()+1;
////    sum_matrix.array() = sum_matrix.array()*1.1+.1;
//  sum_matrix.array() = sum_matrix.array().sin();
//}
//
//template<>
//inline void func<OP::TRANS>(Eigen::Vector<double, 16> &sum_matrix) {
////  sum_matrix.array() = sum_matrix.array().sqrt();
//  //    sum_matrix.array() = sum_matrix.array()+1;
//    sum_matrix.array() = sum_matrix.array()*1.1+.1;
////  sum_matrix.array() = sum_matrix.array().sin();
//}

//template<typename T>
//void func_wrap(Eigen::Vector<double, 16> &sum_matrix, T op);

//template<OP T>
//void func_wrap(Eigen::Vector<double, 16> &sum_matrix, T op){
//  printf("here!!\n");
//  func<T>(sum_matrix);
//}

template<OP ...T>
inline void FK(Eigen::Vector<double, 16> &sum_matrix) {
  ([&sum_matrix]() {
    if constexpr (T == OP::ROT) {
      sum_matrix.array() = sum_matrix.array().sqrt();
      sum_matrix.array() = sum_matrix.array().sin();
    }
    if constexpr (T == OP::TRANS) {
      sum_matrix.array() = sum_matrix.array()*.9 + 1;
    }
  }(), ...);
}


int main(int arc, char **argv) {

  constexpr int iterations = 128 * 128 * 128;
  Eigen::Vector<double, 16> sum_matrix;
  sum_matrix << .01, .02, .03, .04, .05, .06, .07, .08, .09, .010, .011, .012, .013, .014, .015, .016;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
    FK<OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::ROT, OP::TRANS>(sum_matrix);
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

//  for (auto &val: sum) {
//    std::cout << "final value: " << val << std::endl;
//  }
  for (auto &val: sum_matrix) {
    std::cout << "final value: " << val << std::endl;
  }

  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations * sum_matrix.size()) << " nanoseconds"
            << std::endl;

  return 0;
}
