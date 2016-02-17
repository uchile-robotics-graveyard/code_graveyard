/*
 *   Copyright (c) 2007 John Weaver
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#if !defined(_MUNKRES_H_)
#define _MUNKRES_H_
#include <list>
#include <utility>

namespace munkres {
template<typename FloatMatrix, typename IntMatrix>
class Munkres {
public:
    void solve(FloatMatrix &m);
private:
    static const int NORMAL = 0;
    static const int STAR = 1;
    static const int PRIME = 2; 
    inline bool find_uncovered_in_matrix(double,int&,int&);
    inline bool pair_in_list(const std::pair<int,int> &, const std::list<std::pair<int,int> > &);
    int step1(void);
    int step2(void);
    int step3(void);
    int step4(void);
    int step5(void);
    int step6(void);
    IntMatrix mask_matrix;
    FloatMatrix matrix;
    bool *row_mask;
    bool *col_mask;
    int saverow, savecol;
};
}

// Possible matrix implementations
#include "matrix.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// Concepts for original matrix class
template<typename T>
inline int rows(const munkres::Matrix<T> &m) {return m.rows();};
template<typename T>
inline int cols(const munkres::Matrix<T> &m) {return m.columns();};
template<typename T>
inline void zero(munkres::Matrix<T> &m, size_t rows, size_t cols) { m.resize(rows, cols); }

// Concepts for eigen C++ matrices
template<typename T, int R, int C>
inline int rows(const Eigen::Matrix<T,R,C> &m) {return m.rows();};
template<typename T, int R, int C>
inline int cols(const Eigen::Matrix<T,R,C> &m) {return m.cols();};
template<typename T>
void zero(Eigen::Matrix<T,-1,-1> &m, size_t rows, size_t cols) { m = Eigen::Matrix<T,-1,-1>::Zero(rows, cols); }
template<typename T, int R, int C>
inline void zero(Eigen::Matrix<T,R,C> &m, size_t rows, size_t cols) { m = Eigen::Matrix<T,R,C>::Zero(); }

// Concetps for OpenCV matrices
inline int rows(const cv::Mat &m) {return m.rows;};
inline int cols(const cv::Mat &m) {return m.cols;};
template<typename T>
inline void zero(cv::Mat &m, size_t rows, size_t cols) { m = cv::Mat_<T>::zeros(rows, cols); }

// Include the code
#include "munkres.hpp"

#endif /* !defined(_MUNKRES_H_) */
