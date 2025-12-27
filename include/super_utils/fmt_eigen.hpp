// Copyright 2024 Yunfan REN
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef FMT_EIGEN_HPP
#define FMT_EIGEN_HPP
#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include <Eigen/Dense>
#include <cstring>
#include <iostream>
#include <regex>

template <typename MatrixType>
class MatrixFormatter : fmt::formatter<std::string>
{
private:
  int precision = 6;  // Default precision if not specified
public:
  // Parse the format string to extract the precision
  template <typename ParseContext>
  auto parse(ParseContext & ctx)
  {
    auto it = ctx.begin();
    auto end = ctx.end();
    // Check if precision is specified in the format string
    std::string input_str = std::string(it, end);
    std::regex re(R"(\.(\d+))");
    std::smatch match;
    if (std::regex_search(input_str, match, re)) {
      precision = std::stoi(match[1]);
    }
    while (it != end && *it != '}') it++;  // Skip to the end of the range
    // Return the iterator after parsing
    return it;
  }

  template <typename Scalar, typename FormatContext>
  typename std::enable_if<std::is_floating_point<Scalar>::value>::type format_element(
    Scalar value, FormatContext & ctx)
  {
    fmt::format_to(ctx.out(), "{:.{}f} ", value, precision);
  }

  template <typename Scalar, typename FormatContext>
  typename std::enable_if<std::is_integral<Scalar>::value>::type format_element(
    Scalar value, FormatContext & ctx)
  {
    fmt::format_to(ctx.out(), "{:d} ", value);
  }

  // Format the matrix with the specified precision
  template <typename FormatContext>
  auto format(const MatrixType & matrix, FormatContext & ctx)
  {
    for (int i = 0; i < matrix.rows(); i++) {
      for (int j = 0; j < matrix.cols(); j++) {
        format_element(matrix(i, j), ctx);
      }
      if (i != matrix.rows() - 1) {
        fmt::format_to(ctx.out(), "\n");
      }
    }
    return ctx.out();
  }
};

/// Specialize the formatter for Eigen::Matrix
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct fmt::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
: MatrixFormatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>
{
};

/// Specialize the formatter for Eigen::Transpose
template <typename MatrixType>
struct fmt::formatter<Eigen::Transpose<MatrixType>> : MatrixFormatter<Eigen::Transpose<MatrixType>>
{
};

/// Specialize the formatter for Eigen::Block
template <typename Scalar, int Rows, int Cols, int BlockRows, int BlockCols, bool InnerPanel>
struct fmt::formatter<
  Eigen::Block<Eigen::Matrix<Scalar, Rows, Cols>, BlockRows, BlockCols, InnerPanel>>
: MatrixFormatter<Eigen::Block<Eigen::Matrix<Scalar, Rows, Cols>, BlockRows, BlockCols, InnerPanel>>
{
};

/// Specialize the formatter for Eigen::Diagno
template <typename Scalar, int Rows, int Cols>
struct fmt::formatter<Eigen::Diagonal<Eigen::Matrix<Scalar, Rows, Cols>>>
: MatrixFormatter<Eigen::Diagonal<Eigen::Matrix<Scalar, Rows, Cols>>>
{
};

#endif
