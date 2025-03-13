#pragma once
#include "pinocchio/container/boost-container-limits.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <unsupported/Eigen/CXX11/Tensor>

#include <map>
#include <iterator>
#include <sstream>

#include <boost/variant.hpp>
#include <boost/variant/static_visitor.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/type_traits.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/serialization/nvp.hpp>