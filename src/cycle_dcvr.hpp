#ifndef __DCVR__
#define __DCVR__
#include <unordered_map>
// #include <ilcplex/ilocplex.h>
#include "Matrix.h"
#include "Vertices.h"
#include "Path.h"
enum Dcvr_mode {NON_ZEROS, BASIC, EVERYTHING};
std::unordered_map<int, PathWrapper> cycle_dcvr_fractional(const Vertices &, const Node &, const Matrix &, distance_t );
std::list<Path> cycle_dcvr(const Vertices &, const Node &, const Matrix &, distance_t, Dcvr_mode mode = Dcvr_mode::BASIC);
#endif