#ifndef __ITERPCA__
#define __ITERPCA__
#include "Vertices.h"
#include "Arborescence.h"
#include "Matrix.h"
#include "Node.hpp"
#include "Penalties.h"

Arborescence iterPCA_with_check(Vertices &vertices, Matrix &costs, Penalties &penalties, penalty_t &sum_theta, Node last_node, const Node &root_node);
Arborescence iterPCA(Vertices &vertices, Matrix &costs, Penalties &penalties, penalty_t &sum_theta, Node last_node, const Node &root_node);

#endif