// #include <vector>
// TODO Define hash function for pairs so that we can replace this with hashset.
#include <set>
// #define MIN(A, B) ((A < B) ? A : B)
// Which of the following is necessary?
// #include <list>
#include <unordered_set>
// #include <map>
#include <unordered_map>
#include <stack> // Mostly for DFS
#include <string>
// #include <queue>
// #include <iostream>
// #include <array>
#include <assert.h>
// #include "PcaReader.h"
// #include "Arborescence.h"
#include "iterPCA.hpp"
#include "Our_Graph.h"

// class Node{
//
// };

typedef std::pair<Node, Node> Edge;

bool operator==(Edge const & a, Edge const & b){
    // TODO Directed or undirected?
    return (a.first == b.first && a.second == b.second);
}

struct EdgeHasher
{
  std::size_t operator()(const Edge& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:
      Node k1 = k.first;
      Node k2 = k.second;


      return (k1 + k2) * (k1 + k2 + 1) / 2 + k2;
    }
};

const double EPS = 1e-8;
// What should the types be? Use typedef so you can change 'em later
// These two should preferably be the same
// TODO Epsilon check

// V: list of nodes (including the root). can be as simple as a range 0 to |V| - 1
// pi(v) for every vertex v (even the root r)
// Shouldn't we copy penalties as well?
// Does vertices shrink in size?
// Arborescence.. parent dictionary or set?

// NOTE: The costs matrix and the penalties vector do not change in size.
// TODO: Output Theta as a single penalty
Arborescence iterPCA_with_check(Vertices &vertices, Matrix &costs, Penalties &penalties, penalty_t &sum_theta, Node last_node, const Node &root_node){
    assert (vertices.size() * 2 == penalties.size());
    return iterPCA(vertices, costs, penalties, sum_theta, last_node, root_node);
}
Arborescence iterPCA(Vertices &vertices, Matrix &costs, Penalties &penalties, penalty_t &sum_theta, Node last_node, const Node &root_node)
{

    if (vertices.size() == 1) // It only includes the root
    {
        return Arborescence();
    }

    penalty_t theta;
    for (Node v : vertices)
    {
        if (v == root_node)
            continue;
        distance_t minimum_cost = costs[root_node][v];
        for (Node tmp_node : vertices)
        {
            if (tmp_node == v)
                continue;
            if (minimum_cost > costs[tmp_node][v])
            {
                minimum_cost = costs[tmp_node][v];
            }
        }

        theta = (minimum_cost < penalties[v]) ? minimum_cost : penalties[v];
        sum_theta += theta;
        penalties[v] -= theta;

        // ZF: should start from 0
        // for (Node tmp_node = 1; tmp_node < matrix_size; tmp_node++)
        for (Node tmp_node : vertices)
        {
            costs[tmp_node][v] -= theta;
        }
    }
    // Major case 1

    // Dijkstra? No, modified DFS
    // Search tree? -> DFS
    Arborescence tmp_arb;
    std::stack<Node> container;
    std::unordered_set<Node> visited_so_far;
    container.push(root_node);
    while (container.size() != 0)
    {
        Node _node = container.top();
        container.pop();
        if (visited_so_far.find(_node) == visited_so_far.end())
        {
            visited_so_far.insert(_node);
            for (Node _next : vertices)
            {
                if (tmp_arb.find(_next) != tmp_arb.end()) continue;

                // Epsilon check Not equal to 0 but some threshold
                if (-EPS < costs[_node][_next] && costs[_node][_next] < EPS)
                { // E0
                    container.push(_next);
                    tmp_arb[_next] = _node;
                }
            }
        }
    }

    // They both contain the root
    if (vertices.size() == visited_so_far.size())
    {
        return tmp_arb;
    }

    // Major case 2
    // bool zero_penalty_flag = false;

    // ZF: range-based loop instead
    // for (Node _next : vertices) ???
    // for (Node v = 1; v < matrix_size; v++)
    // TODO Should it include the root?
    for (Node v : vertices)
    {
        // Epsilon check
        if (v == root_node)
            continue;
        if (-EPS < penalties[v] && penalties[v] < EPS)
        {
            // std::cout << "Major case 2: zero-penalty node\n";
            std::unordered_set<Edge, EdgeHasher> A;
            Arborescence arb_T;
            // ZF: range over vertices in these two loops as well?
            // the ranges should include the root
            // for (Node u = 1; u < matrix_size - 1; u++)
            for (Node u : vertices)
            {
                // for (Node w = u + 1; w < matrix_size; w++)
                if (u == v)
                    continue;
                for (Node w : vertices)
                {
                    if (u == w || w == v)
                        continue;
                    if (costs[u][w] > costs[u][v] + costs[v][w])
                    {
                        costs[u][w] = costs[u][v] + costs[v][w];
                        A.insert(Edge(u, w));
                    }
                }
            }
            vertices.erase(v);
            arb_T = iterPCA(vertices, costs, penalties, sum_theta, last_node, root_node);
            vertices.insert(v);
            // TODO optimize intersection
            bool empty_intersection = true;
            Graph g;
            bool is_functional = false;
            for (std::pair<Node, Node> kv : arb_T)
            {
                Node _u = kv.second, _w = kv.first;
                Edge uw(_u, _w);

                if (A.find(uw) == A.end())
                {
                    g[_u].insert(_w);
                }
                else
                {
                    is_functional = true;
                    empty_intersection = false;
                    g[_u].insert(v);
                    g[v].insert(_w);
                }
            }

            if (empty_intersection)
            {
                return arb_T;
            }
            else
            {
                Arborescence tmp_arb;
                std::stack<Node> container;
                std::unordered_set<Node> visited_so_far;
                container.push(root_node);
                while (container.size() != 0)
                {
                    Node _node = container.top();
                    container.pop();
                    if (visited_so_far.find(_node) == visited_so_far.end())
                    {
                        visited_so_far.insert(_node);
                        for (Node _next : g[_node])
                        {
                            if (tmp_arb.find(_next) != tmp_arb.end()) continue;
                            container.push(_next);
                            tmp_arb[_next] = _node;
                        }
                    }
                }
                return tmp_arb;
            }
        }
    }
    // Major case 3

    // std::cout << "Major case 3: zero-cost cycle\n";

    // Pick an unreachable node
    Node unreachable = -1; // initialize to -1
    for (Node _ : vertices)
    {
        // ZF: visited_so_far.find(_) ???
        if (visited_so_far.find(_) == visited_so_far.end())
        {
            unreachable = _;
            break;
        }
    }

    // ZF: Maybe add an assert showing that unreachable != -1
    // TODO move all of asserts out of the source code
    assert(unreachable != -1);

    // Form the cycle_Z
    std::unordered_set<Node> cycle_Z;
    std ::unordered_map<Node, Node> cycle_map;
    // Does this capture nodes other than the those of the cycle as well?
    // Phase 1
    Node vi = unreachable;
    while (cycle_map.find(vi) == cycle_map.end())
    {
        // cycle_Z.insert(vi);
        for (Node _ : vertices)
        {
            // Epsilon check
            if (_ != vi && -EPS < costs[_][vi] && costs[_][vi] < EPS)
            {
                cycle_map[vi] = _;
                vi = _;
                break;
            }
        }
        // ZF: maybe assert that vi is in cycle_map now
    }
    // TODO: I think this should be outside the while loop
    // assert(cycle_map.find(vi) != cycle_map.end());
    // TODO: What should the ancestor of the common node be?

    // Phase 2
    Node common_node = vi;
    cycle_Z.insert(common_node);
    vi = cycle_map[common_node];
    while (vi != common_node)
    {
        cycle_Z.insert(vi);
        vi = cycle_map[vi];
    }

    last_node++;
    penalty_t last_node_penalty = 0;
    for (Node _ : cycle_Z)
    {
        last_node_penalty += penalties[_];
    }
    assert(last_node < penalties.size());

    penalties[last_node] = last_node_penalty;

    // Copy constructor
    Vertices new_vertices = vertices;
    // new_vertices.insert(root_node);
    for (Node _ : cycle_Z)
    {
        new_vertices.erase(_);
    }

    // Node min_cost_Zw_node, min_cost_wZ_node;
    // Edge min_cost_incoming_edge, min_cost_outgoing_edge;
    // Outgoing and incoming are with respect to the cycle
    std::unordered_map<Node, Node> min_cost_incoming_edge, min_cost_outgoing_edge;
    for (Node w : new_vertices)
    {
        distance_t min_cost_wZ = costs[w][*cycle_Z.begin()];
        // min_cost_wZ_node = *cycle_Z.begin();
        min_cost_incoming_edge[w] = *cycle_Z.begin();
        distance_t min_cost_Zw = costs[*cycle_Z.begin()][w];
        min_cost_outgoing_edge[w] = *cycle_Z.begin();
        // min_cost_Zw_node = *cycle_Z.begin();
        for (Node _v : cycle_Z)
        {
            if (min_cost_wZ > costs[w][_v])
            {
                min_cost_wZ = costs[w][_v];
                // min_cost_wZ_node = _v;
                min_cost_incoming_edge[w] = _v;
            }
            if (min_cost_Zw > costs[_v][w])
            {
                min_cost_Zw = costs[_v][w];
                min_cost_outgoing_edge[w] = _v;
                // min_cost_Zw_node = _v;
            }
        }
        costs[w][last_node] = min_cost_wZ;
        costs[last_node][w] = min_cost_Zw;
    }
    // new_vertices.erase(root_node);
    new_vertices.insert(last_node);

    Node old_last = last_node;

    Arborescence arb_T = iterPCA(new_vertices, costs, penalties, sum_theta, last_node, root_node);
    if (arb_T.find(old_last) == arb_T.end())
    {
        return arb_T;
    }
    else
    {
        // ZF: find node, say enter_node, representing the edge (rb_[last_node], last_node)
        // arb_T[min_cost_outgoing_edge.second] = min_cost_outgoing_edge.first;
        Node k_, v_;
        for (std::pair<Node, Node> kv : arb_T)
        {
            k_ = kv.second;
            v_ = kv.first;
            if (k_ == old_last)
            {
                arb_T[v_] = min_cost_outgoing_edge[v_];
            }
        }

        for (Node _node : cycle_Z) // ZF: cycle_Z
        {
            // don't do it for kv.first == enter_node
            arb_T[_node] = cycle_map[_node];
        }
        // arb_T[min_cost_incoming_edge.second] = min_cost_incoming_edge.first;
        Node tmp_u = arb_T[old_last];
        arb_T[min_cost_incoming_edge[tmp_u]] = tmp_u;
        arb_T.erase(old_last);
        return arb_T;
        // arb_T[min_cost_wZ_node] = ;
    }
}

// TODO Add the check before iterPCA
// penalty is greater that r-t distance
// sizes are doubled
// last_node < penalties.size()
