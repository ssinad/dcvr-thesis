#include <iostream>
#include <ilcplex/ilocplex.h>
#include <numeric> // std::iota
#include <algorithm>
#include <chrono>
#include <ctime>
#include "BasisEvent.hpp"
#include "Penalties.h"
#include "orienteering.h"
#include "heuristics.hpp"
#include "dcvr.hpp"

ILOSTLBEGIN
using namespace std::chrono;



// https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
vector<size_t> sort_indexes(const vector<T> &v)
{

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}

void get_event_count(std::vector<int> &s1, std::vector<int> &s2, std::unordered_map<int, int> &event_count)
{
    std::vector<int> tmp;
    tmp.clear();

    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(tmp, tmp.end()));

    for (int n : tmp)
    {
        if (event_count.find(n) == event_count.end())
        {
            event_count[n] = 0;
        }
        event_count[n] += 1;
    }
}

void get_event_log(std::vector<int> &old_basis, std::vector<int> &new_basis, int iteration, std::unordered_map<int, std::vector<BasisEvent>> &event_log, std::ofstream &out)
{
    std::vector<int> tmp;
    tmp.clear();

    std::set_difference(old_basis.begin(), old_basis.end(), new_basis.begin(), new_basis.end(), std::inserter(tmp, tmp.end()));

    for (int n : tmp)
    {
        BasisEvent be1;
        be1.iteration = iteration;
        be1.event_type = BasisEventType::Exit;
        be1.variable = n;
        if (event_log.find(n) == event_log.end())
        {
            event_log[n];
        }

        event_log[n].push_back(be1);
    }

    tmp.clear();

    std::set_difference(new_basis.begin(), new_basis.end(), old_basis.begin(), old_basis.end(), std::inserter(tmp, tmp.end()));

    for (int n : tmp)
    {
        BasisEvent be2;
        be2.iteration = iteration;
        be2.event_type = BasisEventType::Entry;
        be2.variable = n;
        if (event_log.find(n) == event_log.end())
        {
            event_log[n];
        }

        event_log[n].push_back(be2);
    }

    out << "{ ";
    for (auto &kv : event_log)
    {
        out << "\"" << kv.first << "\": [";
        for (BasisEvent be : kv.second)
        {
            out << "{ \"iteration\":" << be.iteration << ", \"type\":";
            if (be.event_type == BasisEventType::Entry)
            {
                out << "\"entry\"";
            }
            else
            {
                out << "\"exit\"";
            }
            out << "}, ";
        }
        out << "], ";
    }
    out << "}, " << std::endl;
    out.flush();
}

penalty_t best_upper;
std::pair<bool, Path> path_generation_orienteering(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    std::unordered_map<Node, OrienteeringInfo> info;
    auto start_orienteering = clock();
    auto tmp_p = orienteering(vertices, root_node, distances, rewards, distance_limit_D, info);
    auto end_orienteering = clock();
    
    auto orienteering_duration = 1000.0 * (end_orienteering - start_orienteering) / CLOCKS_PER_SEC;
    std::cout << "Orienteering runtime: " << orienteering_duration << " ms" << std::endl;
    Path p = post_process(tmp_p.second, rewards, distances, root_node, distance_limit_D);
    std::cout << "Upper Bounds:" << std::endl;
    best_upper = 0;
    for (auto &kv : info)
    {
        std::cout << kv.second.upper_bound << ", ";
        if (isfinite(kv.second.upper_bound) && kv.second.upper_bound > best_upper)
            best_upper = kv.second.upper_bound;
    }
    std::cout << std::endl;
    Node best_node = tmp_p.first;
    return std::pair<bool, Path>(true, p);
}

std::unordered_map<int, PathWrapper> dcvr_fractional(const Vertices &vertices, const Node &root_node, const Matrix &distances, distance_t distance_limit_D)
{
    // Assert all clients are from D distance fromm root
    std::unordered_map<int, PathWrapper> paths;
    IloEnv env;
    IloModel model(env);
    IloNumVarArray path_var(env);
    IloRangeArray con(env);
    IloObjective obj = IloMinimize(env);
    IloCplex::BasisStatusArray cstat(env);
    std::vector<Path> tmp_paths;

    double apx;
    for (Node v : vertices)
    {
        con.add(IloRange(env, 1.0, IloInfinity));
    }
    model.add(con);
    model.add(obj);

    penalty_t best_lb = 0;
    int path_cnt = 0;
    for (Node v : vertices)
    {
        if (distances[root_node][path_cnt] - distance_limit_D > DISTANCE_EPSILON)
        {
            env.out() << "No solution possible" << endl;
            return paths; // No solutions
        }
        path_var.add(IloNumVar(env, 0.0, IloInfinity));
        char path_var_name[10];
        sprintf(path_var_name, "X_p_%d", path_cnt);
        path_var[path_cnt].setName(path_var_name);
        obj.setLinearCoef(path_var[path_cnt], 1.0);
        con[root_node].setLinearCoef(path_var[path_cnt], 1.0);
        con[path_cnt].setLinearCoef(path_var[path_cnt], 1.0);
        path_cnt++;
    }

    IloCplex solver(model);
    // Use the following to make Cplex less verbose
#ifdef NDEBUG
    solver.setOut(env.getNullStream());
#endif
    Rewards rewards(vertices.size());
    IloNumArray vals(env);
    Path p;
    int iter_cnt = 0;
    bool apx_called = false;
    do
    {
        std::cout << "Iteration #" << iter_cnt + 1 << std::endl;
        std::cout << "---------------------------" << std::endl;
        if (!solver.solve())
        {
            env.error() << "Failed to optimize LP" << endl;
            throw(-1);
        }

        solver.getDuals(vals, con);
        
        for (int cnt = 0; cnt < vertices.size(); ++cnt)
        {
            rewards[cnt] = vals[cnt];
        }
        

        auto b_path = path_generation_heuristic_1(vertices, rewards, distances, distance_limit_D, root_node);
        if (b_path.first)
        {
            p = b_path.second;
        }
        else
        {
            b_path = path_generation_heuristic_2(vertices, rewards, distances, distance_limit_D, root_node);
            if (b_path.first)
            {
                p = b_path.second;
            }
            else
            {
                b_path = path_generation_heuristic_3(vertices, rewards, distances, distance_limit_D, root_node);
                if (b_path.first)
                {
                    p = b_path.second;
                }
                else
                {
                    
                    b_path = path_generation_orienteering(vertices, rewards, distances, distance_limit_D, root_node);
                    p = b_path.second;
                    apx_called = true;
                    std::cout << "Path generated by orienteering" << std::endl;
                }
            }
        }

        // TODO check path distance


        reward_t p_reward = get_path_reward(p, rewards);
        
        assert(get_path_distance(p, distances, root_node) + DISTANCE_EPSILON <= distance_limit_D);
        std::cout << "Objective function value: " << solver.getObjValue() << std::endl;

        if (p_reward - REWARD_EPSILON <= 1)
        {
            break;
        }

        if (apx_called && best_upper != 0)
        {
            apx = p_reward / best_upper;
            std::cout << "Path approx. guarantee: " << apx << std::endl;
            if (apx < 1.0 / 3.0)
            {
                std::cerr << "Bad approx. guarantee!" << std::endl;
                exit(1);
            }

            double this_lb = solver.getObjValue() / best_upper;
            std::cout << "Lower bound on LP value: " << this_lb << std::endl;
            if (this_lb > best_lb)
                best_lb = this_lb;
        }
        
        std::cout << "Best upper bound: " << best_upper << std::endl;
        std::cout << "Best bound on LP value: " << best_lb << std::endl;
        std::cout << "Path reward: " << p_reward << std::endl;
        std::cout << "Reward residue: " << p_reward - 1 << std::endl;

        path_var.add(IloNumVar(env, 0.0, IloInfinity));
        char path_var_name[10];
        sprintf(path_var_name, "X_p_%d", path_cnt);
        path_var[path_cnt].setName(path_var_name);
        obj.setLinearCoef(path_var[path_cnt], 1.0);
        for (Node n : p)
        {
            con[n].setLinearCoef(path_var[path_cnt], 1.0);
        }
        PathWrapper pw;
        pw.path = p;

        paths[path_cnt] = pw;
        path_cnt++;

        
        iter_cnt++;
        std::cout << std::endl;
    } while (get_path_reward(p, rewards) > 1 + REWARD_EPSILON);
    
    solver.getBasisStatuses(cstat, path_var);
    solver.getValues(vals, path_var);
    env.out() << "Values are    " << vals << std::endl;
    for (int cnt = 0; cnt < path_cnt; ++cnt)
    {
        paths[cnt].value = vals[cnt];
        if (cstat[cnt] == IloCplex::Basic)
        {
            paths[cnt].is_basic = true;
        }
        else
        {
            paths[cnt].is_basic = false;
        }
    }
    return paths;
}

std::list<Path> dcvr(const Vertices &vertices, const Node &root_node, const Matrix &distances, distance_t distance_limit_D, Dcvr_mode mode)
{
    std::unordered_map<int, PathWrapper> path_map;
    std::vector<Path> nz_paths;
    std::list<Path> paths;
    path_map = dcvr_fractional(vertices, root_node, distances, distance_limit_D);

    IloEnv env;
    IloModel model(env);
    IloIntVarArray path_var(env);
    IloRangeArray con(env);
    IloObjective obj = IloMinimize(env);

    for (Node v : vertices)
    {
        // if (v == root_node)
        //     continue;
        con.add(IloRange(env, 1.0, IloInfinity));
    }
    model.add(con);
    model.add(obj);
    int path_cnt = 0;
    for (auto &kv : path_map)
    {
        if (kv.second.value <= 0 && mode == Dcvr_mode::NON_ZEROS)
            continue;
        if (!kv.second.is_basic && mode == Dcvr_mode::BASIC)
            continue;
        Path p = kv.second.path;

        nz_paths.push_back(p);
        path_var.add(IloIntVar(env, 0, IloInfinity));
        char path_var_name[10];
        sprintf(path_var_name, "X_p_%d", path_cnt);
        path_var[path_cnt].setName(path_var_name);
        obj.setLinearCoef(path_var[path_cnt], 1);

        for (Node n : p)
        {
            con[n].setLinearCoef(path_var[path_cnt], 1);
        }

        path_cnt++;
    }
    std::cout << "Number of variables in MILP: " << path_cnt << std::endl;
    IloCplex solver(model);
    if (!solver.solve())
    {
        env.error() << "Failed to optimize LP" << endl;
        throw(-1);
    }
    IloNumArray vals(env);
    solver.getValues(vals, path_var);
    env.out() << "Integer values are " << vals << std::endl;
    for (int cnt = 0; cnt < path_cnt; cnt++)
    {
        if (vals[cnt] != 0)
        {
            paths.push_back(nz_paths[cnt]);
        }
    }
    env.out() << "Final MILP objective function value: " << solver.getObjValue() << std::endl;
    return paths;
}
