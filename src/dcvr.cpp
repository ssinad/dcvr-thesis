#include <iostream>
#include <ilcplex/ilocplex.h>
#include <numeric> // std::iota
#include <algorithm>
#include <chrono>
#include <ctime>
#include "BasisEvent.hpp"
#include "Penalties.h"
#include "orienteering.h"
#include "dcvr.hpp"

ILOSTLBEGIN
using namespace std::chrono;

const reward_t REWARD_EPSILON = 1e-6;
const distance_t DISTANCE_EPSILON = 1e-6;
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

std::vector<Node> my_argsort(const Rewards &rewards)
{
    typedef std::pair<reward_t, Node> tmp_pair;
    std::vector<tmp_pair> pairs;
    for (Node cnt = 0; cnt < rewards.size(); cnt++)
    {
        pairs.push_back(tmp_pair(rewards[cnt], cnt));
    }
    std::sort(pairs.begin(), pairs.end(), [&](const tmp_pair &a, const tmp_pair &b) { return a.first > b.first; });

    std::vector<Node> args;
    for (auto &kv : pairs)
    {
        args.push_back(kv.second);
    }
    return args;
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

    // std::vector<int> tmp;
    // std::unordered_set<int>::iterator it, tb, s1b, s1e, s2b, s2e;
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

Path post_process(Path &path, const Rewards &rewards, const Matrix &distances, const Node &root_node, const distance_t distance_limit_D)
{
    // TODO argsort rewards
    std::vector<Node> args = my_argsort(rewards);
    std::unordered_set<Node> path_nodes;
    for (Node j : path)
    {
        path_nodes.insert(j);
    }
    // path_reward = get_path_reward(path, rewards);
    distance_t path_distance = get_path_distance(path, distances, root_node);

    Path new_path = path;
    for (Node i : args)
    {
        if (path_nodes.find(i) != path_nodes.end())
            continue;
        distance_t new_distance;
        Path::iterator current_node_iterator = new_path.begin();
        while (current_node_iterator != new_path.end())
        {
            Node current_node = *current_node_iterator;
            new_distance = path_distance + distances[current_node][i];
            Path::iterator next_node_iterator = std::next(current_node_iterator);
            if (next_node_iterator != new_path.end())
            {
                Node next_node = *next_node_iterator;
                new_distance += distances[i][next_node] - distances[current_node][next_node];
            }
            if (new_distance <= distance_limit_D)
            {
                new_path.insert(next_node_iterator, i);
                path_distance = new_distance;
                break;
            }
            ++current_node_iterator;
        }
    }
    return new_path;
}

std::pair<bool, Path> path_generation_heuristic_1(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path p;
    p.push_back(root_node);
    Path new_path = post_process(p, rewards, distances, root_node, distance_limit_D);
    if (new_path.size() == 1 || get_path_reward(new_path, rewards) - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, new_path);
    }
    else
    {
        std::cout << "Path generated by first heuristic" << std::endl;
        return std::pair<bool, Path>(true, new_path);
    }
}

std::pair<bool, Path> path_generation_heuristic_2(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path best_path;
    reward_t highest_reward = 0;
    for (Node i : vertices)
    {
        for (Node j : vertices)
        {
            if (i == j || i == root_node || j == root_node)
                continue;
            if (rewards[i] + rewards[j] - REWARD_EPSILON <= 1)
                continue;
            if (distances[root_node][i] + distances[i][j] > distance_limit_D)
                continue;
            Path initial_path;
            initial_path.push_back(root_node);
            initial_path.push_back(i);
            initial_path.push_back(j);
            Path improved_path = post_process(initial_path, rewards, distances, root_node, distance_limit_D);
            reward_t reward = get_path_reward(improved_path, rewards);
            if (reward > highest_reward)
            {
                best_path = improved_path;
                highest_reward = reward;
            }
        }
    }
    if (highest_reward - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, best_path);
    }
    else
    {
        std ::cout << "Path generated by second heuristic" << std::endl;
        return std::pair<bool, Path>(true, best_path);
    }
}

std::pair<bool, Path> path_generation_heuristic_3(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path p;
    std::unordered_set<Node> path_nodes;
    p.push_back(root_node);
    path_nodes.insert(root_node);
    Node previous_node = root_node;
    distance_t path_distance = 0, new_distance;
    bool can_add_nodes = true;
    do
    {
        can_add_nodes = false;
        distance_t max_reward_to_distance_ratio = 0;
        Node argmax = root_node;
        for (Node i : vertices)
        {
            if (path_nodes.find(i) != path_nodes.end())
                continue;
            new_distance = path_distance + distances[previous_node][i];
            if (new_distance > distance_limit_D)
                continue;
            can_add_nodes = true;
            // Pay attention to the datatypes when doing the division
            if (distances[previous_node][i] == 0){
                argmax = i;
                break;
            }
            else{
                if (max_reward_to_distance_ratio <= rewards[i] / distances[previous_node][i])
                {
                    max_reward_to_distance_ratio = rewards[i] / distances[previous_node][i];
                    argmax = i;
                }
            }
        }
        if (can_add_nodes)
        {
            p.push_back(argmax);
            path_nodes.insert(argmax);
            path_distance += distances[previous_node][argmax];
            previous_node = argmax;
        }
    } while (can_add_nodes);

    if (p.size() == 1 || get_path_reward(p, rewards) - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, p);
    }
    else
    {
        std::cout << "Path generated by third heuristic" << std::endl;
        return std::pair<bool, Path>(true, p);
    }
}

penalty_t best_upper;
std::pair<bool, Path> path_generation_orienteering(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    std::unordered_map<Node, penalty_t> upper_bounds;
    // #ifndef NDEBUG
    auto start_orienteering = clock();
    // #endif
    auto tmp_p = orienteering(vertices, root_node, distances, rewards, distance_limit_D, upper_bounds);
    // #ifndef NDEBUG
    auto end_orienteering = clock();
    
    auto orienteering_duration = 1000.0 * (end_orienteering - start_orienteering) / CLOCKS_PER_SEC;
    std::cout << "Orienteering runtime: " << orienteering_duration << " ms" << std::endl;
    Path p = post_process(tmp_p.second, rewards, distances, root_node, distance_limit_D);
    std::cout << "Upper Bounds:" << std::endl;
    best_upper = 0;
    for (auto &kv : upper_bounds)
    {
        std::cout << kv.second << ", ";
        if (isfinite(kv.second) && kv.second > best_upper)
            best_upper = kv.second;
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
#ifndef NDEBUG
    std::unordered_map<int, int> event_count;
    std::unordered_map<int, std::vector<BasisEvent>> event_log;
#endif
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
        if (distances[root_node][path_cnt] > distance_limit_D)
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
#ifndef NDEBUG
        event_count[path_cnt] = 0;
        event_log[path_cnt];
#endif
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
#ifndef NDEBUG
    std::vector<int> old_basis, new_basis;

    std::ofstream profile("profile_activity.json");
    std::ofstream logger("log.json");
    profile << "[ ";
    logger << "[ ";
#endif
    int iter_cnt = 0;
    bool apx_called = false;
    do
    {
        // #ifndef NDEBUG
        std::cout << "Iteration #" << iter_cnt + 1 << std::endl;
        std::cout << "---------------------------" << std::endl;
        // #endif
        if (!solver.solve())
        {
            env.error() << "Failed to optimize LP" << endl;
            throw(-1);
        }

        solver.getDuals(vals, con);
        // TODO What should the first path be?
        // env.out() << "Rewards         = " << vals << std::endl;

        // rewards.clear();
        for (int cnt = 0; cnt < vertices.size(); ++cnt)
        {
            rewards[cnt] = vals[cnt];
        }


#ifndef NDEBUG
        // IloCplex::BasisStatusArray cstat(env);
        solver.getBasisStatuses(cstat, path_var);

        // env.out() <<"Number of rows: "<<model.getNrows() << endl << "Basis statuses  = " << cstat << endl;

        old_basis = new_basis;
        new_basis.clear();
        for (int cnt = 0; cnt < path_cnt; ++cnt)
        {
            if (cstat[cnt] == IloCplex::Basic)
            {
                new_basis.push_back(cnt);
            }
        }

        get_event_count(old_basis, new_basis, event_count);
        get_event_count(new_basis, old_basis, event_count);
        profile << "{ ";
        for (auto &kv : event_count)
        {
            profile << "\"" << kv.first << "\": " << kv.second << ", ";
        }
        profile << "}, \n";
        profile.flush();

        get_event_log(old_basis, new_basis, iter_cnt, event_log, logger);
#endif
        

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
                    // std::cout << "Orienteering called" << std::endl;
                    // Path tmp_p = orienteering(vertices, root_node, distances, rewards, distance_limit_D);
                    // p = post_process(tmp_p, rewards, distances, root_node, distance_limit_D);
                    b_path = path_generation_orienteering(vertices, rewards, distances, distance_limit_D, root_node);
                    p = b_path.second;
                    // #ifndef NDEBUG
                    apx_called = true;
                    std::cout << "Path generated by orienteering" << std::endl;
                    // #endif
                }
            }
        }

        // TODO check path distance


        reward_t p_reward = get_path_reward(p, rewards);
        
        assert(get_path_distance(p, distances, root_node) <= distance_limit_D);
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
#ifndef NDEBUG
        event_count[path_cnt] = 0;
        event_log[path_cnt];
#endif
        char path_var_name[10];
        sprintf(path_var_name, "X_p_%d", path_cnt);
        path_var[path_cnt].setName(path_var_name);
        obj.setLinearCoef(path_var[path_cnt], 1.0);
        for (Node n : p)
        {
            con[n].setLinearCoef(path_var[path_cnt], 1.0);
            // std::cout << n << " ";
        }
        // env.out() << std::endl << "Path added" << std::endl;
        PathWrapper pw;
        pw.path = p;

        // paths.push_back(p);
        paths[path_cnt] = pw;
        path_cnt++;

        // profile << "\"iteration\": " << iter_cnt << ", " ;

        // TODO check path distance
        // assert (get_path_distance(p, distances, root_node) > distance_limit_D);
        iter_cnt++;
        // #ifndef NDEBUG
        std::cout << std::endl;
        // #endif
    } while (get_path_reward(p, rewards) > 1 + REWARD_EPSILON);
    
    solver.getBasisStatuses(cstat, path_var);
    #ifndef NDEBUG
    env.out() << "Number of rows: " << solver.getNrows() << " Number of columns: " << solver.getNcols() << std::endl;
    env.out() << "Basis statuses  = " << cstat << endl;
    solver.getSlacks(vals, con);
    env.out() << "Slack variables are    " << vals << std::endl;
    #endif
    solver.getValues(vals, path_var);
    env.out() << "Values are    " << vals << std::endl;
    for (int cnt = 0; cnt < path_cnt; ++cnt)
    {
        paths[cnt].value = vals[cnt];
        // env.out() << "Basis statuses  = " << cstat << endl;
        if (cstat[cnt] == IloCplex::Basic)
        {
            paths[cnt].is_basic = true;
        }
        else
        {
            paths[cnt].is_basic = false;
        }
        // values[cnt] = vals[cnt];
    }
    return paths;
}

std::list<Path> dcvr(const Vertices &vertices, const Node &root_node, const Matrix &distances, distance_t distance_limit_D, Dcvr_mode mode)
{
    // std::unordered_map<int, Path> path_map;
    // std::unordered_map<int, double> value_map;
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
