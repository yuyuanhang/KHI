#ifndef MA_GRAPH_H
#define MA_GRAPH_H
#include <utility>
#include <vector>
#include <stack>
#include <chrono>
#include <random>
#include <queue>
#include <unordered_map>
#include <thread>
#include <unordered_set>
#include <limits>
#include <iomanip>
#include <atomic>
#include <string>
#include <fstream>
#include "data.h"
#include "kd_tree.h"
#include "timer.h"
#include "query.h"
#include <space_l2.h>
#include <BS_thread_pool.hpp>
#include <searcher.hpp>

using namespace std;
typedef pair<float, int> PFI;
typedef std::pair<float, std::pair<int, int>> PFII;

class MAGraph
{
public:
    Data* data;
    KDTree* kd_tree;
    // for distance computation
    hnswlib::L2Space *space;
    hnswlib::DISTFUNC<float> fstdistfunc_;
    void *dist_func_param_{nullptr};
    // for index construction
    int max_threads = 32;
    int M = 0;
    int ef_c = 0;
    bool inter_para = true;
    vector<vector<vector<PFI>>> edges;
    vector<vector<PFI>> reverse_edges;
    vector<size_t> visited_pool;
    vector<size_t> visited_tag;
    unordered_map<thread::id, int> thread_map;
    // for search
    vector<vector<vector<int>>> search_edges;
    size_t metric_distance_computations = 0;
    size_t metric_hops = 0;
    int prefetch_lines = 0;

    explicit MAGraph(Data* data, int M = 32, int ef_c = 100);
    ~MAGraph();
    float dis_compute(int v1, int v2) const;
    void set_threads(int max_threads = 32);
    void build_graph();
    void process_node(KDTree::node* u);
    void copy_first_child(KDTree::node* u);
    priority_queue<PFI> search_on_incomplete_graph(int level, int query_point,
        int ef, int query_k, const vector<int> &enter_points);
    void insert(int id, int level, int c_level, const vector<int> &processed_points);
    vector<PFI> PruneByHeuristic2(const vector<PFI> &old_list, const vector<PFI> &new_list);
    void insert_backward(vector<int> &processing_points, int level, float lb, int a_id);
    void print_progress_bar(int current, int total, int width = 50);
    void init_pool(BS::thread_pool<> &pool);
    void save(const string &prefix);
    void save_graph(const string &file);
    void load(const string &prefix);
    void load_graph(const string &file);

    void search(vector<int> &SearchEF, const string &save_file, int edge_limit, Query* q, int top_k);
    bool check_constraint(int id, vector<float> &range) const;
    vector<KDTree::node*> range_filter(vector<float> &range, int top_k);
    priority_queue<PFI> TopDown_nodeentries_search(const vector<KDTree::node*> &filtered_nodes, float* query_vec,
        int ef, int query_k, vector<float> &range, int edge_limit);
    vector<int> SelectEdge(int pid, vector<float> &range, int edge_limit, searcher::Bitset<uint64_t> &visited_set);

    void average_degree();
};

#endif // MA_GRAPH_H
