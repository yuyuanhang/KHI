#include "ma_graph.h"

MAGraph::MAGraph(Data* data, int M, int ef_c)
{
    this->M = M; this->ef_c = ef_c;
    this->data = data;
    kd_tree = new KDTree(data);

    space = new hnswlib::L2Space(data->d);
    fstdistfunc_ = space->get_dist_func();
    dist_func_param_ = space->get_dist_func_param();
    reverse_edges.resize(data->n);
    visited_pool.resize(data->n, 0);
}

MAGraph::~MAGraph()
{
    delete kd_tree;

    vector<vector<PFI>>().swap(reverse_edges);
    vector<size_t>().swap(visited_pool);
    vector<size_t>().swap(visited_tag);
    vector<vector<vector<PFI>>>().swap(edges);

    vector<vector<vector<int>>>().swap(search_edges);
}

float MAGraph::dis_compute(const int v1, const int v2) const
{
    return fstdistfunc_(data->get_vec(v1), data->get_vec(v2), dist_func_param_);
}

void MAGraph::set_threads(int max_threads)
{
    this->max_threads = max_threads;
    visited_tag.resize(max_threads);
    for (int i = 0; i < max_threads; i++)
    {
        visited_tag[i] = i;
    }
}

void MAGraph::build_graph()
{
    {
        Timer timer("Building kd-tree");
        kd_tree->build(M);
        edges.resize(data->n);
        for (auto i = 0; i < data->n; i++)
        {
            edges[i].resize(kd_tree->n_level);
        }
        cout << "The height of |T|: " << kd_tree->n_level << endl;
    }

    {
        Timer timer("Constructing graph index");
        vector<vector<KDTree::node*>> level_nodes;
        level_nodes.resize(kd_tree->n_level);
        stack<KDTree::node*> stk;
        stk.push(kd_tree->m_root);

        while (!stk.empty()) {
            auto curr = stk.top();
            stk.pop();

            level_nodes[curr->level].push_back(curr);

            for (int i = 0; i < curr->m_count; i++) {
                if (curr->m_child[i]) {
                    stk.push(curr->m_child[i]);
                }
            }
        }


        for (int layer = kd_tree->n_level - 1; layer >= 0; layer--)
        {
            {
                Timer local_timer("Processing level " + to_string(layer));

                int total = static_cast<int>(level_nodes[layer].size());
                int report_interval = (total + 99) / 100;

                if (total <= 100) { inter_para = false; }
                else { inter_para = true; }

                print_progress_bar(0, total);

                if (inter_para)
                {
                    atomic<int> completed{0};

                    BS::thread_pool pool(max_threads);
                    init_pool(pool);

                    const BS::multi_future<void> loop_future = pool.submit_loop(0, total,
                        [&](int i) {
                            process_node(level_nodes[layer][i]);
                            int done = ++completed;
                            if (done % report_interval == 0 || done == total) {
                                print_progress_bar(done, total);
                            }
                        });
                    loop_future.wait();
                }
                else
                {
                    int completed = 0;
                    for (int i = 0; i < total; i++)
                    {
                        process_node(level_nodes[layer][i]);
                        int done = ++completed;
                        if (done % report_interval == 0 || done == total) {
                            print_progress_bar(done, total);
                        }
                    }
                }
            }
        }
    }
}

void MAGraph::process_node(KDTree::node* u)
{
    if (u->m_count)
    {
        copy_first_child(u);
    }
    else
    {
        vector<int> processed_points(1, u->ids[0]);
        for (int i = 1; i < u->n; i++)
        {
            int pid = u->ids[i];
            insert(pid, u->level, -1, processed_points);

            vector<int> processing_points(1, pid);
            insert_backward(processing_points, u->level, numeric_limits<float>::max(), 0);

            processed_points.emplace_back(pid);
        }
        return;
    }

    vector<int> processed_points(u->m_child[0]->ids.begin(), u->m_child[0]->ids.end());
    for (int i = 1; i < u->m_count; i++)
    {
        auto cur_child = u->m_child[i];
        if (inter_para)
        {
            for (int pid : cur_child->ids)
            {
                insert(pid, u->level, cur_child->level, processed_points);
            }
        }
        else
        {
            BS::thread_pool pool(max_threads);
            init_pool(pool);
            const BS::multi_future<void> loop_future = pool.submit_loop(0, cur_child->ids.size(),
                [&](int j) {
                    int pid = cur_child->ids[j];
                    insert(pid, u->level, cur_child->level, processed_points);
                });
            loop_future.wait();
        }

        insert_backward(cur_child->ids, u->level, u->c_b[i].first, u->axis);
        processed_points.insert(processed_points.end(), cur_child->ids.begin(), cur_child->ids.end());
    }
}

void MAGraph::insert(int id, int level, int c_level, const vector<int> &processed_points)
{
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine e(seed);
    int merged_point_num = static_cast<int>(processed_points.size());
    uniform_int_distribution u_start(0, merged_point_num - 1);

    vector<int> enter_points;
    for (int i = 0; i < min(3, merged_point_num); i++)
    {
        int enter_pid = processed_points[u_start(e)];
        enter_points.emplace_back(enter_pid);
    }

    auto search_result = search_on_incomplete_graph(level, id, ef_c, ef_c, enter_points);
    while (!search_result.empty())
    {
        edges[id][level].emplace_back(search_result.top());
        search_result.pop();
    }

    if (c_level >= 0)
    {
        edges[id][level] = PruneByHeuristic2(edges[id][c_level], edges[id][level]);
    }
    else
    {
        vector<PFI> empty_list;
        edges[id][level] = PruneByHeuristic2(empty_list, edges[id][level]);
    }
}

void MAGraph::insert_backward(vector<int> &processing_points, int level, float lb, int a_id)
{
    unordered_set<int> affected_points;
    for (int pid : processing_points)
    {
        for (auto neighbor_pair : edges[pid][level]) {
            int neighborId = neighbor_pair.second;
            if (data->get_att_value(neighborId, a_id) < lb)
            {
                affected_points.insert(neighborId);
            }
        }
    }

    for (auto pid : affected_points)
    {
        reverse_edges[pid].clear();
    }

    for (int pid : processing_points)
    {
        for (auto neighbor_pair : edges[pid][level]) {
            int neighborId = neighbor_pair.second;
            if (affected_points.count(neighborId))
            {
                reverse_edges[neighborId].emplace_back(neighbor_pair.first, pid);
            }
        }
    }

    if (inter_para)
    {
        for (auto pid : affected_points)
        {
            edges[pid][level] = PruneByHeuristic2(edges[pid][level], reverse_edges[pid]);
        }
    }
    else
    {
        vector<int> affected_points_vec(affected_points.begin(), affected_points.end());
        BS::thread_pool pool(max_threads);
        init_pool(pool);
        const BS::multi_future<void> loop_future = pool.submit_loop(0, affected_points_vec.size(),
            [&](int i) {
                int pid = affected_points_vec[i];
                edges[pid][level] = PruneByHeuristic2(edges[pid][level], reverse_edges[pid]);
            });
        loop_future.wait();
    }
}

void MAGraph::copy_first_child(KDTree::node* u)
{
    auto first_child = u->m_child[0];
    int lower_layer = first_child->level;
    int higher_layer = u->level;
    for (auto id : first_child->ids)
    {
        edges[id][higher_layer] = edges[id][lower_layer];
    }
}

priority_queue<PFI> MAGraph::search_on_incomplete_graph(int level, int query_point,
        int ef, int query_k, const vector<int> &enter_points)
{
    auto thread_id = this_thread::get_id();
    int map_id = thread_map[thread_id];
    visited_tag[map_id] += max_threads;
    auto local_tag = visited_tag[map_id];

    priority_queue<PFI, vector<PFI>, greater<>> pool;
    priority_queue<PFI> candidates;

    for (auto pid : enter_points)
    {
        if (visited_pool[pid] == local_tag) continue;
        float dis = dis_compute(query_point, pid);
        visited_pool[pid] = local_tag;
        pool.emplace(dis, pid);
        candidates.emplace(dis, pid);
    }

    float lowerBound = candidates.top().first;

    int layer = level;

    while (!pool.empty())
    {
        auto current_pair = pool.top();
        if (current_pair.first > lowerBound)
            break;
        pool.pop();
        int current_pointId = current_pair.second;
        size_t size = edges[current_pointId][layer].size();

        for (int i = 0; i < size; i++)
        {
            int neighborId = edges[current_pointId][layer][i].second;
            if (visited_pool[neighborId] == local_tag)
                continue;
            visited_pool[neighborId] = local_tag;
            float dis = dis_compute(query_point, neighborId);
            if (candidates.size() < ef || dis < lowerBound)
            {
                candidates.emplace(dis, neighborId);
                pool.emplace(dis, neighborId);

                if (candidates.size() > ef)
                {
                    candidates.pop();
                }
                if (!candidates.empty())
                {
                    lowerBound = candidates.top().first;
                }
            }
        }
    }

    while (candidates.size() > query_k)
        candidates.pop();
    return candidates;
}

vector<PFI> MAGraph::PruneByHeuristic2(const vector<PFI> &old_list, const vector<PFI> &new_list)
{
    priority_queue<PFI, vector<PFI>, greater<>> queue_closest;
    vector<PFI> return_list;
    vector<bool> return_list_belong_to_lowerlayer_list;

    for (auto t : old_list)
        queue_closest.emplace(t);
    for (auto t : new_list)
        queue_closest.emplace(t);
    if (queue_closest.size() <= M)
    {
        while (queue_closest.size())
        {
            return_list.emplace_back(queue_closest.top());
            queue_closest.pop();
        }
        return return_list;
    }

    while (queue_closest.size())
    {
        if (return_list.size() >= M)
            break;

        auto current_pair = queue_closest.top();
        float dist_to_pid = current_pair.first;
        queue_closest.pop();

        bool good = true;
        bool current_old = false;
        for (auto t : old_list)
        {
            if (t.second == current_pair.second)
            {
                current_old = true;
                break;
            }
        }
        for (int i = 0; i < return_list.size(); i++)
        {
            if (current_old && return_list_belong_to_lowerlayer_list[i])
                continue;
            auto second_pair = return_list[i];
            float curdist = dis_compute(current_pair.second, second_pair.second);
            if (curdist < dist_to_pid)
            {
                good = false;
                break;
            }
        }
        if (good)
        {
            return_list.emplace_back(current_pair);
            return_list_belong_to_lowerlayer_list.emplace_back(current_old);
        }
    }
    return return_list;
}

void MAGraph::print_progress_bar(int current, int total, int width) {
    float progress = static_cast<float>(current) / static_cast<float>(total);
    int pos = width * progress;

    cout << "\r[";
    for (int i = 0; i < width; ++i) {
        if (i < pos)
            cout << "=";
        else if (i == pos)
            cout << ">";
        else
            cout << " ";
    }

    cout << "] " << fixed << setprecision(1) << (progress * 100.0f) << "%";
    if (current == total)
    {
        cout << endl;
    }
    cout.flush();
}

void MAGraph::init_pool(BS::thread_pool<> &pool)
{
    auto thread_ids = pool.get_thread_ids();
    thread_map.clear();
    for (int i = 0; i < thread_ids.size(); i++)
    {
        thread_map[thread_ids[i]] = i;
    }
}

void MAGraph::save(const string &prefix)
{
    string tree_file = prefix + "_tree.bin";
    kd_tree->save(tree_file);
    string graph_file = prefix + "_graph.bin";
    save_graph(graph_file);
}

void MAGraph::save_graph(const string &file)
{
    ofstream out(file, ios::binary);
    if (!out) {
        throw runtime_error("Failed to open file for writing (MAGraph)");
    }

    for (auto i = 0; i < data->n; i++)
    {
        for (auto j = 0; j < kd_tree->n_level; j++)
        {
            int sz = static_cast<int>(edges[i][j].size());
            out.write(reinterpret_cast<const char*>(&sz), sizeof(int));
            for (auto k = 0; k < sz; k++)
            {
                out.write(reinterpret_cast<const char*>(&edges[i][j][k].second), sizeof(int));
            }
        }
    }

    out.close();
}

void MAGraph::load(const string &prefix)
{
    string tree_file = prefix + "_tree.bin";
    kd_tree->load(tree_file);
    string graph_file = prefix + "_graph.bin";
    load_graph(graph_file);
}

void MAGraph::load_graph(const string &file)
{
    ifstream in(file, ios::binary);
    if (!in) {
        throw runtime_error("Failed to open file for reading (MAGraph)");
    }

    search_edges.resize(data->n);
    for (auto i = 0; i < data->n; i++)
    {
        search_edges[i].resize(kd_tree->n_level);
        for (auto j = 0; j < kd_tree->n_level; j++)
        {
            int sz = 0;
            in.read(reinterpret_cast<char*>(&sz), sizeof(int));
            search_edges[i][j].resize(sz);
            in.read(reinterpret_cast<char*>(search_edges[i][j].data()), sz * sizeof(int));
        }
    }
}

void MAGraph::search(vector<int> &SearchEF, const string &save_file, int edge_limit, Query* q, int top_k)
{
    ofstream outfile(save_file);
    if (!outfile.is_open())
        throw runtime_error("can not open " + save_file);

    vector<int> HOP;
    vector<int> DCO;
    vector<float> QPS;
    vector<float> RECALL;

    prefetch_lines = ((data->d + 7) / 8 * 8 * sizeof(float)) >> 4;

    for (auto ef : SearchEF)
    {
        int tp = 0;
        float search_time = 0;

        metric_hops = 0;
        metric_distance_computations = 0;

        for (int i = 0; i < q->n_q; i++)
        {
            if (i % 10 == 0) { cout << "Performing the " << i << "th query..." << endl; }
            auto gt = q->knn_id[i];
            auto start = chrono::high_resolution_clock::now();

            vector<KDTree::node*> filtered_nodes = range_filter(q->q_a[i], top_k);
            priority_queue<PFI> res = TopDown_nodeentries_search(filtered_nodes, q->q_v[i], ef, top_k, q->q_a[i], edge_limit);

            auto end = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
            search_time += duration;

            unordered_set<int> record;
            while (!res.empty())
            {
                auto x = res.top().second;
                res.pop();
                if (record.count(x))
                    throw runtime_error("repetitive search results");
                record.insert(x);
                if (find_if(gt.begin(), gt.begin() + top_k, [&](int gt_id) {
                    return gt_id == x;
                    }) != gt.begin() + top_k)
                {
                    tp++;
                }
            }
        }

        float recall = static_cast<float>(tp) / static_cast<float>(q->n_q) / static_cast<float>(top_k);
        float qps = static_cast<float>(q->n_q) / search_time * 1000 * 1000;
        float dco = static_cast<float>(metric_distance_computations) / static_cast<float>(q->n_q);
        float hop = static_cast<float>(metric_hops) / static_cast<float>(q->n_q);

        HOP.emplace_back(hop);
        DCO.emplace_back(dco);
        QPS.emplace_back(qps);
        RECALL.emplace_back(recall);
    }

    for (int i = 0; i < RECALL.size(); i++)
    {
        outfile << SearchEF[i] << "," << RECALL[i] << "," << QPS[i] << "," << DCO[i] << "," << HOP[i] << endl;
    }

    outfile.close();
}

vector<KDTree::node*> MAGraph::range_filter(vector<float> &range, int top_k)
{
    vector<KDTree::node*> res;
    res.reserve(top_k);

    auto qlb = [&](int ax)->float { return range[(ax<<1)    ]; };
    auto qrb = [&](int ax)->float { return range[(ax<<1) + 1]; };

    struct Frame { KDTree::node* n; uint64_t sat; };
    vector<Frame> stk;
    stk.reserve(256);
    stk.push_back({ kd_tree->m_root, 0ull });

    while (!stk.empty()) {
        auto cur = stk.back();
        stk.pop_back();

        KDTree::node* node = cur.n;
        uint64_t mask = 0;
        for (auto dim : node->ignored_dim) {
            mask |= (1ull << dim);
        }
        uint64_t sat = cur.sat | mask;

        int ax = node->axis;
        float lb = qlb(ax), rb = qrb(ax);

        if (__builtin_popcountll(sat) == kd_tree->k) {
            res.emplace_back(node);
            if (res.size() == top_k) break;
            continue;
        }

        if (node->m_count == 0) continue;

        if ((sat & (1ull << ax)) != 0ull) {
            for (int i = 0; i < node->m_count; i++) {
                stk.push_back({ node->m_child[i], sat });
            }
        } else {
            for (int i = 0; i < node->m_count; i++) {
                const auto &cb = node->c_b[i];
                if (!(cb.first > rb || cb.second < lb)) {
                    if (cb.first >= lb && cb.second <= rb) {
                        stk.push_back({ node->m_child[i], sat | (1ull << ax) });
                    } else
                    {
                        stk.push_back({ node->m_child[i], sat });
                    }
                }
            }
        }
    }

    return res;
}

priority_queue<PFI> MAGraph::TopDown_nodeentries_search(const vector<KDTree::node*> &filtered_nodes, float* query_vec,
        int ef, int query_k, vector<float> &range, int edge_limit)
{
    priority_queue<PFI, vector<PFI>, greater<PFI>> candidate_set;
    priority_queue<PFI> top_candidates;
    searcher::Bitset visited_set(data->n);

    for (auto u : filtered_nodes)
    {

        for (int pid : u->ids)
        {
            visited_set.set(pid);
            if (check_constraint(pid, range))
            {
                float dis = fstdistfunc_(query_vec, data->get_vec(pid), dist_func_param_);
                ++metric_distance_computations;
                candidate_set.emplace(dis, pid);
                top_candidates.emplace(dis, pid);
                break;
            }
        }
    }

    float lowerBound = std::numeric_limits<float>::max();
    metric_hops = 0;
    while (!candidate_set.empty())
    {
        auto current_point_pair = candidate_set.top();
        ++metric_hops;
        if (current_point_pair.first > lowerBound)
        {
            break;
        }
        candidate_set.pop();
        int current_pid = current_point_pair.second;
        auto selected_edges = SelectEdge(current_pid, range, edge_limit, visited_set);
        int num_edges = static_cast<int>(selected_edges.size());
        for (int i = 0; i < min(num_edges, 3); i++)
        {
            memory::mem_prefetch_L1(reinterpret_cast<char*>(data->get_vec(selected_edges[i])), this->prefetch_lines);
        }
        for (auto &neighbor_id : selected_edges)
        {
            float dis = fstdistfunc_(query_vec, data->get_vec(neighbor_id), dist_func_param_);
            ++metric_distance_computations;

            if (top_candidates.size() < ef || dis < lowerBound)
            {
                top_candidates.emplace(dis, neighbor_id);
                candidate_set.emplace(dis, neighbor_id);

                if (top_candidates.size() > ef)
                    top_candidates.pop();
                if (!top_candidates.empty())
                    lowerBound = top_candidates.top().first;
            }
        }
    }
    while (top_candidates.size() > query_k)
        top_candidates.pop();
    return top_candidates;
}

bool MAGraph::check_constraint(int id, vector<float> &range) const
{
    for (int i = 0; i < kd_tree->k; i++)
    {
        auto a_v = data->get_att_value(id, i);
        if (!(a_v >= range[2 * i] && a_v <= range[2 * i + 1]))
        {
            return false;
        }
    }
    return true;
}

vector<int> MAGraph::SelectEdge(int pid, vector<float> &range, int edge_limit, searcher::Bitset<uint64_t> &visited_set)
{
    std::vector<int> out;
    out.reserve(edge_limit);

    for (int layer = 0; layer < kd_tree->n_level; layer++)
    {
        for (auto neighborId : search_edges[pid][layer])
        {
            if (visited_set.get(neighborId))
                continue;
            visited_set.set(neighborId);
            bool inrange = check_constraint(neighborId, range);
            if (!inrange) continue;
            out.emplace_back(neighborId);
            if (out.size() == edge_limit) return out;
        }
    }

    return out;
}

void MAGraph::average_degree()
{
    for (int layer = 0; layer < kd_tree->n_level; layer++)
    {
        size_t count = 0;
        for (int i = 0; i < data->n; i++)
        {
            count += search_edges[i][layer].size();
        }
        cout << "Layer " << layer << ", Average degree: " << static_cast<float>(count) / static_cast<float>(data->n) << endl;
    }
}
