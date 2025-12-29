#include "kd_tree.h"

KDTree::KDTree(Data* data)
{
    this->data = data;
    k = data->n_a;
}

KDTree::~KDTree()
{
    delete data;
    delete_tree(m_root);
}

void KDTree::delete_tree(node* u) {
    if (!u) return;

    for (node* child : u->m_child) {
        delete_tree(child);
    }

    delete u;
}

void KDTree::build(float b_f)
{
    m_root = new node;
    m_root->n = data->n;
    m_root->ids.resize(data->n);
    for (int i = 0; i < data->n; i++)
    {
        m_root->ids[i] = i;
    }
    stack<node*> stk;
    stk.push(m_root);

    while (!stk.empty()) {
        auto curr = stk.top();
        stk.pop();

        if (curr->ignored_dim.size() == k)
        {
            if (curr->level >= n_level)
            {
                n_level = curr->level + 1;
            }
            continue;
        }

        while (curr->ignored_dim.count(curr->axis))
        {
            curr->axis = (curr->axis + 1) % k;
        }
        // Sort points by current dimension
        vector<pair<int, float>> p_a(curr->n);
        for (int i = 0; i < curr->n; ++i)
        {
            int id = curr->ids[i];
            p_a[i] = {id, data->get_att_value(id, curr->axis)};
        }

        sort(p_a.begin(), p_a.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

        curr->l_b = p_a[0].second; curr->r_b = p_a.back().second;

        if (curr->n <= 2)
        {
            if (curr->level >= n_level)
            {
                n_level = curr->level + 1;
            }
            continue;
        }

        int mid = curr->n / 2;
        float bound = p_a[mid].second;

        auto l_c = new node;
        auto r_c = new node;
        l_c->level = curr->level + 1; r_c->level = l_c->level;
        l_c->axis = (curr->axis + 1) % k; r_c->axis = l_c->axis;
        l_c->ignored_dim = curr->ignored_dim; r_c->ignored_dim = curr->ignored_dim;

        for (int i = 0; i < curr->n; ++i)
        {
            if (p_a[i].second < bound)
                l_c->ids.push_back(p_a[i].first);
            else
                r_c->ids.push_back(p_a[i].first);
        }

        l_c->n = static_cast<int>(l_c->ids.size());
        r_c->n = static_cast<int>(r_c->ids.size());

        if (b_f * static_cast<float>(min(l_c->n, r_c->n)) <= static_cast<float>(max(l_c->n, r_c->n)))
        {
            curr->m_count = 1;

            l_c->ids = curr->ids;
            l_c->n = l_c->ids.size();
            curr->m_child.emplace_back(l_c);
            curr->c_b.emplace_back(p_a[0].second, p_a.back().second);
            l_c->ignored_dim.insert(curr->axis);

            stk.push(l_c);
            delete r_c;
        } else {
            curr->m_count = 2;

            curr->m_child.push_back(l_c);
            curr->c_b.emplace_back(p_a[0].second, p_a[l_c->n - 1].second);

            curr->m_child.push_back(r_c);
            curr->c_b.emplace_back(p_a[l_c->n].second, p_a.back().second);

            stk.push(l_c); stk.push(r_c);
        }
    }
}

void KDTree::save(const string &file)
{
    ofstream out(file, ios::binary);
    if (!out) {
        throw runtime_error("Failed to open file for writing (KDTree)");
    }

    out.write(reinterpret_cast<char*>(&k), sizeof(int));
    out.write(reinterpret_cast<char*>(&n_level), sizeof(int));

    save_node(out, m_root);

    out.close();
}

void KDTree::save_node(ofstream& out, node* nd) {
    out.write(reinterpret_cast<char*>(&nd->level), sizeof(int));
    out.write(reinterpret_cast<char*>(&nd->axis), sizeof(int));

    int ignored_size = static_cast<int>(nd->ignored_dim.size());
    out.write(reinterpret_cast<char*>(&ignored_size), sizeof(int));
    for (int dim : nd->ignored_dim) {
        out.write(reinterpret_cast<char*>(&dim), sizeof(int));
    }

    out.write(reinterpret_cast<char*>(&nd->l_b), sizeof(float));
    out.write(reinterpret_cast<char*>(&nd->r_b), sizeof(float));

    out.write(reinterpret_cast<char*>(&nd->m_count), sizeof(int));
    for (const auto& p : nd->c_b) {
        out.write(reinterpret_cast<const char*>(&p.first), sizeof(float));
        out.write(reinterpret_cast<const char*>(&p.second), sizeof(float));
    }

    out.write(reinterpret_cast<char*>(&nd->n), sizeof(int));
    for (int id : nd->ids) {
        out.write(reinterpret_cast<char*>(&id), sizeof(int));
    }

    for (node* child : nd->m_child) {
        save_node(out, child);
    }
}

void KDTree::load(const string& file) {
    ifstream in(file, ios::binary);
    if (!in) {
        throw runtime_error("Failed to open file for reading (KDTree)");
    }

    in.read(reinterpret_cast<char*>(&k), sizeof(int));
    in.read(reinterpret_cast<char*>(&n_level), sizeof(int));

    m_root = load_node(in);
    in.close();
}

KDTree::node* KDTree::load_node(ifstream& in) {
    node* nd = new node;

    in.read(reinterpret_cast<char*>(&nd->level), sizeof(int));
    in.read(reinterpret_cast<char*>(&nd->axis), sizeof(int));

    int ignored_size;
    in.read(reinterpret_cast<char*>(&ignored_size), sizeof(int));
    for (int i = 0; i < ignored_size; i++) {
        int dim;
        in.read(reinterpret_cast<char*>(&dim), sizeof(int));
        nd->ignored_dim.insert(dim);
    }

    in.read(reinterpret_cast<char*>(&nd->l_b), sizeof(float));
    in.read(reinterpret_cast<char*>(&nd->r_b), sizeof(float));

    in.read(reinterpret_cast<char*>(&nd->m_count), sizeof(int));
    nd->c_b.resize(nd->m_count);
    for (int i = 0; i < nd->m_count; ++i) {
        float a, b;
        in.read(reinterpret_cast<char*>(&a), sizeof(float));
        in.read(reinterpret_cast<char*>(&b), sizeof(float));
        nd->c_b[i] = {a, b};
    }

    in.read(reinterpret_cast<char*>(&nd->n), sizeof(int));
    nd->ids.resize(nd->n);
    for (int i = 0; i < nd->n; ++i) {
        in.read(reinterpret_cast<char*>(&nd->ids[i]), sizeof(int));
    }

    nd->m_child.resize(nd->m_count);
    for (int i = 0; i < nd->m_count; i++) {
        nd->m_child[i] = load_node(in);
    }

    return nd;
}