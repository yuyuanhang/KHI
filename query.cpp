#include "query.h"

Query::~Query()
{
    for (auto ptr : q_v)
    {
        free(ptr);
    }
    vector<vector<float>>().swap(q_a);
    vector<vector<int>>().swap(knn_id);
    vector<vector<float>>().swap(knn_dist);
}

void Query::load_queries(const string &file) {
    ifstream in(file, ios::binary);

    if (!in) {
        throw runtime_error("can not open " + file + " (Query, query)");
    }

    in.read(reinterpret_cast<char*>(&n_q), sizeof(int));
    in.read(reinterpret_cast<char*>(&d), sizeof(int));

    q_v.resize(n_q);
    for (auto i = 0; i < n_q; i++) {
        auto vec = static_cast<float*>(aligned_alloc(alignment, d * sizeof(float)));
        in.read(reinterpret_cast<char*>(vec), static_cast<streamsize>(d * sizeof(float)));
        q_v[i] = vec;
    }

    in.close();
}

void Query::load_constraints(const string &file, Data* data)
{
    ifstream in(file, ios::binary);

    if (!in)
    {
        throw runtime_error("can not open " + file + " (Query, constraint)");
    }

    in.read(reinterpret_cast<char*>(&n_a), sizeof(int));
    if (n_a != 2 * data->n_a)
    {
        throw runtime_error("the constraint file " + file + " is inconsistent with the meta file in the number of attributes");
    }

    header.resize(n_a);
    for (auto i = 0; i < n_a; i++)
    {
        int len;
        in.read(reinterpret_cast<char*>(&len), sizeof(int));

        header[i].resize(len);
        in.read(&header[i][0], len);
    }

    int n;
    in.read(reinterpret_cast<char*>(&n), sizeof(int));
    if (n != n_q)
    {
        throw runtime_error("the constraint file " + file + " is inconsistent with its query file in the number of queries");
    }

    vector<float*> a_v;
    a_v.resize(n_a);
    for (auto i = 0; i < n_a; i++)
    {
        auto ptr = new float[n];
        in.read(reinterpret_cast<char*>(ptr), n * sizeof(float));
        a_v[i] = ptr;
    }

    vector<int> map(n_a);
    for (auto i = 0; i < data->n_a; i++)
    {
        auto curr_header = data->header[i];
        for (int j = 0; j < n_a; j++)
        {
            if (header[j] == curr_header + "_low")
            {
                map[2 * i] = j;
                break;
            }
        }
        for (int j = 0; j < n_a; j++)
        {
            if (header[j] == curr_header + "_high")
            {
                map[2 * i + 1] = j;
                break;
            }
        }
    }

    q_a.resize(n_q);
    for (auto i = 0; i < n_q; i++)
    {
        for (auto j = 0; j < n_a; j++)
        {
            q_a[i].push_back(a_v[map[j]][i]);
        }
    }

    for (auto i = 0; i < n_a; i++)
    {
        delete[] a_v[i];
    }

    in.close();
}

void Query::load_answers(const string &file) {
    ifstream in(file, ios::binary);
    if (!in) {
        throw runtime_error("can not open " + file + " (Query, answer)");
    }

    int n, top_k;
    in.read(reinterpret_cast<char*>(&n), sizeof(int));
    in.read(reinterpret_cast<char*>(&top_k), sizeof(int));
    if (n != n_q)
    {
        throw runtime_error("the groundtruth file " + file + " is inconsistent with its query file in the number of queries");
    }

    knn_id.resize(n_q, vector<int>(top_k));
    knn_dist.resize(n_q, vector<float>(top_k));

    for (auto i = 0; i < n_q; ++i) {
        in.read(reinterpret_cast<char*>(knn_id[i].data()), top_k * sizeof(int));
        in.read(reinterpret_cast<char*>(knn_dist[i].data()), top_k * sizeof(float));
    }

    in.close();
}
