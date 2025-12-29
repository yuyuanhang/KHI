#include "data.h"

Data::Data(const string &att_file, const string &vec_file)
{
    n = 0; n_a = 0; d = 0;
    load_attribute(att_file);
    load_vector(vec_file);
}

Data::~Data()
{
    free(v_v);
    for (auto ptr : a_v)
    {
        delete[] ptr;
    }
}

void Data::load_attribute(const string &file) {
    ifstream in(file, ios::binary);

    if (!in)
    {
        throw runtime_error("can not open " + file + " (Data, attribute)");
    }

    in.read(reinterpret_cast<char*>(&n_a), sizeof(int));

    header.resize(n_a);
    for (auto i = 0; i < n_a; i++)
    {
        int len;
        in.read(reinterpret_cast<char*>(&len), sizeof(int));

        header[i].resize(len);
        in.read(&header[i][0], len);
    }

    in.read(reinterpret_cast<char*>(&n), sizeof(int));
    a_v.resize(n_a);
    for (auto i = 0; i < n_a; i++)
    {
        auto ptr = new float[n];
        in.read(reinterpret_cast<char*>(ptr), n * sizeof(float));
        a_v[i] = ptr;
    }

    in.close();
}

void Data::load_vector(const string &file)
{
    ifstream in(file, ios::binary);

    if (!in) {
        throw runtime_error("can not open " + file + " (Data, vector)");
    }

    int n_v = 0;
    in.read(reinterpret_cast<char*>(&n_v), sizeof(int));
    in.read(reinterpret_cast<char*>(&d), sizeof(int));

    if (n_v != n)
    {
        throw runtime_error("the number of vectors and the number of attribute values are inconsistent");
    }

    auto N = static_cast<size_t>(n);
    auto D = static_cast<size_t>(d);
    v_v = static_cast<float*>(aligned_alloc(alignment, N * D * sizeof(float)));
    in.read(reinterpret_cast<char*>(v_v), N * D * sizeof(float));

    in.close();
}

float* Data::get_vec(int id) const
{
    return v_v + static_cast<size_t>(id) * d;
}

float Data::get_att_value(int id, int a_id) const
{
    return a_v[a_id][id];
}