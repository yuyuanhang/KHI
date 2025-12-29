#ifndef DATA_H
#define DATA_H
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>

using namespace std;

class Data
{
public:
    int n;
    int n_a;
    int d;

    size_t alignment = 64;
    float* v_v = nullptr;
    vector<float*> a_v;

    vector<string> header;

    Data(const string &att_file, const string &vec_file);
    ~Data();
    void load_attribute(const string &file);
    void load_vector(const string &file);
    [[nodiscard]] float* get_vec(int id) const;
    [[nodiscard]] float get_att_value(int id, int a_id) const;
};

#endif // DATA_H
