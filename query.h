#ifndef QUERY_H
#define QUERY_H
#include "data.h"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <limits>

using namespace  std;

class Query
{
public:
    int n_a;
    int n_q;
    int d;
    vector<string> header;

    size_t alignment = 64;
    vector<vector<float>> q_a;
    vector<float*> q_v;
    vector<vector<int>> knn_id;
    vector<vector<float>> knn_dist;

    ~Query();
    void load_queries(const string &file);
    void load_constraints(const string &file, Data* data);
    void load_answers(const string &file);
};

#endif //QUERY_H
