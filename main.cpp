#include <iostream>
#include <string>
#include "timer.h"
#include "data.h"
#include "ma_graph.h"

using namespace std;

int main(int argc, char* argv[])
{
    string func = argv[1];
    if (func == "-idx")
    {
        string att_file = argv[2];
        string vec_file = argv[3];
        string idx_prefix = argv[4];
        int max_threads = stoi(argv[5]);
        int M = stoi(argv[6]);

        Data* data = nullptr;
        {
            Timer timer("Loading attribute values and vectors");
            data = new Data(att_file, vec_file);
        }

        MAGraph* idx = nullptr;
        {
            Timer timer("Constructing index");
            idx = new MAGraph(data, M);
            idx->set_threads(max_threads);
            idx->build_graph();
        }

        {
            Timer timer("Saving index");
            idx->save(idx_prefix);
        }

        {
            Timer timer("Releasing memory");
            delete idx;
        }
    }
    else if (func == "-q")
    {
        string att_file = argv[2];
        string vec_file = argv[3];
        string idx_prefix = argv[4];
        string q_vec_file = argv[5];
        string q_cons_file = argv[6];
        string ans_file = argv[7];
        string res_file = argv[8];
        int top_k = stoi(argv[9]);
        int edge_limit = stoi(argv[10]);

        Data* data = nullptr;
        {
            Timer timer("Loading attribute values and vectors");
            data = new Data(att_file, vec_file);
        }

        Query* q = nullptr;
        {
            Timer timer("Loading queries");
            q = new Query;
            q->load_queries(q_vec_file);
            q->load_constraints(q_cons_file, data);
            q->load_answers(ans_file);
        }

        MAGraph* idx = nullptr;
        {
            Timer timer("Loading index");
            idx = new MAGraph(data);
            idx->load(idx_prefix);
            idx->average_degree();
        }

        {
            Timer timer("Querying");
            vector<int> SearchEF = {1400, 700, 400, 300, 250, 200, 180, 160, 140, 120, 100, 90, 80, 70, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10};
            idx->search(SearchEF, res_file, edge_limit, q, top_k);
        }

        {
            Timer timer("Releasing memory");
            delete q;
            delete idx;
            delete data;
        }
    }
}
