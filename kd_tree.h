#ifndef KD_TREE_H
#define KD_TREE_H
#include <vector>
#include <unordered_set>
#include <utility>
#include <stack>
#include <algorithm>
#include "data.h"

using namespace std;

class KDTree
{
public:
    struct node
    {
        int level = 0;
        int axis = 0;
        unordered_set<int> ignored_dim;

        float l_b = 0.0f, r_b = 0.0f;

        int m_count = 0;
        vector<node*> m_child;
        vector<pair<float, float>> c_b;

        int n = 0;
        vector<int> ids;
    };

    int k = 0;
    int n_level = 0;
    Data* data = nullptr;
    node* m_root = nullptr;

    explicit KDTree(Data* data);
    ~KDTree();
    void delete_tree(node* u);
    void build(float b_f = 2);
    void save(const string &file);
    void save_node(ofstream& out, node* nd);
    void load(const string& file);
    node* load_node(ifstream& in);
};

#endif // KD_TREE_H
