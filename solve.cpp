#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <climits>
#include <vector>
#include <stack>
#include <set>
#include <map>

using namespace std;

class Graph {
private:
    int capacity;
    int size;
public:
    char *vertices;
    int *edges;
    explicit Graph(int capacity) {
        this->capacity = capacity;
        this->vertices = (char*) malloc(sizeof(char) * capacity);
        this->edges = (int*) malloc(sizeof(int) * pow(capacity, 2));
        this->size = 0;
        memset(this->vertices, 0, sizeof(char) * capacity);
        memset(this->edges, 0, sizeof(int) * pow(capacity, 2));
    }
    int get_size() {
        return size;
    }
    char get_vertex(int u) {
        return vertices[u];
    }
    int get_edge(int i, int j) {
        return edges[capacity * i + j];
    }
    void add_vertex(char label) {
        vertices[size++] = label;
    }
    int get_index(char label) {
        int pos = -1;
        for (int i = 0; i < size && pos == -1; i++) {
            if (vertices[i] == label) {
                pos = i;
            }
        }
        return pos;
    }
    void set_edge(char label1, char label2, int value) {
        int pos1 = -1, pos2 = -1;
        for (int i = 0; i < size && (pos1 == -1 || pos2 == -1); i++) {
            if (vertices[i] == label1 && pos1 == -1) {
                pos1 = i;
            }
            if (vertices[i] == label2 && pos2 == -1) {
                pos2 = i;
            }
        }
        edges[capacity * pos1 + pos2] = value;
        edges[capacity * pos2 + pos1] = value;
    }
};

class MCTSNode {
public:
    char label;
    int index;
    int sim;
    int win;
    map<int, MCTSNode*> children;
    MCTSNode *parent;
    MCTSNode(char label, int index, MCTSNode *parent) {
        this->label = label;
        this->index = index;
        this->parent = parent;
        sim = win = 0;
    }
};

vector<int> mcts(Graph g, int source, int iteration) {
    vector<int> vec;
    MCTSNode root('\0', -1, nullptr);
    MCTSNode *prev, *curr;
    double c = 1;
    int total_sim = 0;
    vector<MCTSNode*> leaves;
    for (int itr = 0; itr < iteration; itr++) {
        int u = source;
        int sum_path = 0;
        set<int> visited;
        stack<MCTSNode*> trace;
        vector<int> path;
        curr = &root;
        trace.push(&root);
        while (u != -1) {
            prev = curr;
            if (prev->children.find(u) != prev->children.end()) {
                curr = prev->children.at(u);
            } else {
                curr = new MCTSNode(g.get_vertex(u), u, prev);
                prev->children.insert(make_pair(u, curr));
            }
            visited.insert(u);
            path.push_back(u);
            trace.push(curr);
            int best_child = -1;
            double best_ucb = INT_MIN;
            int max_edge = INT_MIN;
            for (int v = 0; v < g.get_size(); v++) {
                if (visited.find(v) == visited.end()) {
                    int e = g.get_edge(u, v);
                    if (e > max_edge) {
                        max_edge = e;
                    }
                }
            }
            for (int v = 0; v < g.get_size(); v++) {
                if (visited.find(v) == visited.end()) {
                    int edge = g.get_edge(u, v);
                    int selectedNodeSim = 0;
                    if (curr->children.find(v) != curr->children.end()) {
                        selectedNodeSim = curr->children.at(v)->sim;
                    }
                    double mctsFactor = c * sqrt(log(curr->sim) / selectedNodeSim);
                    if (isnan(mctsFactor)) {
                        mctsFactor = 0;
                    }
                    double ucb = (1 - ((double) edge / max_edge)) + mctsFactor;
                    if (ucb > best_ucb) {
                        best_child = v;
                        best_ucb = ucb;
                    }
                }
            }
            sum_path += g.get_edge(u, best_child);
            u = best_child;
        }
        MCTSNode *leaf;
        if (curr->children.empty()) {
            leaf = new MCTSNode('\0', sum_path, curr);
            curr->children.insert(make_pair(0, leaf));
            leaves.push_back(leaf);
        } else {
            leaf = curr->children.at(0);
        }
        trace.push(leaf);
        while (!trace.empty()) {
            trace.top()->sim++;
            total_sim++;
            trace.pop();
        }
    }
    int min_distance = INT_MAX;
    MCTSNode* top_leaf;
    for (auto &it : leaves) {
        if (it->index < min_distance) {
            min_distance = it->index;
            top_leaf = it;
        }
    }
    curr = top_leaf->parent;
    while (curr != &root) {
        vec.insert(vec.begin(), curr->index);
        curr = curr->parent;
    }
    return vec;
}

void write_csv(const map<int, map<int, double>>& results, const vector<int>& mcts_iterations, const string& csv_filename) {
    ofstream fout(csv_filename);
    fout << "N,";
    for (auto it : mcts_iterations) {
        fout << it << " Iteration,";
    }
    fout << endl;
    for (auto it : results) {
        fout << it.first << ",";
        for (auto it1 : it.second) {
            fout << it1.second << ",";
        }
        fout << endl;
    }
    fout.close();
}

int main() {
    int mcts_iteration_cnt;
    vector<int> mcts_iterations;
    cin >> mcts_iteration_cnt;
    for (int i = 0; i < mcts_iteration_cnt; i++) {
        int a;
        cin >> a;
        mcts_iterations.push_back(a);
    }
    map<int, map<int, double>> results; // map< vertex, map<iteration, correct> >
    for (int j = 0; j < mcts_iteration_cnt; j++) {
        int mcts_iteration = mcts_iterations[j];
        cout << "Start testing on " << mcts_iteration << " Iteration" << endl;
        for (int n = 3; n <= 12; n++) {
            if (results.find(n) == results.end()) {
                results.insert(make_pair(n, map<int, double>()));
            }
            results.at(n).insert(make_pair(mcts_iteration, 0.0));
            for (int x = 1; x <= 100; x++) {
                Graph g(30);
                string fileName = "../testset/graph_" + to_string(n) + "_" + to_string(x) + ".txt";
                ifstream fin(fileName);
                int min_distance;
                int vertex_cnt, edge_cnt, edge;
                char vertex1, vertex2;
                fin >> vertex_cnt;
                for (int k = 0; k < vertex_cnt; k++) {
                    fin >> vertex1;
                    g.add_vertex(vertex1);
                }
                fin >> edge_cnt;
                for (int k = 0; k < edge_cnt; k++) {
                    fin >> vertex1 >> vertex2 >> edge;
                    g.set_edge(vertex1, vertex2, edge);
                }
                for (int k = 0; k < vertex_cnt; k++) {
                    fin >> vertex1;
                }
                fin >> min_distance;
                fin.close();
                vector<int> path = mcts(g, 0, mcts_iteration);
                int sum_path = 0;
                int prev = -1;
                for (auto &it : path) {
                    if (prev != -1) {
                        sum_path += g.get_edge(prev, it);
                    }
                    prev = it;
                }
                //cout << "Graph " << n << " vertex (#" << x << ")" << endl;
                //cout << "Min Distance -> " << min_distance << " , MCTS Result -> " << sum_path << endl;
                results.at(n).at(mcts_iteration) +=  (double) (sum_path - min_distance) / min_distance;
            }
        }
        cout << "Finish testing on " << mcts_iteration << " Iteration" << endl;
    }
    write_csv(results, mcts_iterations, "../result.csv");
    cout << "Result saved on result.csv" << endl;
    return 0;
}
