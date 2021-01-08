#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <vector>

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

int bruteforce_min(Graph g) {
    char *labels = (char*) malloc(sizeof(char) * g.get_size());
    int *direction = (int*) malloc(sizeof(int) * g.get_size());
    memset(direction, 0, sizeof(int) * g.get_size());
    for (int i = 0; i < g.get_size(); i++) {
        labels[i] = g.get_vertex(i);
    }
    int mobileIndex;
    char mobileChar;
    vector<int> path;
    int min_distance = INT_MAX;
    int cnt = 0;
    do {
        mobileIndex = -1;
        mobileChar = 0;
        int distance = 0;
        int prev = -1;
        cout << "S: ";
        for (int i = 0; i < g.get_size(); i++) {
            cout << labels[i] << " ";
            int curr = g.get_index(labels[i]);
            if (prev != -1) {
                distance += g.get_edge(prev, curr);
            }
            prev = curr;
        }
        cout << endl;
        cout << "D: " << distance << endl;
        if (distance < min_distance) {
            min_distance = distance;
            path.clear();
            for (int i = 0; i < g.get_size(); i++) {
                path.push_back(g.get_index(labels[i]));
            }
        }
        for (int i = 1; i < g.get_size(); i++) {
            if (i == 1) {
                if (direction[i] == 1 && labels[i] > labels[i+1] && labels[i] > mobileChar) {
                    mobileIndex = i;
                    mobileChar = labels[mobileIndex];
                }
            } else if (i == g.get_size() - 1) {
                if (direction[i] == 0 && labels[i] > labels[i-1] && labels[i] > mobileChar) {
                    mobileIndex = i;
                    mobileChar = labels[mobileIndex];
                }
            } else {
                if (direction[i] == 0 && labels[i] > labels[i-1] && labels[i] > mobileChar) {
                    mobileIndex = i;
                    mobileChar = labels[mobileIndex];
                } else if (direction[i] == 1 && labels[i] > labels[i+1] && labels[i] > mobileChar) {
                    mobileIndex = i;
                    mobileChar = labels[mobileIndex];
                }
            }
        }
        if (direction[mobileIndex] == 0) {
            char tempc = labels[mobileIndex];
            labels[mobileIndex] = labels[mobileIndex-1];
            labels[mobileIndex-1] = tempc;
            int tempi = direction[mobileIndex];
            direction[mobileIndex] = direction[mobileIndex-1];
            direction[mobileIndex-1] = tempi;
            mobileChar = labels[mobileIndex-1];
        } else {
            char tempc = labels[mobileIndex];
            labels[mobileIndex] = labels[mobileIndex+1];
            labels[mobileIndex+1] = tempc;
            int tempi = direction[mobileIndex];
            direction[mobileIndex] = direction[mobileIndex+1];
            direction[mobileIndex+1] = tempi;
            mobileChar = labels[mobileIndex+1];
        }
        for (int i = 1; i < g.get_size(); i++) {
            if (labels[i] > mobileChar) {
                direction[i] = (direction[i] == 0) ? 1 : 0;
            }
        }
        cnt++;
        cout << "C: " << cnt << endl;
    } while (mobileIndex != -1);
    cout << "Count: " << cnt << endl;
    cout << "Path: ";
    for (auto &it : path) {
        cout << g.get_vertex(it) << " ";
    }
    cout << endl;
    cout << "Distance: " << min_distance << endl;
}

int main() {
    Graph g(30);
    ifstream fin("../graph.txt");
    int vertex_cnt, edge_cnt, edge;
    char vertex1, vertex2;
    fin >> vertex_cnt;
    for (int i = 0; i < vertex_cnt; i++) {
        fin >> vertex1;
        g.add_vertex(vertex1);
    }
    fin >> edge_cnt;
    for (int i = 0; i < edge_cnt; i++) {
        fin >> vertex1 >> vertex2 >> edge;
        g.set_edge(vertex1, vertex2, edge);
    }
    permutation(g);
    return 0;
}
