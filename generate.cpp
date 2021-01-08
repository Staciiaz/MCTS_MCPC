#include <iostream>
#include <fstream>
#include <random>
#include <cstring>
#include <ctime>

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

void bruteforce_min(Graph g, vector<int> &path, int &min_distance) {
    char *labels = (char*) malloc(sizeof(char) * g.get_size());
    int *direction = (int*) malloc(sizeof(int) * g.get_size());
    memset(direction, 0, sizeof(int) * g.get_size());
    for (int i = 0; i < g.get_size(); i++) {
        labels[i] = g.get_vertex(i);
    }
    int mobileIndex;
    char mobileChar;
    min_distance = INT_MAX;
    int cnt = 0;
    do {
        mobileIndex = -1;
        mobileChar = 0;
        int distance = 0;
        int prev = -1;
        //cout << "S: ";
        for (int i = 0; i < g.get_size(); i++) {
            //cout << labels[i] << " ";
            int curr = g.get_index(labels[i]);
            if (prev != -1) {
                distance += g.get_edge(prev, curr);
            }
            prev = curr;
        }
        //cout << endl;
        //cout << "D: " << distance << endl;
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
        //cout << "C: " << cnt << endl;
    } while (mobileIndex != -1);
    /*
    cout << "Count: " << cnt << endl;
    cout << "Path: ";
    for (auto &it : path) {
        cout << g.get_vertex(it) << " ";
    }
    cout << endl;
    cout << "Distance: " << min_distance << endl;
     */
}

int main() {
    srand(time(nullptr));
    for (int n = 3; n <= 12; n++) {
        for (int x = 1; x <= 100; x++) {
            Graph g(30);
            string fileName = "../testset/graph_" + to_string(n) + "_" + to_string(x) + ".txt";
            ofstream fout(fileName);
            char *labels = (char*) malloc(sizeof(char) * n);
            fout << n << endl;
            for (int i = 0; i < n; i++) {
                labels[i] = 'A' + i;
                fout << labels[i] << " ";
                g.add_vertex(labels[i]);
            }
            fout << endl;
            fout << (n * (n - 1) / 2) << endl;
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    int edge = rand() % 9 + 1;
                    fout << labels[i] << " " << labels[j] << " " << edge << endl;
                    g.set_edge(labels[i], labels[j], edge);
                }
            }
            vector<int> path;
            int min_distance;
            bruteforce_min(g, path, min_distance);
            for (auto it : path) {
                fout << labels[it] << " ";
            }
            fout << endl;
            fout << min_distance;
            fout.close();
            cout << "Graph " << n << " vertices (#" << x << ")" << endl;
        }
    }
    return 0;
}