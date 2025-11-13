#include <iostream>
#include <fstream>
#include <iomanip>
#include <unordered_set>
#include "graph.h"
#include "LT.h"
using namespace std;

#define TOTAL_POS_SEEDS 10  // 總共要有 10 個 positive seeds（1 個給定 + 9 個學生挑）

DirectedGraph loadGraph(string);
double activeRate(DirectedGraph&, string);

int main(int argc, char* argv[]) {
    DirectedGraph G = loadGraph(argv[1]);
    double active_rate_of_my_algorithm = activeRate(G, argv[1]);
    cout << "Active_Rate=" << fixed << setprecision(2) << active_rate_of_my_algorithm << endl;
    return 0;
}

/* Load network from given data directory which contains two files: "node.txt" and "edge.txt" and return the loaded network */
DirectedGraph loadGraph(string dataFileDirectory) {
    DirectedGraph G = DirectedGraph();

    // ===== node.txt: 第一行 N，之後每行 id pos_th neg_th =====
    string node_file_path = dataFileDirectory + "/node.txt";
    ifstream f1(node_file_path);
    if (!f1.is_open()) {
        cerr << "Cannot open node file: " << node_file_path << endl;
        exit(EXIT_FAILURE);
    }
    int node_count;
    f1 >> node_count; // 讀掉第一行的節點數
    int node;
    double pos_th, neg_th;
    while (f1 >> node >> pos_th >> neg_th) {
        G.addNode(node, pos_th, neg_th);
    }
    f1.close();

    // ===== edge.txt: 舊格式不變 =====
    string edge_file_path = dataFileDirectory + "/edge.txt";
    ifstream f2(edge_file_path);
    if (!f2.is_open()) {
        cerr << "Cannot open edge file: " << edge_file_path << endl;
        exit(EXIT_FAILURE);
    }
    string line;
    f2 >> line; // 讀掉第一個 token（通常是 edge 數目）
    while (f2 >> line) {
        int from_node = stoi(line);
        f2 >> line;
        int to_node = stoi(line);
        f2 >> line;
        double influence = stod(line);
        G.addEdge(from_node, to_node, influence);
    }
    f2.close();

    return G;
}

/* Run diffusion in given DirectedGraph G with initial active users and return score = (#positive_active - #negative_active)/total nodes */
double activeRate(DirectedGraph& G, string test_file) {
    using std::unordered_set;

    // ===== 讀 given_pos.txt：1 個正向 seed =====
    string given_pos_file = test_file + "/given_pos.txt";
    ifstream gp(given_pos_file);
    if (!gp.is_open()) {
        cerr << "Cannot open given_pos file: " << given_pos_file << endl;
        exit(EXIT_FAILURE);
    }
    int given_pos_seed;
    gp >> given_pos_seed;
    gp.close();

    // ===== 讀 neg_seed.txt：10 個負向 seeds =====
    string neg_seed_file = test_file + "/neg_seed.txt";
    ifstream gn(neg_seed_file);
    if (!gn.is_open()) {
        cerr << "Cannot open neg_seed file: " << neg_seed_file << endl;
        exit(EXIT_FAILURE);
    }
    unordered_set<int> neg_seeds;
    int x;
    while (gn >> x) {
        neg_seeds.insert(x);
    }
    gn.close();

    // ===== 呼叫學生的 seedSelection：選 9 個額外正向 seeds =====
    unsigned int extraPosSeeds = TOTAL_POS_SEEDS - 1; // 扣掉 given_pos
    unordered_set<int> studentSeeds = seedSelection(G, extraPosSeeds);
    if (studentSeeds.size() != extraPosSeeds) {
        vector<int> v(studentSeeds.begin(), studentSeeds.end());
        cerr << "Seed set size invalid\nNeed :" << extraPosSeeds
             << "\nGet :" << studentSeeds.size() << endl;
        cerr << "Your seeds: ";
        for (size_t i = 0; i < v.size(); ++i)
            cerr << v[i] << ' ';
        cerr << endl;
        exit(EXIT_FAILURE);
    }

    // ===== 合併正向 seeds（1 個給定 + 學生挑的 9 個）=====
    unordered_set<int> pos_seeds;
    pos_seeds.insert(given_pos_seed);
    pos_seeds.insert(studentSeeds.begin(), studentSeeds.end());
    if (pos_seeds.size() != TOTAL_POS_SEEDS) {
        cerr << "Total positive seed size invalid\nNeed :" << TOTAL_POS_SEEDS
             << "\nGet :" << pos_seeds.size() << endl;
        exit(EXIT_FAILURE);
    }

    // ===== 做 signed LT 擴散 =====
    unordered_set<int> final_pos, final_neg;
    diffuse_signed_all(&G, pos_seeds, neg_seeds, final_pos, final_neg);

    // 原始分數：(#positive - #negative)
    int score = static_cast<int>(final_pos.size()) -
                static_cast<int>(final_neg.size());

    // 圖的節點總數（用你 graph.h 的 getSize；沒的話就用 G.getAllNodes().size()）
    int N = G.getSize();
    if (N == 0) return 0.0;

    double ar = static_cast<double>(score) / static_cast<double>(N);

    // Debug 安全輸出（只寫到 stderr，不影響評分）
    cerr << "[Debug] Dataset: " << test_file
         << " | PosActive=" << final_pos.size()
         << " NegActive=" << final_neg.size()
         << " TotalNodes=" << N
         << " RawScore=" << score
         << " ActiveRate=" << ar << endl;

    // ===== 回傳新的 Active rate =====
    return ar;
}

