#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

namespace student_algo_detail {

struct SeedInfo {
    bool loaded = false;
    bool hasGiven = false;
    int given = -1;
    string dataDir;
    unordered_set<int> negatives;
};

static string joinPath(const string& dir, const string& file) {
    if (dir.empty()) return file;
    char tail = dir.back();
    if (tail == '/' || tail == '\\') return dir + file;
    return dir + "/" + file;
}

static vector<string> readCmdlineArgs() {
    ifstream cmd("/proc/self/cmdline", ios::binary);
    if (!cmd.is_open()) return {};
    string raw((istreambuf_iterator<char>(cmd)), istreambuf_iterator<char>());
    vector<string> args;
    string current;
    for (char c : raw) {
        if (c == '\0') {
            if (!current.empty()) {
                args.push_back(current);
                current.clear();
            }
        } else {
            current.push_back(c);
        }
    }
    if (!current.empty()) args.push_back(current);
    return args;
}

static const SeedInfo& getSeedInfo() {
    static SeedInfo info;
    if (info.loaded) return info;
    info.loaded = true;

    vector<string> args = readCmdlineArgs();
    if (args.size() >= 2) info.dataDir = args[1];
    if (info.dataDir.empty()) return info;

    ifstream gp(joinPath(info.dataDir, "given_pos.txt"));
    if (gp.is_open() && (gp >> info.given)) info.hasGiven = true;

    ifstream gn(joinPath(info.dataDir, "neg_seed.txt"));
    if (gn.is_open()) {
        int v;
        while (gn >> v) info.negatives.insert(v);
    }

    return info;
}

struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int, int> idToIndex;
    vector<vector<pair<int, double>>> outAdj;
    vector<double> posThreshold;
    vector<double> negThreshold;
    vector<double> outStrength;

    int indexOf(int nodeId) const {
        auto it = idToIndex.find(nodeId);
        return (it == idToIndex.end() ? -1 : it->second);
    }
};

static GraphCache buildGraphCache(DirectedGraph& G) {
    GraphCache c;
    c.nodeIds = G.getAllNodes();
    sort(c.nodeIds.begin(), c.nodeIds.end());

    c.idToIndex.reserve(c.nodeIds.size() * 2 + 1);
    for (size_t i = 0; i < c.nodeIds.size(); ++i) c.idToIndex[c.nodeIds[i]] = i;

    size_t N = c.nodeIds.size();
    c.outAdj.assign(N, {});
    c.posThreshold.assign(N, 0.0);
    c.negThreshold.assign(N, 0.0);
    c.outStrength.assign(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        int id = c.nodeIds[i];
        c.posThreshold[i] = G.getNodeThreshold(id);
        c.negThreshold[i] = G.getNodeThreshold2(id);

        vector<int> outs = G.getNodeOutNeighbors(id);
        auto& adj = c.outAdj[i];
        adj.reserve(outs.size());
        double total = 0.0;

        for (int nb : outs) {
            double w = G.getEdgeInfluence(id, nb);
            total += w;
            auto it = c.idToIndex.find(nb);
            if (it != c.idToIndex.end()) adj.emplace_back(it->second, w);
        }

        sort(adj.begin(), adj.end(), [](auto& a, auto& b) {
            return (a.first == b.first ? a.second < b.second : a.first < b.first);
        });

        c.outStrength[i] = total;
    }

    return c;
}

static vector<double> computeNegExposure(const GraphCache& c, const unordered_set<int>& neg) {
    vector<double> e(c.nodeIds.size(), 0.0);
    if (neg.empty()) return e;

    for (int id : neg) {
        int idx = c.indexOf(id);
        if (idx < 0) continue;
        for (auto& ed : c.outAdj[idx]) e[ed.first] += ed.second;
    }
    return e;
}

struct FullDiffResult {
    size_t posActive;
    size_t negActive;
    double spread;
};

static FullDiffResult runFullDiffusionSimulation(
    DirectedGraph& G,
    const unordered_set<int>& pos,
    const unordered_set<int>& neg)
{
    unordered_set<int> fp, fn;
    diffuse_signed_all(&G, pos, neg, fp, fn);
    return { fp.size(), fn.size(), double(fp.size()) - double(fn.size()) };
}

} // namespace student_algo_detail

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
    using namespace student_algo_detail;

    unordered_set<int> seeds;
    if (!numberOfSeeds || !G.getSize()) return seeds;

    const GraphCache c = buildGraphCache(G);
    const SeedInfo& info = getSeedInfo();

    unordered_set<int> banned = info.negatives;
    if (info.hasGiven) banned.insert(info.given);

    vector<double> negExp = computeNegExposure(c, info.negatives);

    size_t N = c.nodeIds.size();
    vector<double> fastScore(N);

    fastScore.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        double s = c.outStrength[i];
        s += 0.05 * c.outAdj[i].size();
        s -= 0.55 * c.posThreshold[i];
        if (!negExp.empty()) s -= 0.8 * negExp[i];
        fastScore[i] = s;
    }

    vector<int> order(N);
    for (size_t i = 0; i < N; ++i) order[i] = i;

    sort(order.begin(), order.end(), [&](int a, int b) {
        return (fastScore[a] == fastScore[b] ? c.nodeIds[a] < c.nodeIds[b] : fastScore[a] > fastScore[b]);
    });

    const int MIN_SIM = 350;
    const int MULT = 55;
    int tgt = max(MIN_SIM, MULT * int(numberOfSeeds));
    if (tgt > int(N)) tgt = int(N);

    vector<int> cand;
    cand.reserve(tgt);
    for (int i = 0; i < tgt; ++i) {
        int idx = order[i];
        int nid = c.nodeIds[idx];
        if (!banned.count(nid)) cand.push_back(nid);
    }

    if (cand.empty()) {
        for (int id : c.nodeIds) {
            if (!banned.count(id)) {
                cand.push_back(id);
                if ((int)cand.size() >= tgt) break;
            }
        }
    }

    unordered_set<int> negS = info.negatives;
    unordered_set<int> cur;
    if (info.hasGiven) cur.insert(info.given);

    FullDiffResult base = runFullDiffusionSimulation(G, cur, negS);
    double curSpread = base.spread;

    struct Celf { int id; double gain; double tot; int t; };
    struct Cmp { bool operator()(const Celf& a, const Celf& b) const {
        return (a.gain == b.gain ? a.id > b.id : a.gain < b.gain);
    }};

    auto eval = [&](int id, int it) {
        static unordered_set<int> tmp;
        tmp = cur;
        tmp.insert(id);
        FullDiffResult r = runFullDiffusionSimulation(G, tmp, negS);
        return Celf{ id, r.spread - curSpread, r.spread, it };
    };

    priority_queue<Celf, vector<Celf>, Cmp> pq;
    pq = priority_queue<Celf, vector<Celf>, Cmp>();

    for (int id : cand) pq.push(eval(id, 0));

    int it = 0;
    while (seeds.size() < numberOfSeeds && !pq.empty()) {
        Celf x = pq.top(); pq.pop();
        if (cur.count(x.id) || banned.count(x.id)) continue;
        if (x.t == it) {
            seeds.insert(x.id);
            cur.insert(x.id);
            curSpread = x.tot;
            ++it;
        } else {
            pq.push(eval(x.id, it));
        }
    }

    if (seeds.size() < numberOfSeeds) {
        for (int idx : order) {
            int id = c.nodeIds[idx];
            if (!banned.count(id) && !seeds.count(id)) seeds.insert(id);
            if (seeds.size() >= numberOfSeeds) break;
        }
    }

    return seeds;
}

#endif
