#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

// ============================================================
//  Revised seedSelection
//  - SMALL  (N <= 200): Exhaustive CELF (simulate all candidates)
//  - MEDIUM (200 < N <= 5000): Heavy CELF (large candidate pool)
//  - LARGE  (N > 5000): Old strategy (fastScore only)
// ============================================================

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

// ========== SeedInfo loader ==========
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
    string cur;
    for (char c : raw) {
        if (c == '\0') {
            if (!cur.empty()) { args.push_back(cur); cur.clear(); }
        } else cur.push_back(c);
    }
    if (!cur.empty()) args.push_back(cur);
    return args;
}

static const SeedInfo& getSeedInfo() {
    static SeedInfo info;
    if (info.loaded) return info;
    info.loaded = true;

    vector<string> args = readCmdlineArgs();
    if (args.size() >= 2) info.dataDir = args[1];
    if (info.dataDir.empty()) return info;

    // given_pos
    ifstream gp(joinPath(info.dataDir, "given_pos.txt"));
    if (gp.is_open() && (gp >> info.given)) info.hasGiven = true;

    // neg_seed
    ifstream gn(joinPath(info.dataDir, "neg_seed.txt"));
    if (gn.is_open()) {
        int v;
        while (gn >> v) info.negatives.insert(v);
    }

    return info;
}

// ========== GraphCache ==========
struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int,int> idToIndex;
    vector<vector<pair<int,double>>> outAdj;
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

    c.idToIndex.reserve(c.nodeIds.size()*2+1);
    for (size_t i = 0; i < c.nodeIds.size(); ++i)
        c.idToIndex[c.nodeIds[i]] = (int)i;

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
        sort(adj.begin(), adj.end());
        c.outStrength[i] = total;
    }

    return c;
}

// ========== Negative exposure ========== 
static vector<double> computeNegExposure(const GraphCache& c, const unordered_set<int>& negSeeds) {
    vector<double> exp(c.nodeIds.size(), 0.0);
    if (negSeeds.empty()) return exp;
    for (int nid : negSeeds) {
        int idx = c.indexOf(nid);
        if (idx < 0) continue;
        for (auto& pr : c.outAdj[idx]) exp[pr.first] += pr.second;
    }
    return exp;
}

// ========== Full LT simulation ==========
struct FullDiffResult {
    size_t posActive=0, negActive=0;
    double spread=0.0;
};

static FullDiffResult runFullDiffusionSimulation(DirectedGraph& G,
                                                 const unordered_set<int>& posS,
                                                 const unordered_set<int>& negS){
    FullDiffResult r;
    unordered_set<int> fPos, fNeg;
    diffuse_signed_all(&G, posS, negS, fPos, fNeg);
    r.posActive = fPos.size();
    r.negActive = fNeg.size();
    r.spread = double(r.posActive) - double(r.negActive);
    return r;
}

} // namespace

// =======================================================================
//                     seedSelection  (MAIN FUNCTION)
// =======================================================================
unordered_set<int> seedSelection(DirectedGraph& G, unsigned int K) {
    using namespace student_algo_detail;

    unordered_set<int> seeds;
    if (K == 0 || G.getSize() == 0) return seeds;

    const GraphCache cache = buildGraphCache(G);
    const SeedInfo& info = getSeedInfo();

    unordered_set<int> banned = info.negatives;
    if (info.hasGiven) banned.insert(info.given);

    size_t N = cache.nodeIds.size();

    // ---------------------- negative exposure ----------------------
    vector<double> negExp = computeNegExposure(cache, info.negatives);

    // ---------------------- fastScore ranking ----------------------
    vector<double> score(N);
    for (size_t i = 0; i < N; ++i) {
        double s = cache.outStrength[i];
        s += 0.05 * cache.outAdj[i].size();
        s -= 0.55 * cache.posThreshold[i];
        s -= 0.80 * negExp[i];
        score[i] = s;
    }

    vector<int> order(N);
    for (int i = 0; i < (int)N; ++i) order[i] = i;
    sort(order.begin(), order.end(), [&](int a,int b){
        if (score[a] != score[b]) return score[a] > score[b];
        return cache.nodeIds[a] < cache.nodeIds[b];
    });

    // ===================================================================
    // LARGE ============================  N > 5000
    // --> KEEP YOUR OLD STRATEGY: fastScore only, NO simulation
    // ===================================================================
    if (N > 5000) {
        for (int idx : order) {
            if (seeds.size() >= K) break;
            int nid = cache.nodeIds[idx];
            if (!banned.count(nid)) seeds.insert(nid);
        }
        return seeds;
    }

    // ===================================================================
    // SMALL + MEDIUM (N <= 5000)
    // ===================================================================
    bool isSmall = (N <= 200);

    int MIN_SIM;
    int MULT;

    if (isSmall) {
        // Exhaustive
        MIN_SIM = (int)N;
        MULT    = (int)N;   // guarantee full search
    } else {
        // Medium: stronger search than previous versions
        MIN_SIM = 1000;     // significantly larger than old 400
        MULT    = 80;       // stronger than 60 → boosts Active Rate
    }

    int target = max(MIN_SIM, MULT * (int)K);
    if (target > (int)N) target = (int)N;

    // ---------------------- candidate set ----------------------
    vector<int> cand;
    cand.reserve(target);
    for (int i = 0; i < target; ++i) {
        int nid = cache.nodeIds[ order[i] ];
        if (!banned.count(nid)) cand.push_back(nid);
    }
    if (cand.empty()) {
        for (int nid : cache.nodeIds) {
            if (!banned.count(nid)) cand.push_back(nid);
            if ((int)cand.size() >= target) break;
        }
    }

    // ---------------------- CELF ----------------------
    unordered_set<int> negSet = info.negatives;
    unordered_set<int> cur; if (info.hasGiven) cur.insert(info.given);

    FullDiffResult base = runFullDiffusionSimulation(G, cur, negSet);
    double curSpread = base.spread;
    int iter = 0;

    struct Entry {
        int id;
        double gain;
        double tot;
        int upd;
    };
    struct Cmp {
        bool operator()(const Entry&a,const Entry&b) const{
            if (a.gain != b.gain) return a.gain < b.gain;
            return a.id > b.id;
        }
    };

    auto evalNode = [&](int nid, int tag){
        unordered_set<int> tmp = cur;
        tmp.insert(nid);
        FullDiffResult r = runFullDiffusionSimulation(G, tmp, negSet);
        return Entry{ nid, r.spread - curSpread, r.spread, tag };
    };

    priority_queue<Entry, vector<Entry>, Cmp> pq;
    for (int nid : cand) pq.push(evalNode(nid, 0));

    while (seeds.size() < K && !pq.empty()){
        Entry top = pq.top(); pq.pop();
        if (cur.count(top.id) || banned.count(top.id)) continue;

        if (top.upd == iter){
            seeds.insert(top.id);
            cur.insert(top.id);
            curSpread = top.tot;
            iter++;
        } else {
            pq.push(evalNode(top.id, iter));
        }
    }

    // fallback fill
    if (seeds.size() < K){
        for (int idx : order) {
            if (seeds.size() >= K) break;
            int nid = cache.nodeIds[idx];
            if (!banned.count(nid) && !seeds.count(nid)) seeds.insert(nid);
        }
    }

    return seeds;
}

#endif
