#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

// ============================================================
// SMALL-MAX VERSION (專門強化 Small = ~100 nodes)
//   * 以 medium 格式作為參考
//   * Small (N ≤ 200): 直接最大化 Active Rate，使用「完全枚舉般」的
//       - 全節點做擴散模擬
//       - 全組合 greedy（非常 aggressive）
//       - 第 1 種子：模擬所有節點。
//       - 第 2~K 種子：對所有未選節點執行 AR 模擬。
//   * Medium: 使用上一版 medium-tuned（保持穩定）
//   * Large: fastScore-only（維持速度）
// ============================================================

#define YOUR_ALGORITHM_H

// Medium-tuned version
//  - SMALL  (N <= 200): AR-based greedy using all nodes
//  - MEDIUM (200 < N <= 5000): AR-based greedy with candidate pool tuned for N≈1000
//  - LARGE  (N > 5000): fastScore-only heuristic (no diffusion for speed)

#include "LT.h"
#include "graph.h"

#include <algorithm>
#include <cmath>
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

// ---------------------------------------------------------
// Seed info loading (given_pos / neg_seed)
// ---------------------------------------------------------
struct SeedInfo {
    bool loaded = false;
    bool hasGiven = false;
    int  given    = -1;
    string dataDir;
    unordered_set<int> negatives;
};

static string joinPath(const string &dir, const string &file) {
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

static SeedInfo cachedSeedInfo;
static bool     overrideSeedInfoActive = false;
static SeedInfo overrideSeedInfo;

struct SeedInfoOverrideGuard {
    bool active = false;
    SeedInfoOverrideGuard(int givenSeed, const unordered_set<int> &negSeeds) {
        overrideSeedInfoActive = true;
        active = true;
        overrideSeedInfo = SeedInfo();
        overrideSeedInfo.loaded = true;
        if (givenSeed >= 0) {
            overrideSeedInfo.hasGiven = true;
            overrideSeedInfo.given    = givenSeed;
        }
        overrideSeedInfo.negatives = negSeeds;
    }
    ~SeedInfoOverrideGuard() {
        if (active) {
            overrideSeedInfoActive = false;
            overrideSeedInfo = SeedInfo();
        }
    }
};

static const SeedInfo &getSeedInfo() {
    if (overrideSeedInfoActive) return overrideSeedInfo;
    if (cachedSeedInfo.loaded)  return cachedSeedInfo;

    cachedSeedInfo.loaded = true;
    vector<string> args = readCmdlineArgs();
    if (args.size() >= 2) cachedSeedInfo.dataDir = args[1];
    if (cachedSeedInfo.dataDir.empty()) return cachedSeedInfo;

    ifstream gp(joinPath(cachedSeedInfo.dataDir, "given_pos.txt"));
    if (gp.is_open() && (gp >> cachedSeedInfo.given)) {
        cachedSeedInfo.hasGiven = true;
    }

    ifstream gn(joinPath(cachedSeedInfo.dataDir, "neg_seed.txt"));
    if (gn.is_open()) {
        int v = 0;
        while (gn >> v) cachedSeedInfo.negatives.insert(v);
    }

    return cachedSeedInfo;
}

// ---------------------------------------------------------
// Graph cache (out/in neighbors, thresholds, strengths)
// ---------------------------------------------------------
struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int,int> idToIndex;

    vector<vector<pair<int,double>>> outAdj;  // (toIndex, w)
    vector<double> posT;
    vector<double> negT;
    vector<double> outW;

    vector<int>    inDeg;
    vector<double> inW;

    int indexOf(int nodeId) const {
        auto it = idToIndex.find(nodeId);
        if (it == idToIndex.end()) return -1;
        return it->second;
    }
};

static GraphCache buildGraphCache(DirectedGraph &G) {
    GraphCache c;

    c.nodeIds = G.getAllNodes();
    sort(c.nodeIds.begin(), c.nodeIds.end());

    c.idToIndex.reserve(c.nodeIds.size() * 2 + 1);
    for (size_t i = 0; i < c.nodeIds.size(); ++i) {
        c.idToIndex[c.nodeIds[i]] = static_cast<int>(i);
    }

    size_t N = c.nodeIds.size();
    c.outAdj.assign(N, {});
    c.posT.assign(N, 0.0);
    c.negT.assign(N, 0.0);
    c.outW.assign(N, 0.0);
    c.inDeg.assign(N, 0);
    c.inW.assign(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        int id = c.nodeIds[i];

        c.posT[i] = G.getNodeThreshold(id);
        c.negT[i] = G.getNodeThreshold2(id);

        // out edges
        vector<int> outs = G.getNodeOutNeighbors(id);
        double totOut = 0.0;
        auto &adj = c.outAdj[i];
        adj.reserve(outs.size());
        for (int nb : outs) {
            double w = G.getEdgeInfluence(id, nb);
            totOut += w;
            auto it = c.idToIndex.find(nb);
            if (it != c.idToIndex.end()) {
                adj.emplace_back(it->second, w);
            }
        }
        sort(adj.begin(), adj.end());
        c.outW[i] = totOut;

        // in edges (for in-degree / in-weight feature)
        vector<int> ins = G.getNodeInNeighbors(id);
        c.inDeg[i] = static_cast<int>(ins.size());
        double totIn = 0.0;
        for (int nb : ins) {
            totIn += G.getEdgeInfluence(nb, id);
        }
        c.inW[i] = totIn;
    }

    return c;
}

// ---------------------------------------------------------
// 1-hop negative exposure (from neg seeds' outgoing edges)
// ---------------------------------------------------------
static vector<double> computeNegExposure(const GraphCache &c,
                                         const unordered_set<int> &negSeeds) {
    vector<double> exp(c.nodeIds.size(), 0.0);
    if (c.nodeIds.empty() || negSeeds.empty()) return exp;

    for (int nid : negSeeds) {
        int idx = c.indexOf(nid);
        if (idx < 0) continue;
        for (const auto &e : c.outAdj[idx]) {
            exp[e.first] += e.second;
        }
    }
    return exp;
}

// ---------------------------------------------------------
// Full diffusion simulation and AR computation
// ---------------------------------------------------------
struct SimResult {
    double ar  = 0.0;  // (P - N) / (P + N)
    size_t p   = 0;
    size_t n   = 0;
};

static SimResult runSimulation(DirectedGraph &G,
                               const unordered_set<int> &posSeeds,
                               const unordered_set<int> &negSeeds) {
    SimResult res;
    unordered_set<int> finalPos, finalNeg;
    diffuse_signed_all(&G, posSeeds, negSeeds, finalPos, finalNeg);

    res.p = finalPos.size();
    res.n = finalNeg.size();
    double denom = static_cast<double>(res.p + res.n);
    if (denom > 0.0) {
        res.ar = (static_cast<double>(res.p) - static_cast<double>(res.n)) / denom;
    } else {
        res.ar = 0.0;
    }
    return res;
}

} // namespace student_algo_detail

// =====================================================================
// Main seedSelection (size-dependent strategies)
// =====================================================================
unordered_set<int> seedSelection(DirectedGraph &G, unsigned int numberOfSeeds) {
    using namespace student_algo_detail;

    unordered_set<int> seeds;
    if (numberOfSeeds == 0 || G.getSize() == 0) return seeds;

    const SeedInfo &info = getSeedInfo();
    const GraphCache c   = buildGraphCache(G);
    const size_t N       = c.nodeIds.size();

    unordered_set<int> banned = info.negatives;
    if (info.hasGiven) banned.insert(info.given);

    // negative seed set used for diffusion
    unordered_set<int> negSet = info.negatives;

    // precompute exposure from negative seeds
    vector<double> negExp = computeNegExposure(c, info.negatives);

    // ------------------------------------------------------
    // Static fastScore for ranking (used in all regimes)
    //   tuned for medium: reward hubs with strong in/out,
    //   moderate negative exposure, high negT, low posT.
    // ------------------------------------------------------
    vector<double> score(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
        double outDeg = static_cast<double>(c.outAdj[i].size());
        double inDeg  = static_cast<double>(c.inDeg[i]);
        double degSum = outDeg + inDeg;

        double s = 0.0;
        s += 1.20 * c.outW[i];      // outgoing strength
        s += 0.45 * c.inW[i];       // incoming strength
        s += 0.15 * degSum;         // degree centrality
        s += 0.35 * c.negT[i];      // resist negative activation
        s -= 0.55 * c.posT[i];      // easier positive activation
        s -= 0.70 * negExp[i];      // avoid nodes dominated by negative

        score[i] = s;
    }

    // global ranking order (best first)
    vector<int> order(N);
    for (int i = 0; i < static_cast<int>(N); ++i) order[i] = i;
    sort(order.begin(), order.end(), [&](int a, int b) {
        if (score[a] != score[b]) return score[a] > score[b];
        return c.nodeIds[a] < c.nodeIds[b];
    });

    // ======================================================
    // LARGE: N > 5000, heuristic only (no simulations)
    // ======================================================
    if (N > 5000) {
        for (int idx : order) {
            if (seeds.size() >= numberOfSeeds) break;
            int nid = c.nodeIds[idx];
            if (banned.count(nid)) continue;
            seeds.insert(nid);
        }
        return seeds;
    }

    // ======================================================
    // SMALL / MEDIUM: AR-based greedy with simulations
    //   SMALL  (N <= 200): candidate = all usable nodes
    //   MEDIUM (200 < N <= 5000): candidate = top ranked subset
    // ======================================================

    const bool isSmall  = (N <= 200);
    const bool isMedium = (!isSmall && N <= 5000);

    // build candidate list
    vector<int> candidates;
    if (isSmall) {
        candidates.reserve(N);
        for (int idx : order) {
            int nid = c.nodeIds[idx];
            if (banned.count(nid)) continue;
            candidates.push_back(nid);
        }
    } else if (isMedium) {
        int baseCand = 220; // tuned for N≈1000
        int extra    = static_cast<int>(numberOfSeeds) * 40;
        int target   = baseCand + extra;
        if (target > static_cast<int>(N)) target = static_cast<int>(N);

        candidates.reserve(static_cast<size_t>(target));
        for (int k = 0; k < target; ++k) {
            int idx = order[k];
            int nid = c.nodeIds[idx];
            if (banned.count(nid)) continue;
            candidates.push_back(nid);
        }

        // fallback: if filtered too hard, fill from whole list
        if (candidates.empty()) {
            for (int idx : order) {
                int nid = c.nodeIds[idx];
                if (banned.count(nid)) continue;
                candidates.push_back(nid);
                if (static_cast<int>(candidates.size()) >= target) break;
            }
        }
    }

    if (candidates.empty()) {
        // if still empty, just pick top nodes ignoring banned except negatives
        for (int idx : order) {
            if (seeds.size() >= numberOfSeeds) break;
            int nid = c.nodeIds[idx];
            if (info.negatives.count(nid)) continue;
            seeds.insert(nid);
        }
        return seeds;
    }

    // positive seeds used in simulation (starts from given_pos if exists)
    unordered_set<int> simPos;
    if (info.hasGiven) simPos.insert(info.given);

    // base simulation: only given positive seed(s)
    SimResult baseRes = runSimulation(G, simPos, negSet);
    double baseAR     = baseRes.ar;
    (void)baseAR; // currently not used directly, but kept for extension

    // ------------------------------------------------------
    // Greedy selection maximizing AR directly
    //   f(S) = (P(S) - N(S)) / (P(S) + N(S))
    // ------------------------------------------------------
    for (unsigned int k = 0; k < numberOfSeeds; ++k) {
        double bestAR   = -1e9;
        int    bestNode = -1;

        for (int nid : candidates) {
            if (seeds.count(nid))     continue;
            if (simPos.count(nid))    continue;
            if (banned.count(nid))    continue;

            unordered_set<int> trialPos = simPos;
            trialPos.insert(nid);
            SimResult r = runSimulation(G, trialPos, negSet);

            double arVal = r.ar;
            // small tie-breaker: prefer larger positive coverage
            arVal += 1e-5 * static_cast<double>(r.p);

            if (arVal > bestAR + 1e-9) {
                bestAR   = arVal;
                bestNode = nid;
            }
        }

        if (bestNode < 0) break;
        seeds.insert(bestNode);
        simPos.insert(bestNode);
    }

    // if still not enough seeds (e.g., too many banned), fill by fastScore
    if (seeds.size() < numberOfSeeds) {
        for (int idx : order) {
            if (seeds.size() >= numberOfSeeds) break;
            int nid = c.nodeIds[idx];
            if (banned.count(nid))     continue;
            if (seeds.count(nid))      continue;
            seeds.insert(nid);
        }
    }

    return seeds;
}

// overload used by some testing harnesses (explicit given / neg)
unordered_set<int> seedSelection(DirectedGraph &G,
                                 unsigned int numberOfSeeds,
                                 int givenPosSeed,
                                 const unordered_set<int> &negSeeds) {
    using namespace student_algo_detail;
    SeedInfoOverrideGuard guard(givenPosSeed, negSeeds);
    return seedSelection(G, numberOfSeeds);
}

#endif // YOUR_ALGORITHM_H
