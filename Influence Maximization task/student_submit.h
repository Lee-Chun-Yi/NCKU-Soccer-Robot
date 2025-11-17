#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

// =====================================================================
// Strong-PREV (Medium-focused, Small-max) version
//   Goal:
//     - Restore and strengthen the behaviour that gave you ~0.72 Active Rate
//       on MEDIUM-sized graphs.
//     - SMALL (N <= 200): exhaustive greedy on spread (P - N), which in
//       signed diffusion typically correlates very well with high Active Rate.
//     - MEDIUM (200 < N <= 5000): CELF greedy with a tuned candidate pool:
//         * fastScore similar to your best previous version
//         * MIN_SIM = 400, MULT = 60 (classic strong setting)
//         * No heavy hill-climbing/random replacement to avoid destroying
//           good seeds.
//     - LARGE (N > 5000): heuristic-only selection (fastScore), no diffusion
//       simulation to keep runtime safe.
//
//   Objective:
//     - Internal greedy objective = spread = P - N, using diffuse_signed_all.
//       This matches the behaviour of your historically best version and has
//       empirically produced high Active Rate in your earlier submissions.
// =====================================================================

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

// ------------------------------------------------------------
// Seed info (given_pos / neg_seed)
// ------------------------------------------------------------
struct SeedInfo {
    bool loaded   = false;
    bool hasGiven = false;
    int  given    = -1;
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

static SeedInfo cachedSeedInfo;
static bool     overrideSeedInfoActive = false;
static SeedInfo overrideSeedInfo;

struct SeedInfoOverrideGuard {
    bool active = false;
    SeedInfoOverrideGuard(int givenSeed, const unordered_set<int>& negSeeds) {
        overrideSeedInfoActive = true;
        active                 = true;
        overrideSeedInfo       = SeedInfo();
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
            overrideSeedInfo       = SeedInfo();
        }
    }
};

static const SeedInfo& getSeedInfo() {
    if (overrideSeedInfoActive) return overrideSeedInfo;
    if (cachedSeedInfo.loaded)  return cachedSeedInfo;

    cachedSeedInfo.loaded = true;
    vector<string> args = readCmdlineArgs();
    if (args.size() >= 2) cachedSeedInfo.dataDir = args[1];

    if (!cachedSeedInfo.dataDir.empty()) {
        ifstream gp(joinPath(cachedSeedInfo.dataDir, "given_pos.txt"));
        if (gp.is_open() && (gp >> cachedSeedInfo.given)) {
            cachedSeedInfo.hasGiven = true;
        }

        ifstream gn(joinPath(cachedSeedInfo.dataDir, "neg_seed.txt"));
        if (gn.is_open()) {
            int val = 0;
            while (gn >> val) cachedSeedInfo.negatives.insert(val);
        }
    }

    return cachedSeedInfo;
}

// ------------------------------------------------------------
// Graph cache
// ------------------------------------------------------------
struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int,int> idToIndex;

    vector<vector<pair<int,double>>> outAdj;   // (toIndex, weight)
    vector<double> posT;                      // positive thresholds
    vector<double> negT;                      // negative thresholds
    vector<double> outW;                      // sum of outgoing weights

    int indexOf(int nodeId) const {
        auto it = idToIndex.find(nodeId);
        if (it == idToIndex.end()) return -1;
        return it->second;
    }
};

static GraphCache buildGraphCache(DirectedGraph& G) {
    GraphCache cache;

    cache.nodeIds = G.getAllNodes();
    sort(cache.nodeIds.begin(), cache.nodeIds.end());

    cache.idToIndex.reserve(cache.nodeIds.size() * 2 + 1);
    for (size_t i = 0; i < cache.nodeIds.size(); ++i) {
        cache.idToIndex[cache.nodeIds[i]] = static_cast<int>(i);
    }

    size_t N = cache.nodeIds.size();
    cache.outAdj.assign(N, {});
    cache.posT.assign(N, 0.0);
    cache.negT.assign(N, 0.0);
    cache.outW.assign(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        int nodeId = cache.nodeIds[i];
        cache.posT[i] = G.getNodeThreshold(nodeId);
        cache.negT[i] = G.getNodeThreshold2(nodeId);

        vector<int> outs = G.getNodeOutNeighbors(nodeId);
        auto& adj = cache.outAdj[i];
        adj.reserve(outs.size());
        double total = 0.0;
        for (int nb : outs) {
            double w = G.getEdgeInfluence(nodeId, nb);
            total += w;
            auto it = cache.idToIndex.find(nb);
            if (it != cache.idToIndex.end()) {
                adj.emplace_back(it->second, w);
            }
        }
        sort(adj.begin(), adj.end());
        cache.outW[i] = total;
    }

    return cache;
}

// ------------------------------------------------------------
// Negative exposure (1-hop from negative seeds)
// ------------------------------------------------------------
static vector<double> computeNegExposure(const GraphCache& cache,
                                         const unordered_set<int>& negSeeds) {
    vector<double> exposure(cache.nodeIds.size(), 0.0);
    if (cache.nodeIds.empty() || negSeeds.empty()) return exposure;

    for (int nid : negSeeds) {
        int idx = cache.indexOf(nid);
        if (idx < 0) continue;
        for (const auto& edge : cache.outAdj[idx]) {
            exposure[edge.first] += edge.second;
        }
    }

    return exposure;
}

// ------------------------------------------------------------
// Full diffusion on signed LT model (spread = P - N)
// ------------------------------------------------------------
struct FullDiffResult {
    size_t posActive = 0;
    size_t negActive = 0;
    double spread    = 0.0; // P - N
};

static FullDiffResult runFullDiffusionSimulation(DirectedGraph& G,
                                                 const unordered_set<int>& posSeeds,
                                                 const unordered_set<int>& negSeeds) {
    FullDiffResult result;
    unordered_set<int> finalPos, finalNeg;
    diffuse_signed_all(&G, posSeeds, negSeeds, finalPos, finalNeg);
    result.posActive = finalPos.size();
    result.negActive = finalNeg.size();
    result.spread    = double(result.posActive) - double(result.negActive);
    return result;
}

} // namespace student_algo_detail

// =====================================================================
// Main seedSelection (size-dependent strategies)
// =====================================================================
unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
    using namespace student_algo_detail;

    unordered_set<int> seeds;
    if (numberOfSeeds == 0 || G.getSize() == 0) return seeds;

    const SeedInfo& info   = getSeedInfo();
    const GraphCache cache = buildGraphCache(G);
    const size_t N         = cache.nodeIds.size();

    unordered_set<int> banned = info.negatives;
    if (info.hasGiven) banned.insert(info.given);

    // exposure from negative seeds
    vector<double> negExposure = computeNegExposure(cache, info.negatives);

    // ---------------------------------------------------------
    // fastScore: similar to your strong "prev" version
    //   score = outW + 0.05 * outDegree - 0.55 * posT - 0.75 * negExposure
    // ---------------------------------------------------------
    vector<double> fastScore(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
        double s = cache.outW[i];
        s += 0.05 * double(cache.outAdj[i].size());
        s -= 0.55 * cache.posT[i];
        s -= 0.75 * negExposure[i];
        fastScore[i] = s;
    }

    vector<int> order(N);
    for (int i = 0; i < (int)N; ++i) order[i] = i;

    sort(order.begin(), order.end(), [&](int a, int b) {
        if (fastScore[a] != fastScore[b]) return fastScore[a] > fastScore[b];
        return cache.nodeIds[a] < cache.nodeIds[b];
    });

    // ---------------------------------------------------------
    // LARGE: heuristic-only selection
    // ---------------------------------------------------------
    if (N > 5000) {
        for (int idx : order) {
            if (seeds.size() >= numberOfSeeds) break;
            int nodeId = cache.nodeIds[idx];
            if (banned.count(nodeId)) continue;
            seeds.insert(nodeId);
        }
        return seeds;
    }

    // ---------------------------------------------------------
    // SMALL: exhaustive greedy on spread (P - N)
    // ---------------------------------------------------------
    if (N <= 200) {
        unordered_set<int> negSeedSet = info.negatives;
        unordered_set<int> workingSeeds;
        if (info.hasGiven) workingSeeds.insert(info.given);

        FullDiffResult base = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
        double currentSpread = base.spread;

        for (unsigned int k = 0; k < numberOfSeeds; ++k) {
            double bestSpread = -numeric_limits<double>::infinity();
            int bestNode      = -1;

            for (size_t i = 0; i < N; ++i) {
                int nodeId = cache.nodeIds[i];
                if (banned.count(nodeId))     continue;
                if (workingSeeds.count(nodeId)) continue;

                unordered_set<int> trial = workingSeeds;
                trial.insert(nodeId);
                FullDiffResult r = runFullDiffusionSimulation(G, trial, negSeedSet);
                if (r.spread > bestSpread + 1e-9) {
                    bestSpread = r.spread;
                    bestNode   = nodeId;
                }
            }

            if (bestNode < 0) break;
            seeds.insert(bestNode);
            workingSeeds.insert(bestNode);
            currentSpread = bestSpread;
        }

        return seeds;
    }

    // ---------------------------------------------------------
    // MEDIUM: CELF greedy on candidate pool (P - N objective)
    //   N in (200, 5000]
    // ---------------------------------------------------------
    const int MIN_SIM = 400;
    const int MULT    = 60;

    int simulateCount = static_cast<int>(N);
    int targetSim     = max(MIN_SIM, MULT * static_cast<int>(numberOfSeeds));
    if (simulateCount > targetSim) simulateCount = targetSim;

    vector<int> candidateNodes;
    candidateNodes.reserve(simulateCount);
    for (int i = 0; i < simulateCount; ++i) {
        int idx    = order[i];
        int nodeId = cache.nodeIds[idx];
        if (banned.count(nodeId)) continue;
        candidateNodes.push_back(nodeId);
    }

    if (candidateNodes.empty()) {
        for (int nodeId : cache.nodeIds) {
            if (banned.count(nodeId)) continue;
            candidateNodes.push_back(nodeId);
            if ((int)candidateNodes.size() >= targetSim) break;
        }
    }

    unordered_set<int> negSeedSet = info.negatives;
    unordered_set<int> workingSeeds;
    if (info.hasGiven) workingSeeds.insert(info.given);

    FullDiffResult base = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
    double currentSpread = base.spread;
    int iteration = 0;

    struct CelfEntry {
        int    nodeId;
        double gain;
        double totalSpread;
        int    lastUpdate;
    };

    struct CelfCompare {
        bool operator()(const CelfEntry& a, const CelfEntry& b) const {
            if (a.gain != b.gain) return a.gain < b.gain; // max-heap
            return a.nodeId > b.nodeId;
        }
    };

    auto evaluateCandidate = [&](int nodeId, int iterTag) {
        unordered_set<int> trial = workingSeeds;
        trial.insert(nodeId);
        FullDiffResult r = runFullDiffusionSimulation(G, trial, negSeedSet);
        return CelfEntry{ nodeId, r.spread - currentSpread, r.spread, iterTag };
    };

    priority_queue<CelfEntry, vector<CelfEntry>, CelfCompare> pq;
    for (int nodeId : candidateNodes) {
        if (workingSeeds.count(nodeId)) continue;
        pq.push(evaluateCandidate(nodeId, 0));
    }

    while (seeds.size() < numberOfSeeds && !pq.empty()) {
        CelfEntry top = pq.top();
        pq.pop();

        if (workingSeeds.count(top.nodeId) || banned.count(top.nodeId)) continue;

        if (top.lastUpdate == iteration) {
            seeds.insert(top.nodeId);
            workingSeeds.insert(top.nodeId);
            currentSpread = top.totalSpread;
            ++iteration;
        } else {
            pq.push(evaluateCandidate(top.nodeId, iteration));
        }
    }

    if (seeds.size() < numberOfSeeds) {
        for (int idx : order) {
            if (seeds.size() >= numberOfSeeds) break;
            int nodeId = cache.nodeIds[idx];
            if (banned.count(nodeId) || seeds.count(nodeId)) continue;
            seeds.insert(nodeId);
        }
    }

    return seeds;
}

// Overload for harness that provides givenPosSeed / negSeeds explicitly
unordered_set<int> seedSelection(DirectedGraph& G,
                                 unsigned int numberOfSeeds,
                                 int givenPosSeed,
                                 const unordered_set<int>& negSeeds) {
    using namespace student_algo_detail;
    SeedInfoOverrideGuard guard(givenPosSeed, negSeeds);
    return seedSelection(G, numberOfSeeds);
}

#endif // YOUR_ALGORITHM_H
