#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

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

struct SeedFiles {
    bool loaded = false;
    string dataDir;
    unordered_set<int> forbidden;
    vector<int> givenPos;
    vector<int> negSeeds;
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
            if (!cur.empty()) {
                args.push_back(cur);
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) args.push_back(cur);
    return args;
}

static vector<int> readIntList(const string& path) {
    vector<int> values;
    ifstream in(path);
    if (!in.is_open()) return values;
    int v = 0;
    while (in >> v) values.push_back(v);
    return values;
}

static const SeedFiles& getSeedFiles() {
    static SeedFiles files;
    if (files.loaded) return files;
    files.loaded = true;

    vector<string> args = readCmdlineArgs();
    if (args.size() >= 2) files.dataDir = args[1];
    if (files.dataDir.empty()) return files;

    vector<int> given = readIntList(joinPath(files.dataDir, "given_pos.txt"));
    vector<int> neg = readIntList(joinPath(files.dataDir, "neg_seed.txt"));

    files.givenPos = given;
    files.negSeeds = neg;

    for (int v : given) files.forbidden.insert(v);
    for (int v : neg) files.forbidden.insert(v);

    return files;
}

struct SpreadStats {
    int pos = 0;
    int neg = 0;
};

struct ScoreContext {
    DirectedGraph* graph = nullptr;
    vector<int> nodes;
    unordered_map<int, int> nodeIndex;
    vector<double> posOutStrength;
    vector<double> negInRisk;
    vector<double> twoHopStrength;
    vector<double> negAdjPenalty;
    unordered_set<int> baseNegSeeds;
    unordered_set<int> forbidden;
    bool ready = false;
};

static ScoreContext gScoreContext;

static int getNodeIndex(int nodeId) {
    if (!gScoreContext.ready) return -1;
    auto it = gScoreContext.nodeIndex.find(nodeId);
    if (it == gScoreContext.nodeIndex.end()) return -1;
    return it->second;
}

static double getNodeRisk(int nodeId) {
    int idx = getNodeIndex(nodeId);
    if (idx < 0) return 0.0;
    double exposure = gScoreContext.negInRisk[idx] + gScoreContext.negAdjPenalty[idx];
    return exposure;
}

static void prepareScoreContext(
        DirectedGraph& G,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden) {
    ScoreContext ctx;
    ctx.graph = &G;
    ctx.baseNegSeeds = baseNeg;
    ctx.forbidden = forbidden;
    ctx.nodes = G.getAllNodes();
    const size_t n = ctx.nodes.size();
    ctx.nodeIndex.reserve(n * 2);
    for (size_t i = 0; i < n; ++i) ctx.nodeIndex[ctx.nodes[i]] = static_cast<int>(i);
    ctx.posOutStrength.assign(n, 0.0);
    ctx.negInRisk.assign(n, 0.0);
    ctx.twoHopStrength.assign(n, 0.0);
    ctx.negAdjPenalty.assign(n, 0.0);

    vector<vector<pair<int, double>>> positiveEdges(n);

    for (size_t idx = 0; idx < n; ++idx) {
        const int nodeId = ctx.nodes[idx];
        const vector<int> outs = G.getNodeOutNeighbors(nodeId);
        for (int nb : outs) {
            double w = G.getEdgeInfluence(nodeId, nb);
            auto nbIt = ctx.nodeIndex.find(nb);
            if (w > 0) {
                ctx.posOutStrength[idx] += w;
                if (nbIt != ctx.nodeIndex.end()) positiveEdges[idx].emplace_back(nbIt->second, w);
                if (baseNeg.count(nb)) ctx.negAdjPenalty[idx] += 0.8 + 0.4 * w;
            } else if (w < 0) {
                ctx.negAdjPenalty[idx] += std::min(2.0, std::fabs(w));
            }
        }

        const vector<int> ins = G.getNodeInNeighbors(nodeId);
        for (int nb : ins) {
            double w = G.getEdgeInfluence(nb, nodeId);
            if (w < 0) {
                ctx.negInRisk[idx] += std::fabs(w);
                if (baseNeg.count(nb)) ctx.negAdjPenalty[idx] += 0.6 + 0.4 * std::fabs(w);
            } else if (baseNeg.count(nb)) {
                ctx.negAdjPenalty[idx] += 0.5;
            }
        }
    }

    for (size_t idx = 0; idx < n; ++idx) {
        double accum = 0.0;
        for (const auto& edge : positiveEdges[idx]) {
            double neighborStrength = ctx.posOutStrength[edge.first];
            double bounded = std::min(neighborStrength, 12.0);
            accum += edge.second * (0.35 + 0.02 * bounded);
        }
        ctx.twoHopStrength[idx] = accum;
    }

    ctx.ready = true;
    gScoreContext = std::move(ctx);
}

static double computeNodeScore(DirectedGraph& G, int nodeId) {
    if (!gScoreContext.ready || gScoreContext.graph != &G) return 0.0;
    int idx = getNodeIndex(nodeId);
    if (idx < 0) return 0.0;

    const double pos = gScoreContext.posOutStrength[idx];
    const double twoHop = gScoreContext.twoHopStrength[idx];
    const double negRisk = gScoreContext.negInRisk[idx];
    const double adjPenalty = gScoreContext.negAdjPenalty[idx];

    double score = 1.4 * pos + 0.5 * twoHop - 1.2 * negRisk - 0.8 * adjPenalty;
    score += 0.25 / (1.0 + negRisk);
    return score;
}

struct CandidateEntry {
    int node = -1;
    double gain = numeric_limits<double>::lowest();
    size_t last_update = static_cast<size_t>(-1);
    SpreadStats stats;
};

struct CandidateComparator {
    bool operator()(const CandidateEntry& a, const CandidateEntry& b) const {
        if (a.gain == b.gain) return a.node > b.node;
        return a.gain < b.gain;
    }
};

static constexpr int SIMULATION_LIMIT = 300;
static int gSimulationCount = 0;

static void resetSimulationBudget() { gSimulationCount = 0; }

static SpreadStats simulateSpread(
        DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& selected,
        int extraNode) {
    if (gSimulationCount >= SIMULATION_LIMIT) {
        return SpreadStats{0, 60};
    }
    ++gSimulationCount;

    static unordered_set<int> posSeeds;
    static unordered_set<int> negSeeds;

    posSeeds.clear();
    posSeeds.insert(basePos.begin(), basePos.end());
    posSeeds.insert(selected.begin(), selected.end());
    if (extraNode >= 0) posSeeds.insert(extraNode);

    negSeeds.clear();
    negSeeds.insert(baseNeg.begin(), baseNeg.end());

    unordered_set<int> posActive = posSeeds;
    unordered_set<int> negActive = negSeeds;

    SpreadStats stats;
    stats.pos = static_cast<int>(posActive.size());
    stats.neg = static_cast<int>(negActive.size());

    while (true) {
        auto result = diffuse_one_round_signed(&G, posActive, negActive);
        const auto& newPos = result.first;
        const auto& newNeg = result.second;
        if (newPos.empty() && newNeg.empty()) break;
        posActive.insert(newPos.begin(), newPos.end());
        negActive.insert(newNeg.begin(), newNeg.end());
        stats.pos = static_cast<int>(posActive.size());
        stats.neg = static_cast<int>(negActive.size());
        if (stats.neg > stats.pos * 2 || stats.neg > 10) {
            stats.pos = std::max(0, stats.pos - 5);
            stats.neg += 25;
            return stats;
        }
        if (stats.pos + stats.neg >= G.getSize() * 2) break;
    }

    return stats;
}

static void fillWithHeuristics(const vector<pair<int, double>>& ranking,
        const vector<int>& allNodes,
        const unordered_set<int>& forbidden,
        unordered_set<int>& selected,
        unsigned int numberOfSeeds) {
    const double riskLimit = 8.0;
    for (const auto& item : ranking) {
        if (selected.size() >= numberOfSeeds) break;
        if (forbidden.count(item.first)) continue;
        if (selected.count(item.first)) continue;
        if (getNodeRisk(item.first) > riskLimit) continue;
        selected.insert(item.first);
    }

    if (selected.size() >= numberOfSeeds) return;

    vector<pair<double, int>> fallback;
    fallback.reserve(allNodes.size());
    for (int node : allNodes) {
        if (forbidden.count(node)) continue;
        if (selected.count(node)) continue;
        double risk = getNodeRisk(node);
        if (risk > riskLimit * 1.2) continue;
        int idx = getNodeIndex(node);
        double score = (idx >= 0 && gScoreContext.ready) ? gScoreContext.posOutStrength[idx] : 0.0;
        fallback.emplace_back(score, node);
    }
    sort(fallback.begin(), fallback.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first > b.first;
        return a.second < b.second;
    });
    for (const auto& item : fallback) {
        if (selected.size() >= numberOfSeeds) break;
        selected.insert(item.second);
    }

    if (selected.size() >= numberOfSeeds) return;

    for (int node : allNodes) {
        if (selected.size() >= numberOfSeeds) break;
        if (forbidden.count(node)) continue;
        if (selected.count(node)) continue;
        selected.insert(node);
    }
}

static void refineSelection(const vector<pair<int, double>>& ranking,
        const unordered_set<int>& forbidden,
        unsigned int numberOfSeeds,
        unordered_set<int>& selected) {
    if (selected.size() <= numberOfSeeds) {
        if (selected.size() == numberOfSeeds) return;
    }

    vector<pair<double, int>> scored;
    scored.reserve(selected.size());
    DirectedGraph* graphPtr = gScoreContext.graph;
    for (int node : selected) {
        double sc = graphPtr ? computeNodeScore(*graphPtr, node) : 0.0;
        scored.emplace_back(sc, node);
    }
    sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first > b.first;
        return a.second < b.second;
    });

    selected.clear();
    for (const auto& item : scored) {
        if (selected.size() >= numberOfSeeds) break;
        selected.insert(item.second);
    }

    for (const auto& item : ranking) {
        if (selected.size() >= numberOfSeeds) break;
        if (forbidden.count(item.first)) continue;
        selected.insert(item.first);
    }
}

static vector<int> buildCandidatePool(const vector<pair<int, double>>& ranking,
        unsigned int numberOfSeeds,
        const unordered_set<int>& forbidden) {
    vector<int> pool;
    if (ranking.empty()) return pool;
    size_t limit = static_cast<size_t>(numberOfSeeds) * 6;
    limit = std::max<size_t>(limit, 120);
    limit = std::min<size_t>(limit, 200);
    limit = std::min(limit, ranking.size());
    double topScore = ranking.front().second;
    double threshold = topScore * 0.3;
    const double riskLimit = 7.0;

    for (size_t i = 0; i < ranking.size() && pool.size() < limit; ++i) {
        int node = ranking[i].first;
        double score = ranking[i].second;
        if (forbidden.count(node)) continue;
        if (score < threshold && pool.size() + 10 < limit) continue;
        double risk = getNodeRisk(node);
        if (risk > riskLimit) continue;
        int idx = getNodeIndex(node);
        if (idx >= 0 && gScoreContext.negAdjPenalty[idx] > 3.5) continue;
        pool.push_back(node);
    }

    return pool;
}

static unordered_set<int> runSeedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden) {
    unordered_set<int> selected;
    if (numberOfSeeds == 0 || G.getSize() == 0) return selected;

    prepareScoreContext(G, baseNeg, forbidden);
    resetSimulationBudget();

    vector<int> allNodes = gScoreContext.ready ? gScoreContext.nodes : G.getAllNodes();
    if (allNodes.empty()) return selected;

    vector<pair<int, double>> ranking;
    ranking.reserve(allNodes.size());
    for (int node : allNodes) {
        double score = computeNodeScore(G, node);
        ranking.emplace_back(node, score);
    }
    sort(ranking.begin(), ranking.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
        if (a.second != b.second) return a.second > b.second;
        return a.first < b.first;
    });

    vector<int> candidatePool = buildCandidatePool(ranking, numberOfSeeds, forbidden);
    if (candidatePool.empty()) {
        fillWithHeuristics(ranking, allNodes, forbidden, selected, numberOfSeeds);
        refineSelection(ranking, forbidden, numberOfSeeds, selected);
        return selected;
    }

    SpreadStats currentStats = simulateSpread(G, basePos, baseNeg, selected, -1);

    priority_queue<CandidateEntry, vector<CandidateEntry>, CandidateComparator> heap;
    for (int node : candidatePool) {
        CandidateEntry entry;
        entry.node = node;
        entry.gain = numeric_limits<double>::lowest();
        heap.push(entry);
    }

    while (!heap.empty() && selected.size() < numberOfSeeds) {
        if (gSimulationCount >= SIMULATION_LIMIT) break;

        CandidateEntry entry = heap.top();
        heap.pop();

        if (selected.count(entry.node)) continue;

        if (entry.last_update == selected.size()) {
            selected.insert(entry.node);
            currentStats = entry.stats;
            continue;
        }

        SpreadStats stats = simulateSpread(G, basePos, baseNeg, selected, entry.node);
        double dPos = double(stats.pos) - double(currentStats.pos);
        double dNeg = double(stats.neg) - double(currentStats.neg);
        double marginal = dPos * 1.35 - dNeg * 2.2;
        entry.gain = marginal;
        entry.last_update = selected.size();
        entry.stats = stats;

        bool negativeExplosion = stats.neg > stats.pos * 1.5 + 4 || (stats.neg - currentStats.neg) > 10;
        if (entry.gain < -20.0 || negativeExplosion) continue;
        heap.push(entry);
    }

    if (selected.size() < numberOfSeeds) {
        fillWithHeuristics(ranking, allNodes, forbidden, selected, numberOfSeeds);
    }

    refineSelection(ranking, forbidden, numberOfSeeds, selected);

    return selected;
}

} // namespace student_algo_detail

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
    using namespace student_algo_detail;

    if (numberOfSeeds == 0 || G.getSize() == 0) return {};

    const SeedFiles& seedFiles = getSeedFiles();
    unordered_set<int> forbidden = seedFiles.forbidden;

    unordered_set<int> basePos;
    unordered_set<int> baseNeg;
    basePos.reserve(seedFiles.givenPos.size());
    baseNeg.reserve(seedFiles.negSeeds.size());

    for (int v : seedFiles.givenPos)
        if (G.isNodeExist(v)) {
            basePos.insert(v);
            forbidden.insert(v);
        }
    for (int v : seedFiles.negSeeds)
        if (G.isNodeExist(v)) {
            baseNeg.insert(v);
            forbidden.insert(v);
        }

    return runSeedSelection(G, numberOfSeeds, basePos, baseNeg, forbidden);
}

unordered_set<int> seedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds) {
    using namespace student_algo_detail;

    if (numberOfSeeds == 0 || G.getSize() == 0) return {};

    const SeedFiles& seedFiles = getSeedFiles();
    unordered_set<int> forbidden = seedFiles.forbidden;

    unordered_set<int> basePos;
    unordered_set<int> baseNeg;
    basePos.reserve(seedFiles.givenPos.size() + 1);
    baseNeg.reserve(seedFiles.negSeeds.size() + givenNegSeeds.size());

    for (int v : seedFiles.givenPos)
        if (G.isNodeExist(v)) {
            basePos.insert(v);
            forbidden.insert(v);
        }

    if (G.isNodeExist(givenPosSeed)) {
        basePos.insert(givenPosSeed);
        forbidden.insert(givenPosSeed);
    }

    for (int v : seedFiles.negSeeds)
        if (G.isNodeExist(v)) {
            baseNeg.insert(v);
            forbidden.insert(v);
        }

    for (int v : givenNegSeeds)
        if (G.isNodeExist(v)) {
            baseNeg.insert(v);
            forbidden.insert(v);
        }

    return runSeedSelection(G, numberOfSeeds, basePos, baseNeg, forbidden);
}

#endif
