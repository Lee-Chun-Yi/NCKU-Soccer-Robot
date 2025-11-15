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

    double score = 1.8 * pos + 1.0 * twoHop - 0.4 * negRisk - 0.2 * adjPenalty;
    score += 0.15 / (1.0 + negRisk);
    return score;
}

struct CandidateEntry {
    int node = -1;
    double gain = numeric_limits<double>::lowest();
    size_t last_update = numeric_limits<size_t>::max();
    SpreadStats stats;
};

struct CandidateComparator {
    bool operator()(const CandidateEntry& a, const CandidateEntry& b) const {
        if (a.gain == b.gain) return a.node > b.node;
        return a.gain < b.gain;
    }
};

static constexpr int SIMULATION_LIMIT = 8000;
static constexpr int MAX_PROPAGATION_ROUNDS = 32;
static int gSimulationCount = 0;

static void resetSimulationBudget() { gSimulationCount = 0; }

static SpreadStats simulateSpread(
        DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& selected,
        int extraNode) {
    if (gSimulationCount >= SIMULATION_LIMIT) {
        return SpreadStats{0, 0};
    }
    ++gSimulationCount;

    static unordered_set<int> posActive;
    static unordered_set<int> negActive;

    posActive.clear();
    posActive.insert(basePos.begin(), basePos.end());
    posActive.insert(selected.begin(), selected.end());
    if (extraNode >= 0) posActive.insert(extraNode);

    negActive.clear();
    negActive.insert(baseNeg.begin(), baseNeg.end());

    SpreadStats stats;
    stats.pos = static_cast<int>(posActive.size());
    stats.neg = static_cast<int>(negActive.size());

    int rounds = 0;
    while (rounds < MAX_PROPAGATION_ROUNDS) {
        auto result = diffuse_one_round_signed(&G, posActive, negActive);
        const auto& newPos = result.first;
        const auto& newNeg = result.second;
        if (newPos.empty() && newNeg.empty()) break;
        posActive.insert(newPos.begin(), newPos.end());
        negActive.insert(newNeg.begin(), newNeg.end());
        ++rounds;
    }

    stats.pos = static_cast<int>(posActive.size());
    stats.neg = static_cast<int>(negActive.size());

    return stats;
}

static double evaluateStats(const SpreadStats& stats) {
    return static_cast<double>(stats.pos) - 1.6 * static_cast<double>(stats.neg);
}

static void fillWithHeuristics(const vector<pair<int, double>>& ranking,
        const vector<int>& allNodes,
        const unordered_set<int>& forbidden,
        unordered_set<int>& selected,
        vector<int>& selectedOrder,
        unsigned int numberOfSeeds) {
    const double riskLimit = 8.0;
    for (const auto& item : ranking) {
        if (selected.size() >= numberOfSeeds) break;
        if (forbidden.count(item.first)) continue;
        if (selected.count(item.first)) continue;
        if (getNodeRisk(item.first) > riskLimit) continue;
        selected.insert(item.first);
        selectedOrder.push_back(item.first);
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
        if (selected.insert(item.second).second) selectedOrder.push_back(item.second);
    }

    if (selected.size() >= numberOfSeeds) return;

    for (int node : allNodes) {
        if (selected.size() >= numberOfSeeds) break;
        if (forbidden.count(node)) continue;
        if (selected.count(node)) continue;
        if (selected.insert(node).second) selectedOrder.push_back(node);
    }
}

static vector<int> collectTopCandidates(const vector<pair<int, double>>& ranking,
        size_t limit,
        const unordered_set<int>& forbidden) {
    if (limit == 0) return {};
    vector<int> result;
    result.reserve(limit);
    for (const auto& item : ranking) {
        if (forbidden.count(item.first)) continue;
        result.push_back(item.first);
        if (result.size() >= limit) break;
    }
    return result;
}

static void dropReplaceRefinement(DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden,
        const vector<int>& candidateList,
        vector<int>& seeds,
        SpreadStats& bestStats,
        int passes) {
    if (seeds.empty() || candidateList.empty()) return;
    unordered_set<int> seedSet(seeds.begin(), seeds.end());
    double bestValue = evaluateStats(bestStats);
    passes = max(passes, 1);
    for (int pass = 0; pass < passes; ++pass) {
        bool passImproved = false;
        for (size_t i = 0; i < seeds.size(); ++i) {
            if (gSimulationCount >= SIMULATION_LIMIT) return;
            int original = seeds[i];
            bool replaced = false;
            seedSet.erase(original);
            for (int candidate : candidateList) {
                if (gSimulationCount >= SIMULATION_LIMIT) break;
                if (seedSet.count(candidate)) continue;
                if (forbidden.count(candidate)) continue;
                seedSet.insert(candidate);
                SpreadStats stats = simulateSpread(G, basePos, baseNeg, seedSet, -1);
                double val = evaluateStats(stats);
                if (val > bestValue + 1e-6) {
                    seeds[i] = candidate;
                    bestStats = stats;
                    bestValue = val;
                    replaced = true;
                    passImproved = true;
                    break;
                }
                seedSet.erase(candidate);
            }
            if (!replaced) {
                seedSet.insert(original);
            }
        }
        if (!passImproved) break;
    }
}

static void pairwiseSwapRefinement(DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden,
        const vector<int>& candidateList,
        vector<int>& seeds,
        SpreadStats& bestStats) {
    if (seeds.size() < 2 || candidateList.empty()) return;
    unordered_set<int> seedSet(seeds.begin(), seeds.end());
    double bestValue = evaluateStats(bestStats);
    for (size_t i = 0; i < seeds.size(); ++i) {
        for (size_t j = i + 1; j < seeds.size(); ++j) {
            if (gSimulationCount >= SIMULATION_LIMIT) return;
            int first = seeds[i];
            seedSet.erase(first);
            bool replaced = false;
            for (int candidate : candidateList) {
                if (gSimulationCount >= SIMULATION_LIMIT) break;
                if (seedSet.count(candidate)) continue;
                if (forbidden.count(candidate)) continue;
                seedSet.insert(candidate);
                SpreadStats stats = simulateSpread(G, basePos, baseNeg, seedSet, -1);
                double val = evaluateStats(stats);
                if (val > bestValue + 1e-6) {
                    seeds[i] = candidate;
                    bestStats = stats;
                    bestValue = val;
                    replaced = true;
                    break;
                }
                seedSet.erase(candidate);
            }
            if (!replaced) {
                seedSet.insert(first);
            }

            if (gSimulationCount >= SIMULATION_LIMIT) return;
            int second = seeds[j];
            seedSet.erase(second);
            replaced = false;
            for (int candidate : candidateList) {
                if (gSimulationCount >= SIMULATION_LIMIT) break;
                if (seedSet.count(candidate)) continue;
                if (forbidden.count(candidate)) continue;
                seedSet.insert(candidate);
                SpreadStats stats = simulateSpread(G, basePos, baseNeg, seedSet, -1);
                double val = evaluateStats(stats);
                if (val > bestValue + 1e-6) {
                    seeds[j] = candidate;
                    bestStats = stats;
                    bestValue = val;
                    replaced = true;
                    break;
                }
                seedSet.erase(candidate);
            }
            if (!replaced) {
                seedSet.insert(second);
            }
        }
    }
}

static void refineSelection(DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const vector<pair<int, double>>& ranking,
        const unordered_set<int>& forbidden,
        vector<int>& seeds,
        SpreadStats& bestStats) {
    if (seeds.empty()) return;
    size_t phaseOneLimit = std::min<size_t>(ranking.size(), 600);
    vector<int> phaseOneCandidates = collectTopCandidates(ranking, phaseOneLimit, forbidden);
    dropReplaceRefinement(G, basePos, baseNeg, forbidden, phaseOneCandidates, seeds, bestStats, 3);

    size_t phaseTwoLimit = std::min<size_t>(phaseOneCandidates.size(), static_cast<size_t>(300));
    vector<int> phaseTwoCandidates;
    if (phaseTwoLimit > 0 && !phaseOneCandidates.empty()) {
        phaseTwoCandidates.assign(phaseOneCandidates.begin(), phaseOneCandidates.begin() + phaseTwoLimit);
    }
    pairwiseSwapRefinement(G, basePos, baseNeg, forbidden, phaseTwoCandidates, seeds, bestStats);

    if (gSimulationCount < SIMULATION_LIMIT) {
        unordered_set<int> finalSet(seeds.begin(), seeds.end());
        bestStats = simulateSpread(G, basePos, baseNeg, finalSet, -1);
    }
}

struct BeamState {
    vector<int> seeds;
    SpreadStats stats;
    double value = numeric_limits<double>::lowest();
};

static pair<vector<int>, SpreadStats> runBeamSearch(DirectedGraph& G,
        unsigned int numberOfSeeds,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden,
        const vector<int>& beamCandidates,
        const SpreadStats& baseStats) {
    pair<vector<int>, SpreadStats> best;
    best.second = baseStats;
    if (numberOfSeeds == 0 || beamCandidates.empty()) return best;

    const size_t beamWidth = 5;
    const size_t expansionsPerState = 60;
    vector<BeamState> beam(1);
    beam[0].stats = baseStats;
    beam[0].value = evaluateStats(baseStats);

    for (unsigned int depth = 0; depth < numberOfSeeds; ++depth) {
        vector<BeamState> next;
        for (const BeamState& state : beam) {
            unordered_set<int> stateSet(state.seeds.begin(), state.seeds.end());
            size_t expansions = 0;
            for (int candidate : beamCandidates) {
                if (stateSet.count(candidate)) continue;
                if (forbidden.count(candidate)) continue;
                SpreadStats stats = simulateSpread(G, basePos, baseNeg, stateSet, candidate);
                BeamState newState = state;
                newState.seeds.push_back(candidate);
                newState.stats = stats;
                newState.value = evaluateStats(stats);
                next.push_back(std::move(newState));
                ++expansions;
                if (gSimulationCount >= SIMULATION_LIMIT) break;
                if (expansions >= expansionsPerState) break;
            }
            if (gSimulationCount >= SIMULATION_LIMIT) break;
        }
        if (next.empty()) break;
        sort(next.begin(), next.end(), [](const BeamState& a, const BeamState& b) {
            if (a.value != b.value) return a.value > b.value;
            return a.seeds < b.seeds;
        });
        if (next.size() > beamWidth) next.resize(beamWidth);
        beam.swap(next);
        if (beam.empty()) break;
    }

    if (!beam.empty() && beam.front().seeds.size() == numberOfSeeds) {
        best.first = beam.front().seeds;
        best.second = beam.front().stats;
    }

    return best;
}

static vector<int> buildCandidatePool(const vector<pair<int, double>>& ranking,
        unsigned int numberOfSeeds,
        const unordered_set<int>& forbidden) {
    vector<int> pool;
    if (ranking.empty()) return pool;
    size_t limit = std::max<size_t>(1000, static_cast<size_t>(numberOfSeeds) * 50);
    limit = std::min(limit, ranking.size());
    double topScore = ranking.front().second;
    double threshold = topScore * 0.05;

    for (size_t i = 0; i < ranking.size() && pool.size() < limit; ++i) {
        int node = ranking[i].first;
        double score = ranking[i].second;
        if (forbidden.count(node)) continue;
        if (score < threshold && pool.size() + 25 < limit) continue;
        pool.push_back(node);
    }

    return pool;
}

static unordered_set<int> runSeedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden) {
    unordered_set<int> selectedSet;
    size_t reserveSize = static_cast<size_t>(numberOfSeeds) * 2 + 8;
    selectedSet.reserve(reserveSize);
    vector<int> selectedOrder;
    selectedOrder.reserve(static_cast<size_t>(numberOfSeeds) + 4);
    if (numberOfSeeds == 0 || G.getSize() == 0) return selectedSet;

    prepareScoreContext(G, baseNeg, forbidden);
    resetSimulationBudget();

    vector<int> allNodes = gScoreContext.ready ? gScoreContext.nodes : G.getAllNodes();
    if (allNodes.empty()) return selectedSet;

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

    SpreadStats currentStats = simulateSpread(G, basePos, baseNeg, selectedSet, -1);
    SpreadStats baseStats = currentStats;

    if (!candidatePool.empty()) {
        priority_queue<CandidateEntry, vector<CandidateEntry>, CandidateComparator> heap;
        for (int node : candidatePool) {
            CandidateEntry entry;
            entry.node = node;
            heap.push(entry);
        }

        while (!heap.empty() && selectedOrder.size() < numberOfSeeds) {
            if (gSimulationCount >= SIMULATION_LIMIT) break;

            CandidateEntry entry = heap.top();
            heap.pop();

            if (selectedSet.count(entry.node)) continue;

            if (entry.last_update == selectedOrder.size()) {
                selectedSet.insert(entry.node);
                selectedOrder.push_back(entry.node);
                currentStats = entry.stats;
                continue;
            }

            SpreadStats stats = simulateSpread(G, basePos, baseNeg, selectedSet, entry.node);
            double marginal = evaluateStats(stats) - evaluateStats(currentStats);
            entry.gain = marginal;
            entry.last_update = selectedOrder.size();
            entry.stats = stats;
            heap.push(entry);
        }
    }

    if (selectedOrder.size() < numberOfSeeds) {
        fillWithHeuristics(ranking, allNodes, forbidden, selectedSet, selectedOrder, numberOfSeeds);
        if (!selectedOrder.empty() && gSimulationCount < SIMULATION_LIMIT) {
            unordered_set<int> tmp(selectedOrder.begin(), selectedOrder.end());
            currentStats = simulateSpread(G, basePos, baseNeg, tmp, -1);
        }
    }

    vector<int> refinedSeeds = selectedOrder;
    if (refinedSeeds.size() > numberOfSeeds) refinedSeeds.resize(numberOfSeeds);
    SpreadStats refinedStats = currentStats;
    refineSelection(G, basePos, baseNeg, ranking, forbidden, refinedSeeds, refinedStats);

    vector<int> bestSeeds = refinedSeeds;
    SpreadStats bestStats = refinedStats;

    if (gSimulationCount < SIMULATION_LIMIT) {
        size_t beamLimit = std::max<size_t>(200, static_cast<size_t>(numberOfSeeds) * 20);
        beamLimit = std::min<size_t>(beamLimit, 450);
        beamLimit = std::min(beamLimit, ranking.size());
        vector<int> beamCandidates = collectTopCandidates(ranking, beamLimit, forbidden);
        auto beamResult = runBeamSearch(G, numberOfSeeds, basePos, baseNeg, forbidden, beamCandidates, baseStats);
        if (!beamResult.first.empty() && beamResult.first.size() == numberOfSeeds) {
            double refinedValue = evaluateStats(bestStats);
            double beamValue = evaluateStats(beamResult.second);
            if (beamValue > refinedValue + 1e-6) {
                bestSeeds = beamResult.first;
                bestStats = beamResult.second;
            }
        }
    }

    unordered_set<int> finalSelection(bestSeeds.begin(), bestSeeds.end());
    return finalSelection;
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
