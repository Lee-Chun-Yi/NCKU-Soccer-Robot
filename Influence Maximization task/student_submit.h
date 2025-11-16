#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int, int> idToIndex;
    vector<vector<pair<int, double>>> outAdj;
    vector<double> outStrength;
    vector<double> posThreshold;
    vector<double> negThreshold;
    vector<int> posOutDegree;
    vector<double> negExposure;

    size_t size() const { return nodeIds.size(); }
};

static GraphCache buildGraphCache(DirectedGraph& G) {
    GraphCache cache;
    cache.nodeIds = G.getAllNodes();
    sort(cache.nodeIds.begin(), cache.nodeIds.end());

    const size_t n = cache.nodeIds.size();
    cache.outAdj.assign(n, {});
    cache.outStrength.assign(n, 0.0);
    cache.posThreshold.assign(n, 0.0);
    cache.negThreshold.assign(n, 0.0);
    cache.posOutDegree.assign(n, 0);
    cache.negExposure.assign(n, 0.0);

    for (size_t idx = 0; idx < n; ++idx)
        cache.idToIndex[cache.nodeIds[idx]] = static_cast<int>(idx);

    for (size_t idx = 0; idx < n; ++idx) {
        int nodeId = cache.nodeIds[idx];
        cache.posThreshold[idx] = G.getNodeThreshold(nodeId);
        cache.negThreshold[idx] = G.getNodeThreshold2(nodeId);

        vector<int> neighbors = G.getNodeOutNeighbors(nodeId);
        cache.posOutDegree[idx] = static_cast<int>(neighbors.size());
        vector<pair<int, double>> adj;
        adj.reserve(neighbors.size());
        double strength = 0.0;
        for (int nb : neighbors) {
            auto it = cache.idToIndex.find(nb);
            if (it == cache.idToIndex.end()) continue;
            double infl = G.getEdgeInfluence(nodeId, nb);
            if (infl <= 0) continue;
            adj.emplace_back(it->second, infl);
            strength += infl;
        }
        cache.outAdj[idx] = move(adj);
        cache.outStrength[idx] = strength;
    }
    return cache;
}

static void computeNegExposure(GraphCache& cache, const unordered_set<int>& givenNegSeeds) {
    if (cache.size() == 0) return;
    vector<double> exposure(cache.size(), 0.0);
    for (int negSeed : givenNegSeeds) {
        auto itSeed = cache.idToIndex.find(negSeed);
        if (itSeed == cache.idToIndex.end()) continue;
        int seedIdx = itSeed->second;
        for (const auto& edge : cache.outAdj[seedIdx])
            exposure[edge.first] += edge.second;
    }
    cache.negExposure = move(exposure);
}

struct DiffusionEstimate {
    double posEstimate;
    double negEstimate;
    double depthWeightedGain;
};

static DiffusionEstimate runPartialDiffusion(
        const GraphCache& cache,
        int startIdx,
        int maxDepth,
        int visitLimit) {
    DiffusionEstimate res{0.0, 0.0, 0.0};
    if (startIdx < 0 || static_cast<size_t>(startIdx) >= cache.size()) return res;

    const size_t n = cache.size();
    vector<double> posInfluence(n, 0.0);
    vector<char> state(n, 0); // 0: inactive, 1: pos, 2: neg
    queue<pair<int, int>> q;

    state[startIdx] = 1;
    res.posEstimate = 1.0;
    res.depthWeightedGain = 1.0;
    q.emplace(startIdx, 0);
    int expanded = 1;

    while (!q.empty()) {
        auto [idx, depth] = q.front();
        q.pop();
        if (depth >= maxDepth) continue;

        for (const auto& edge : cache.outAdj[idx]) {
            int nbIdx = edge.first;
            posInfluence[nbIdx] += edge.second;

            double net = posInfluence[nbIdx] - cache.negExposure[nbIdx];
            if (state[nbIdx] == 0 && net >= cache.posThreshold[nbIdx]) {
                state[nbIdx] = 1;
                res.posEstimate += 1.0;
                double weight = static_cast<double>(maxDepth + 1 - (depth + 1)) /
                                static_cast<double>(maxDepth + 1);
                res.depthWeightedGain += max(0.0, weight);
                if (++expanded >= visitLimit) {
                    while (!q.empty()) q.pop();
                    break;
                }
                q.emplace(nbIdx, depth + 1);
            } else if (state[nbIdx] == 0 && net <= cache.negThreshold[nbIdx]) {
                state[nbIdx] = 2;
                res.negEstimate += 1.0;
            }
        }
    }
    return res;
}

static vector<pair<double, int>> scoreFastCandidates(const GraphCache& cache,
        const unordered_set<int>& skipNodes) {
    vector<pair<double, int>> scores;
    scores.reserve(cache.size());
    for (size_t idx = 0; idx < cache.size(); ++idx) {
        int nodeId = cache.nodeIds[idx];
        if (skipNodes.count(nodeId)) continue;
        double fastScore = cache.outStrength[idx]
                + 0.1 * static_cast<double>(cache.posOutDegree[idx])
                - 0.6 * cache.posThreshold[idx]
                - 0.8 * cache.negExposure[idx];
        scores.emplace_back(fastScore, nodeId);
    }
    sort(scores.begin(), scores.end(), [](const auto& a, const auto& b) {
        if (a.first == b.first) return a.second < b.second;
        return a.first > b.first;
    });
    return scores;
}

static unordered_set<int> deterministicSelection(GraphCache& cache,
        const vector<pair<double, int>>& fastOrdered,
        unsigned int numberOfSeeds,
        int maxDepth,
        int visitLimit) {
    unordered_set<int> selected;
    if (numberOfSeeds == 0) return selected;

    const size_t candidateLimit = min({cache.size(),
            static_cast<size_t>(50 + 2 * numberOfSeeds),
            static_cast<size_t>(150)});

    vector<pair<double, int>> finalScores;
    finalScores.reserve(candidateLimit);

    for (size_t i = 0; i < candidateLimit && i < fastOrdered.size(); ++i) {
        int nodeId = fastOrdered[i].second;
        int idx = cache.idToIndex[nodeId];
        auto estimate = runPartialDiffusion(cache, idx, maxDepth, visitLimit);

        if (cache.negExposure[idx] > (cache.outStrength[idx] + cache.posThreshold[idx]) * 1.2)
            continue;
        if (estimate.posEstimate <= estimate.negEstimate)
            continue;

        double finalScore = 1.5 * (estimate.posEstimate - estimate.negEstimate)
                + 0.6 * cache.outStrength[idx]
                - 0.8 * cache.negExposure[idx]
                - 0.3 * cache.posThreshold[idx]
                + 0.4 * estimate.depthWeightedGain;
        if (finalScore <= 0) continue;
        finalScores.emplace_back(finalScore, nodeId);
    }

    sort(finalScores.begin(), finalScores.end(), [](const auto& a, const auto& b) {
        if (a.first == b.first) return a.second < b.second;
        return a.first > b.first;
    });

    for (const auto& entry : finalScores) {
        if (selected.size() >= numberOfSeeds) break;
        selected.insert(entry.second);
    }

    size_t idxFallback = 0;
    while (selected.size() < numberOfSeeds && idxFallback < fastOrdered.size()) {
        int nodeId = fastOrdered[idxFallback++].second;
        if (!selected.count(nodeId))
            selected.insert(nodeId);
    }
    return selected;
}

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
    unordered_set<int> empty;
    return seedSelection(G, numberOfSeeds, -1, empty);
}

unordered_set<int> seedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds) {
    GraphCache cache = buildGraphCache(G);
    computeNegExposure(cache, givenNegSeeds);

    unordered_set<int> skipNodes = givenNegSeeds;
    if (givenPosSeed >= 0)
        skipNodes.insert(givenPosSeed);

    vector<pair<double, int>> fastOrdered = scoreFastCandidates(cache, skipNodes);

    unordered_set<int> seeds = deterministicSelection(cache, fastOrdered,
            numberOfSeeds, 2, 900);

    return seeds;
}

#endif
