#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

/*
 * seedSelection:
 *   - G: DirectedGraph (loaded externally)
 *   - numberOfSeeds: how many positive seeds to select (should be 9)
 *
 * Return an unordered_set<int> containing exactly numberOfSeeds distinct ids.
 * No I/O; deterministic heuristic.
 */
unordered_set<int> seedSelection(DirectedGraph& G,
                                 unsigned int numberOfSeeds,
                                 int givenPosSeed,
                                 const unordered_set<int>& givenNegSeeds) {
    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;

    vector<int> nodes = G.getAllNodes();
    if (nodes.empty()) return seeds;

    unordered_map<int, size_t> id2idx;
    id2idx.reserve(nodes.size() * 2);
    for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;

    const double EPS = 1e-12;
    const double LAMBDA = 1.0;   // uncovered outgoing influence weight
    const double GAMMA  = 0.35;  // penalty for high self threshold
    const double DELTA  = 0.12;  // penalty for being heavily influenced already
    const double ETA    = 0.25;  // reward for resisting negative activation
    const double ZETA   = 0.35;  // penalty for touching provided negative seeds

    struct EdgeInfo {
        int targetIdx;
        double weightFactor; // scaled influence for scoring updates
        double coverGain;    // normalized coverage contribution w / threshold
    };

    vector<double> pos_th(nodes.size(), 0.0);
    vector<double> in_strength(nodes.size(), 0.0);
    vector<double> neg_resilience(nodes.size(), 0.0);
    for (size_t i = 0; i < nodes.size(); ++i) {
        int u = nodes[i];
        pos_th[i] = std::max(EPS, G.getNodeThreshold(u));
        double in_sum = 0.0;
        for (int p : G.getNodeInNeighbors(u)) {
            in_sum += std::max(0.0, G.getEdgeInfluence(p, u));
        }
        in_strength[i] = in_sum;
        neg_resilience[i] = std::fabs(G.getNodeThreshold2(u));
    }

    vector<vector<EdgeInfo>> out_edges(nodes.size());
    vector<vector<pair<int, double>>> back_edges(nodes.size());
    vector<double> nodePenalty(nodes.size(), 0.0);
    for (size_t i = 0; i < nodes.size(); ++i) {
        int u = nodes[i];
        for (int v : G.getNodeOutNeighbors(u)) {
            auto it = id2idx.find(v);
            if (it == id2idx.end()) continue;
            size_t j = it->second;
            double w = std::max(0.0, G.getEdgeInfluence(u, v));
            if (w <= 0.0) continue;
            double coverGain = w / pos_th[j];
            out_edges[i].push_back({static_cast<int>(j), LAMBDA * w, coverGain});
            back_edges[j].push_back({static_cast<int>(i), LAMBDA * w});
            if (givenNegSeeds.count(v)) nodePenalty[i] += ZETA * w;
        }
    }

    vector<double> residual_need(nodes.size(), 1.0);
    vector<double> score(nodes.size(), 0.0);
    for (size_t i = 0; i < nodes.size(); ++i) {
        score[i] = -GAMMA * pos_th[i]
                 - DELTA * in_strength[i]
                 + ETA   * neg_resilience[i]
                 - nodePenalty[i];
        for (const auto& e : out_edges[i]) {
            score[i] += e.weightFactor * residual_need[e.targetIdx];
        }
    }

    const double MIN_SCORE = -std::numeric_limits<double>::infinity();

    auto disable_node = [&](int idx) {
        score[idx] = MIN_SCORE;
    };

    auto apply_seed_idx = [&](int idx) {
        for (const auto& e : out_edges[idx]) {
            int tgt = e.targetIdx;
            if (residual_need[tgt] <= 0.0) continue;
            double delta = std::min(residual_need[tgt], e.coverGain);
            if (delta <= 0.0) continue;
            residual_need[tgt] -= delta;
            for (const auto& back : back_edges[tgt]) {
                score[back.first] -= back.second * delta;
            }
        }
    };

    unordered_set<int> banned = givenNegSeeds;
    banned.insert(givenPosSeed);
    for (int b : banned) {
        auto it = id2idx.find(b);
        if (it != id2idx.end()) disable_node(static_cast<int>(it->second));
    }

    auto itPos = id2idx.find(givenPosSeed);
    if (itPos != id2idx.end()) {
        apply_seed_idx(static_cast<int>(itPos->second));
    }

    for (unsigned int k = 0; k < numberOfSeeds && seeds.size() < numberOfSeeds; ++k) {
        double bestScore = MIN_SCORE;
        int bestIdx = -1;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (score[i] > bestScore) {
                bestScore = score[i];
                bestIdx = static_cast<int>(i);
            }
        }
        if (bestIdx < 0) break;
        seeds.insert(nodes[bestIdx]);
        disable_node(bestIdx);
        apply_seed_idx(bestIdx);
    }

    return seeds;
}

#endif
