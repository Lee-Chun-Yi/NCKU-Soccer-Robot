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
#include <queue>

using namespace std;

struct NodeGain {
    double score;
    int node_id;
    int round;
    bool operator<(const NodeGain& other) const {
        return score < other.score;
    }
};

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds,
    int givenPosSeed, const unordered_set<int>& givenNegSeeds) {

    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;

    vector<int> nodes = G.getAllNodes();
    if (nodes.empty()) return seeds;

    unordered_set<int> forbidden;
    forbidden.insert(givenPosSeed);
    for (int neg : givenNegSeeds) forbidden.insert(neg);

    unordered_map<int, size_t> id2idx;
    id2idx.reserve(nodes.size() * 2);
    for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;

    const double EPS = 1e-12;
    const size_t N = nodes.size();

    vector<double> pos_th(N, 0.0);
    vector<double> neg_th_mag(N, 0.0);
    vector<double> in_strength(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        int u = nodes[i];
        pos_th[i] = max(EPS, G.getNodeThreshold(u));
        neg_th_mag[i] = max(EPS, fabs(G.getNodeThreshold2(u)));

        double sum_in = 0.0;
        vector<int> innei = G.getNodeInNeighbors(u);
        for (int p : innei) {
            double w = G.getEdgeInfluence(p, u);
            if (w > 0.0) sum_in += w;
        }
        in_strength[i] = sum_in;
    }

    vector<double> pos_coverage(N, 0.0);
    vector<double> neg_coverage(N, 0.0);

    // Select heuristic parameters based on graph size
    double LAMBDA, PHI, GAMMA, DELTA, ETA;
    
    if (N <= 200) {
        // Small graphs (≈100 nodes): emphasize spread, reduce over-penalty
        LAMBDA = 2.0;
        PHI    = 0.55;
        GAMMA  = 0.05;
        DELTA  = 0.01;
        ETA    = 0.35;
    }
    else if (N <= 2000) {
        // Medium graphs (≈1k nodes): balanced and stable baseline
        LAMBDA = 1.8;
        PHI    = 1.0;
        GAMMA  = 0.1;
        DELTA  = 0.03;
        ETA    = 0.25;
    }
    else {
        // Large graphs (≈10k nodes): avoid over-penalizing central nodes
        LAMBDA = 1.8;
        PHI    = 0.8;
        GAMMA  = 0.1;
        DELTA  = 0.03;
        ETA    = 0.30;
    }


    auto marginal_gain = [&](int u) -> double {
        if (seeds.count(u) || forbidden.count(u)) return -numeric_limits<double>::infinity();
        auto it_u = id2idx.find(u);
        if (it_u == id2idx.end()) return -numeric_limits<double>::infinity();
        size_t iu = it_u->second;

        double score = 0.0;
        score -= GAMMA * pos_th[iu];
        score -= DELTA * in_strength[iu];
        score += ETA * neg_th_mag[iu];

        vector<int> outnei = G.getNodeOutNeighbors(u);
        for (int v : outnei) {
            auto it_v = id2idx.find(v);
            if (it_v == id2idx.end()) continue;
            size_t j = it_v->second;
            double w = G.getEdgeInfluence(u, v);

            if (w > EPS) {
                double need = max(0.0, 1.0 - pos_coverage[j]);
                if (need > 0.0) {
                    double add = min(need, w / pos_th[j]);
                    score += LAMBDA * add;
                }
            }
            else if (w < -EPS) {
                double need_neg = max(0.0, 1.0 - neg_coverage[j]);
                if (need_neg > 0.0) {
                    double add_neg = min(need_neg, fabs(w) / neg_th_mag[j]);
                    score -= PHI * add_neg;
                }
            }
        }
        return score;
    };

    auto apply_influence = [&](int u) {
        auto it_u = id2idx.find(u);
        if (it_u == id2idx.end()) return;

        vector<int> outnei = G.getNodeOutNeighbors(u);
        for (int v : outnei) {
            auto it_v = id2idx.find(v);
            if (it_v == id2idx.end()) continue;
            size_t j = it_v->second;
            double w = G.getEdgeInfluence(u, v);

            if (w > EPS) {
                double add = w / pos_th[j];
                pos_coverage[j] = min(1.0, pos_coverage[j] + add);
            }
            else if (w < -EPS) {
                double add_neg = fabs(w) / neg_th_mag[j];
                neg_coverage[j] = min(1.0, neg_coverage[j] + add_neg);
            }
        }
    };

    priority_queue<NodeGain> pq;
    for (int u : nodes) {
        if (forbidden.count(u)) continue;
        pq.push({marginal_gain(u), u, 0});
    }

    for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {
        int bestNode = -1;
        while (!pq.empty()) {
            NodeGain top = pq.top();
            pq.pop();
            if (top.round < (int)k) {
                pq.push({marginal_gain(top.node_id), top.node_id, (int)k});
                continue;
            }
            bestNode = top.node_id;
            break;
        }
        if (bestNode == -1) break;

        seeds.insert(bestNode);
        apply_influence(bestNode);
    }

    return seeds;
}

#endif
