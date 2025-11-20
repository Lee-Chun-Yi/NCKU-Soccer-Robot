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

// Node gain structure used in CELF priority queue
struct NodeGain {
    double score;   // heuristic marginal gain
    int    node_id; // node id
    int    round;   // iteration k when this score was last updated

    bool operator<(const NodeGain& other) const {
        // max-heap by score
        return score < other.score;
    }
};

unordered_set<int> seedSelection(DirectedGraph& G,
                                 unsigned int numberOfSeeds,
                                 int givenPosSeed,
                                 const unordered_set<int>& givenNegSeeds) {
    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;

    // Get all nodes
    vector<int> nodes = G.getAllNodes();
    if (nodes.empty()) return seeds;

    // Forbidden seeds: given positive + all given negatives
    unordered_set<int> forbidden;
    forbidden.insert(givenPosSeed);
    for (int neg : givenNegSeeds) {
        forbidden.insert(neg);
    }

    // Map node id -> index
    unordered_map<int, size_t> id2idx;
    id2idx.reserve(nodes.size() * 2);
    for (size_t i = 0; i < nodes.size(); ++i) {
        id2idx[nodes[i]] = i;
    }

    const double EPS = 1e-12;
    const size_t N  = nodes.size();

    // Precompute:
    //  - pos_th: positive threshold of each node
    //  - neg_th_mag: absolute value of negative threshold
    //  - in_strength: total positive inbound influence
    vector<double> pos_th(N, 0.0);
    vector<double> neg_th_mag(N, 0.0);
    vector<double> in_strength(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        int u = nodes[i];

        double pth = G.getNodeThreshold(u);
        double nth = G.getNodeThreshold2(u);

        pos_th[i]     = max(EPS, pth);
        neg_th_mag[i] = max(EPS, fabs(nth));

        double sum_in = 0.0;
        vector<int> innei = G.getNodeInNeighbors(u);
        for (int p : innei) {
            double w = G.getEdgeInfluence(p, u);
            if (w > 0.0) {
                sum_in += w;
            }
        }
        in_strength[i] = sum_in;
    }

    // Positive / negative coverage approximation
    vector<double> pos_coverage(N, 0.0);
    vector<double> neg_coverage(N, 0.0);

    // Tuned heuristic weights (single global profile)
    const double LAMBDA = 1.65;  // positive contribution weight
    const double PHI    = 0.75;  // negative influence penalty
    const double GAMMA  = 0.08;  // self positive-threshold penalty
    const double DELTA  = 0.025; // inbound strength penalty
    const double ETA    = 0.33;  // reward for large negative threshold

    // Heuristic marginal gain of adding node u as a seed
    auto marginal_gain = [&](int u) -> double {
        if (seeds.count(u) || forbidden.count(u)) {
            return -numeric_limits<double>::infinity();
        }

        auto it_u = id2idx.find(u);
        if (it_u == id2idx.end()) {
            return -numeric_limits<double>::infinity();
        }
        size_t iu = it_u->second;

        double score = 0.0;

        // Node intrinsic properties
        score -= GAMMA * pos_th[iu];
        score -= DELTA * in_strength[iu];
        score += ETA   * neg_th_mag[iu];

        // Outgoing neighbors contribution
        vector<int> outnei = G.getNodeOutNeighbors(u);
        for (int v : outnei) {
            auto it_v = id2idx.find(v);
            if (it_v == id2idx.end()) continue;
            size_t j = it_v->second;

            double w = G.getEdgeInfluence(u, v);

            if (w > EPS) {
                // Positive edge: increase positive coverage of v
                double need = max(0.0, 1.0 - pos_coverage[j]);
                if (need > 0.0) {
                    double add = w / pos_th[j];
                    if (add > need) add = need;
                    score += LAMBDA * add;
                }
            } else if (w < -EPS) {
                // Negative edge: this seed adds negative pressure to v
                double need_neg = max(0.0, 1.0 - neg_coverage[j]);
                if (need_neg > 0.0) {
                    double add_neg = fabs(w) / neg_th_mag[j];
                    if (add_neg > need_neg) add_neg = need_neg;
                    score -= PHI * add_neg;
                }
            }
        }

        return score;
    };

    // Apply the effect of actually selecting u as a seed,
    // updating coverage arrays.
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
                if (add > 0.0) {
                    pos_coverage[j] += add;
                    if (pos_coverage[j] > 1.0) pos_coverage[j] = 1.0;
                }
            } else if (w < -EPS) {
                double add_neg = fabs(w) / neg_th_mag[j];
                if (add_neg > 0.0) {
                    neg_coverage[j] += add_neg;
                    if (neg_coverage[j] > 1.0) neg_coverage[j] = 1.0;
                }
            }
        }
    };

    // Initialize CELF priority queue with initial scores
    priority_queue<NodeGain> pq;
    for (int u : nodes) {
        if (forbidden.count(u)) continue;
        double sc = marginal_gain(u);
        pq.push(NodeGain{sc, u, 0});
    }

    // CELF main loop: select numberOfSeeds seeds
    for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {
        int bestNode = -1;

        // Find the best node whose score is up-to-date for round k
        while (!pq.empty()) {
            NodeGain top = pq.top();
            pq.pop();

            if (top.round < static_cast<int>(k)) {
                // Outdated score: recompute and push back
                double new_sc = marginal_gain(top.node_id);
                pq.push(NodeGain{new_sc, top.node_id, static_cast<int>(k)});
                continue;
            } else {
                // Score is up-to-date for this round
                bestNode = top.node_id;
                break;
            }
        }

        if (bestNode == -1) break;

        seeds.insert(bestNode);
        apply_influence(bestNode);
    }

    return seeds;
}

#endif // YOUR_ALGORITHM_H
