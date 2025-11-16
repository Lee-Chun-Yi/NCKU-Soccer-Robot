#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>

using namespace std;

/*
 * seedSelection:
 *   - G: DirectedGraph (loaded externally)
 *   - numberOfSeeds: how many positive seeds to select (should be 9)
 *
 * Return an unordered_set<int> containing exactly numberOfSeeds distinct ids.
 * No I/O; deterministic heuristic.
 */
unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	vector<int> nodes = G.getAllNodes();
	if (nodes.empty()) return seeds;

	// Map node id -> index
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(nodes.size() * 2);
	for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;

    // Precompute positive/negative thresholds and incoming strengths
    const double EPS = 1e-12;
    vector<double> pos_th(nodes.size(), 0.0);
    vector<double> in_strength(nodes.size(), 0.0);
    vector<double> neg_resilience(nodes.size(), 0.0); // prefer harder-to-turn-negative nodes
    for (size_t i = 0; i < nodes.size(); ++i) {
        int u = nodes[i];
        pos_th[i] = std::max(EPS, G.getNodeThreshold(u));
        double in_sum = 0.0;
        for (int p : G.getNodeInNeighbors(u)) {
            in_sum += std::max(0.0, G.getEdgeInfluence(p, u));
        }
        in_strength[i] = in_sum;
        // larger |neg_th| means harder to be negatively activated
        neg_resilience[i] = std::fabs(G.getNodeThreshold2(u));
    }

	// Coverage over each node v: accumulated fraction of v's threshold explained
	// by influence from already-selected seeds.
	vector<double> coverage(nodes.size(), 0.0);

	// Heuristic weights
    const double LAMBDA = 1.0;   // uncovered outgoing influence weight
    const double GAMMA  = 0.5;   // penalty for candidate's own pos threshold
    const double DELTA  = 0.15;  // penalty for incoming strength (reduces redundancy)
    const double ETA    = 0.2;   // reward for strong negative threshold magnitude (robustness)

	auto marginal_gain = [&](int u) -> double {
		if (seeds.count(u)) return -1e300; // already selected
        double score = -GAMMA * std::max(EPS, G.getNodeThreshold(u))
                     -DELTA * in_strength[id2idx[u]]
                     +ETA   * neg_resilience[id2idx[u]];
		for (int v : G.getNodeOutNeighbors(u)) {
			auto it = id2idx.find(v);
			if (it == id2idx.end()) continue;
			size_t j = it->second;
			double w = std::max(0.0, G.getEdgeInfluence(u, v));
			double need = std::max(0.0, 1.0 - coverage[j]);
			score += LAMBDA * w * need;
		}
		return score;
	};

	auto apply_coverage = [&](int u) {
		for (int v : G.getNodeOutNeighbors(u)) {
			auto it = id2idx.find(v);
			if (it == id2idx.end()) continue;
			size_t j = it->second;
			double w = std::max(0.0, G.getEdgeInfluence(u, v));
			double th = std::max(EPS, pos_th[j]);
			coverage[j] = std::min(1.0, coverage[j] + w / th);
		}
	};

	for (unsigned int k = 0; k < numberOfSeeds && seeds.size() < numberOfSeeds; ++k) {
		double bestScore = -1e300;
		int bestNode = nodes[0];
		for (int u : nodes) {
			double sc = marginal_gain(u);
			if (sc > bestScore) { bestScore = sc; bestNode = u; }
		}
		seeds.insert(bestNode);
		apply_coverage(bestNode);
	}

	return seeds;
}

#endif
