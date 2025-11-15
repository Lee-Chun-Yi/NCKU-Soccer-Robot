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
	int    node_id;
	int    round;

	bool operator<(const NodeGain& other) const {
		return score < other.score;   // max-heap
	}
};

unordered_set<int> seedSelection(DirectedGraph& G,
	unsigned int numberOfSeeds,
	int givenPosSeed,
	const unordered_set<int>& givenNegSeeds)
{
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	vector<int> nodes = G.getAllNodes();
	if (nodes.empty()) return seeds;

	const size_t N = nodes.size();
	const double EPS = 1e-12;

	// id -> index
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(N * 2);
	for (size_t i = 0; i < N; ++i) {
		id2idx[nodes[i]] = i;
	}

	// thresholds, in-strength, neg-out-strength
	vector<double> pos_th(N, 0.0);
	vector<double> neg_th_mag(N, 0.0);
	vector<double> in_strength(N, 0.0);
	vector<double> neg_out_strength(N, 0.0);

	for (size_t i = 0; i < N; ++i) {
		int u = nodes[i];

		double pth = G.getNodeThreshold(u);   // positive threshold
		double nth = G.getNodeThreshold2(u);  // negative threshold (usually negative)

		pos_th[i] = std::max(EPS, pth);
		neg_th_mag[i] = std::max(EPS, std::fabs(nth));

		// in-strength (positive only)
		vector<int> innei = G.getNodeInNeighbors(u);
		double sum_in = 0.0;
		for (int p : innei) {
			double w = G.getEdgeInfluence(p, u);
			if (w > 0.0) sum_in += w;
		}
		in_strength[i] = sum_in;

		// neg_out_strength (how much negative u can spread out)
		vector<int> outnei = G.getNodeOutNeighbors(u);
		double sum_neg_out = 0.0;
		for (int v : outnei) {
			double w = G.getEdgeInfluence(u, v);
			if (w < 0.0) sum_neg_out += std::fabs(w);
		}
		neg_out_strength[i] = sum_neg_out;
	}

	// coverage
	vector<double> pos_coverage(N, 0.0);
	vector<double> neg_coverage(N, 0.0);

	// heuristic parameters (slightly rebalanced)
	const double LAMBDA = 1.30;  // positive gain
	const double PHI = 1.00;  // negative penalty (比你原本高一點)
	const double GAMMA = 0.18;  // self positive threshold penalty
	const double DELTA = 0.12;  // in-strength penalty
	const double ETA = 0.40;  // reward for large negative threshold (strong against negative)
	const double RISK = 0.25;  // new: penalty for neg_out_strength

	auto is_forbidden = [&](int node) -> bool {
		if (node == givenPosSeed) return true;
		return givenNegSeeds.find(node) != givenNegSeeds.end();
	};

	// marginal gain
	auto marginal_gain = [&](int u) -> double {
		if (seeds.count(u) || is_forbidden(u)) {
			return -std::numeric_limits<double>::infinity();
		}

		auto it_u = id2idx.find(u);
		if (it_u == id2idx.end()) {
			return -std::numeric_limits<double>::infinity();
		}
		size_t iu = it_u->second;

		double score = 0.0;

		// node-level terms
		score -= GAMMA * pos_th[iu];
		score -= DELTA * in_strength[iu];
		score += ETA * neg_th_mag[iu];
		score -= RISK * neg_out_strength[iu];   // new: avoid nodes that can spread lots of negative

		// 1-hop contribution
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (int v : outnei) {
			auto it_v = id2idx.find(v);
			if (it_v == id2idx.end()) continue;
			size_t j = it_v->second;

			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				double need = std::max(0.0, 1.0 - pos_coverage[j]);
				if (need > 0.0) {
					double add = w / pos_th[j];
					double eff = std::min(need, add);
					score += LAMBDA * eff;
				}
			}
			else if (w < -EPS) {
				double need_neg = std::max(0.0, 1.0 - neg_coverage[j]);
				if (need_neg > 0.0) {
					double add_neg = std::fabs(w) / neg_th_mag[j];
					double eff_neg = std::min(need_neg, add_neg);
					score -= PHI * eff_neg;
				}
			}
		}

		return score;
	};

	// apply influence
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
					pos_coverage[j] = std::min(1.0, pos_coverage[j] + add);
				}
			}
			else if (w < -EPS) {
				double add_neg = std::fabs(w) / neg_th_mag[j];
				if (add_neg > 0.0) {
					neg_coverage[j] = std::min(1.0, neg_coverage[j] + add_neg);
				}
			}
		}
	};

	// apply given positive seed once
	if (id2idx.find(givenPosSeed) != id2idx.end()) {
		apply_influence(givenPosSeed);
	}

	// CELF
	priority_queue<NodeGain> pq;

	// k=0 initial gains
	for (int u : nodes) {
		double sc = marginal_gain(u);
		pq.push(NodeGain{ sc, u, 0 });
	}

	for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {
		int bestNode = -1;

		while (true) {
			if (pq.empty()) break;

			NodeGain top = pq.top();
			pq.pop();

			if (top.round < (int)k) {
				double new_score = marginal_gain(top.node_id);
				pq.push(NodeGain{ new_score, top.node_id, (int)k });
				continue;
			}
			else {
				bestNode = top.node_id;
				break;
			}
		}

		if (bestNode == -1 || is_forbidden(bestNode)) break;

		seeds.insert(bestNode);
		apply_influence(bestNode);
	}

	return seeds;
}

#endif // YOUR_ALGORITHM_H
