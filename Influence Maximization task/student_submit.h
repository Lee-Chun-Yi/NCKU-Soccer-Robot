// your_algorithm.h
#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>

using namespace std;

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	if (numberOfSeeds == 0 || G.getSize() == 0) return seeds;

	vector<int> nodes = G.getAllNodes();
	unordered_map<int, int> id2idx;
	for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;
	const int N = nodes.size();

	vector<double> pos_th(N), neg_th(N), in_pos(N, 0.0), pos_cov(N, 0.0), neg_cov(N, 0.0);
	vector<vector<pair<int, double>>> edge_out(N);
	for (int i = 0; i < N; ++i) {
		int u = nodes[i];
		pos_th[i] = max(1e-6, G.getNodeThreshold(u));
		neg_th[i] = max(1e-6, fabs(G.getNodeThreshold2(u)));
		for (int v : G.getNodeInNeighbors(u)) {
			double w = G.getEdgeInfluence(v, u);
			if (w > 0) in_pos[i] += w;
		}
		for (int v : G.getNodeOutNeighbors(u)) {
			double w = G.getEdgeInfluence(u, v);
			edge_out[i].emplace_back(id2idx[v], w);
		}
	}

	vector<double> two_hop_score(N, 0.0);
	for (int i = 0; i < N; ++i) {
		for (const auto&[j, w1] : edge_out[i]) {
			for (const auto&[k, w2] : edge_out[j]) {
				double contrib = (w1 * w2) / pos_th[k];
				two_hop_score[i] += contrib;
			}
		}
	}

	vector<int> community_label(N, -1);
	int comm_id = 0;
	for (int i = 0; i < N; ++i) {
		if (community_label[i] != -1) continue;
		queue<int> q;
		q.push(i);
		community_label[i] = comm_id++;
		while (!q.empty()) {
			int u = q.front(); q.pop();
			for (const auto&[v, _] : edge_out[u]) {
				if (community_label[v] == -1) {
					community_label[v] = community_label[u];
					q.push(v);
				}
			}
		}
	}

	const double ALPHA = 0.6, LAMBDA = 1.2, PHI = 0.6, GAMMA = 0.25, DELTA = 0.2, ETA = 0.1;
	const double COMM_PENALTY = 0.2;
	vector<double> static_score(N);
	for (int i = 0; i < N; ++i) {
		static_score[i] = -GAMMA * pos_th[i] - DELTA * in_pos[i] + ETA * neg_th[i] + ALPHA * two_hop_score[i];
	}

	int givenSeed = -1;
	ifstream fin("given_pos.txt");
	if (fin >> givenSeed && id2idx.count(givenSeed)) {
		int i = id2idx[givenSeed];
		seeds.insert(givenSeed);
		for (const auto&[j, w] : edge_out[i]) {
			if (w > 0) pos_cov[j] += w / pos_th[j];
			else if (w < 0) neg_cov[j] += fabs(w) / neg_th[j];
		}
	}

	unordered_set<int> used_comm;

	auto score = [&](int u) -> double {
		if (seeds.count(u)) return -1e9;
		int i = id2idx[u];
		double s = static_score[i];
		for (const auto&[j, w] : edge_out[i]) {
			if (w > 0) {
				double add = w / pos_th[j];
				s += LAMBDA * min(1.0 - pos_cov[j], add);
			}
			else if (w < 0) {
				double add = fabs(w) / neg_th[j];
				s -= PHI * min(1.0 - neg_cov[j], add);
			}
		}
		if (used_comm.count(community_label[i])) s -= COMM_PENALTY;
		return s;
	};

	struct Entry {
		double gain;
		int node;
		int round;
		bool operator<(const Entry& e) const { return gain < e.gain; }
	};

	priority_queue<Entry> pq;
	for (int u : nodes) pq.push({ score(u), u, 0 });

	unordered_map<int, int> recalc_count;
	int round = 0;
	while (seeds.size() < numberOfSeeds && !pq.empty()) {
		Entry top = pq.top(); pq.pop();
		if (seeds.count(top.node)) continue;
		if (top.round < round) {
			if (++recalc_count[top.node] <= 2) {
				pq.push({ score(top.node), top.node, round });
			}
			continue;
		}
		seeds.insert(top.node);
		int i = id2idx[top.node];
		used_comm.insert(community_label[i]);
		for (const auto&[j, w] : edge_out[i]) {
			if (w > 0) pos_cov[j] = min(1.0, pos_cov[j] + w / pos_th[j]);
			else if (w < 0) neg_cov[j] = min(1.0, neg_cov[j] + fabs(w) / neg_th[j]);
		}
		++round;
	}

	return seeds;
}

#endif
