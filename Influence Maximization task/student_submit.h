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
#include <chrono>
#include <iostream>
#include <numeric>
#include <iterator>

using namespace std;
using Clock = chrono::steady_clock;

enum SizeClass { SMALL, MEDIUM, LARGE };

vector<int> rankNodesByHeuristic(DirectedGraph& G) {
        vector<int> nodes = G.getAllNodes();
        vector<double> score(nodes.size(), 0.0);
        for (size_t i = 0; i < nodes.size(); ++i) {
                int u = nodes[i];
                double posTh = max(1e-6, G.getNodeThreshold(u));
                double negTh = fabs(G.getNodeThreshold2(u));
                const vector<int> outs = G.getNodeOutNeighbors(u);
                double posSum = 0.0, negSum = 0.0;
                for (int v : outs) {
                        double w = G.getEdgeInfluence(u, v);
                        if (w >= 0) posSum += w;
                        else negSum += fabs(w);
                }
                score[i] = posSum - 0.35 * posTh - 0.25 * negTh - 0.15 * negSum + 0.02 * outs.size();
        }
        vector<int> order(nodes.size());
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int a, int b) {
                if (fabs(score[a] - score[b]) > 1e-9) return score[a] > score[b];
                return nodes[a] < nodes[b];
        });
        vector<int> ranked;
        ranked.reserve(nodes.size());
        for (int idx : order) ranked.push_back(nodes[idx]);
        return ranked;
}

SizeClass classifyGraphSize(DirectedGraph& G) {
	int nodeCount = G.getSize();
	int edgeCount = 0;
	for (int u : G.getAllNodes()) {
		edgeCount += G.getNodeOutNeighbors(u).size();
	}
	if (nodeCount <= 150) return SMALL;
	if (nodeCount <= 1500) return MEDIUM;
	return LARGE;
}

unordered_set<int> seedSelectionSmall(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	vector<int> nodes = G.getAllNodes();
	unordered_map<int, int> id2idx;
	for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;
	const int N = nodes.size();
	vector<double> score(N, 0.0);
	for (int i = 0; i < N; ++i) {
		for (int v : G.getNodeOutNeighbors(nodes[i])) {
			double w = G.getEdgeInfluence(nodes[i], v);
			score[i] += max(0.0, w);
		}
	}
	vector<int> order(N);
	iota(order.begin(), order.end(), 0);
	sort(order.begin(), order.end(), [&](int a, int b) { return score[a] > score[b]; });
	for (int i = 0; i < N && seeds.size() < numberOfSeeds; ++i) seeds.insert(nodes[order[i]]);
	return seeds;
}

unordered_set<int> seedSelectionMedium(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	vector<int> nodes = G.getAllNodes();
	unordered_map<int, int> id2idx;
	for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;
	const int N = nodes.size();

	vector<double> outSum(N, 0.0), posTh(N);
	for (int i = 0; i < N; ++i) {
		int u = nodes[i];
		posTh[i] = max(1e-6, G.getNodeThreshold(u));
		for (int v : G.getNodeOutNeighbors(u)) {
			double w = G.getEdgeInfluence(u, v);
			if (w > 0) outSum[i] += w;
		}
	}

	vector<double> fastScore(N);
	for (int i = 0; i < N; ++i) {
		fastScore[i] = outSum[i] - 0.5 * posTh[i];
	}

	vector<int> order(N);
	iota(order.begin(), order.end(), 0);
	sort(order.begin(), order.end(), [&](int a, int b) { return fastScore[a] > fastScore[b]; });

	int simLimit = min((int)N, 1000);
	unordered_set<int> picked;
	vector<double> marginal(N);
	for (int i = 0; i < simLimit; ++i) {
		int u = nodes[order[i]];
		if (picked.count(u)) continue;
		picked.insert(u);
		unordered_set<int> tmpSeed = { u };
		unordered_set<int> actPos, actNeg;
		diffuse_signed_all(&G, tmpSeed, {}, actPos, actNeg);
		marginal[order[i]] = actPos.size() - actNeg.size();
	}
	sort(order.begin(), order.end(), [&](int a, int b) { return marginal[a] > marginal[b]; });
	for (int i = 0; i < N && seeds.size() < numberOfSeeds; ++i) seeds.insert(nodes[order[i]]);
	return seeds;
}

unordered_set<int> seedSelectionLarge(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	vector<int> nodes = G.getAllNodes();
	unordered_map<int, int> id2idx;
	for (size_t i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;
	const int N = nodes.size();

	vector<double> outSum(N, 0.0), posTh(N);
	vector<vector<pair<int, double>>> edge_out(N);
	for (int i = 0; i < N; ++i) {
		int u = nodes[i];
		posTh[i] = max(1e-6, G.getNodeThreshold(u));
		for (int v : G.getNodeOutNeighbors(u)) {
			double w = G.getEdgeInfluence(u, v);
			if (w > 0) outSum[i] += w;
			edge_out[i].emplace_back(id2idx[v], w);
		}
	}

	vector<double> negRisk(N, 0.0);
        for (int i = 0; i < N; ++i) {
                for (auto& edgeInfo : edge_out[i]) {
                        double w = edgeInfo.second;
                        if (w < 0) negRisk[i] += fabs(w);
                }
        }

	vector<int> comm(N, -1);
	int commId = 0;
	for (int i = 0; i < N; ++i) {
		if (comm[i] != -1) continue;
		queue<int> q;
		q.push(i);
		comm[i] = commId++;
                while (!q.empty()) {
                        int u = q.front(); q.pop();
                        for (auto& edgeInfo : edge_out[u]) {
                                int v = edgeInfo.first;
                                if (comm[v] == -1) {
                                        comm[v] = comm[u];
                                        q.push(v);
                                }
                        }
                }
	}

	vector<double> fastScore(N);
	for (int i = 0; i < N; ++i) {
		fastScore[i] = outSum[i] - 0.5 * posTh[i] - 0.4 * negRisk[i];
	}

	vector<int> order(N);
	iota(order.begin(), order.end(), 0);
	sort(order.begin(), order.end(), [&](int a, int b) { return fastScore[a] > fastScore[b]; });

	int simLimit = min((int)N, 1500);
	unordered_set<int> picked;
	vector<double> marginal(N);
	for (int i = 0; i < simLimit; ++i) {
		int u = nodes[order[i]];
		if (picked.count(u)) continue;
		picked.insert(u);
		unordered_set<int> tmpSeed = { u };
		unordered_set<int> actPos, actNeg;
		diffuse_signed_all(&G, tmpSeed, {}, actPos, actNeg);
		double gain = actPos.size() - actNeg.size();
		if (seeds.count(u)) gain -= 2.0;
		marginal[order[i]] = gain;
	}
	sort(order.begin(), order.end(), [&](int a, int b) { return marginal[a] > marginal[b]; });
	unordered_set<int> usedComm;
	for (int i = 0; i < N && seeds.size() < numberOfSeeds; ++i) {
		int u = nodes[order[i]];
		if (usedComm.count(comm[order[i]])) continue;
		seeds.insert(u);
		usedComm.insert(comm[order[i]]);
	}
	return seeds;
}

unordered_set<int> autoStrategySeedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
        SizeClass sz = classifyGraphSize(G);
        if (sz == SMALL) return seedSelectionSmall(G, numberOfSeeds);
        if (sz == MEDIUM) return seedSelectionMedium(G, numberOfSeeds);
        return seedSelectionLarge(G, numberOfSeeds);
}

unordered_set<int> seedSelection(DirectedGraph& G,
                unsigned int numberOfSeeds,
                int givenPosSeed,
                const unordered_set<int>& givenNegSeeds) {
        unordered_set<int> forbidden = givenNegSeeds;
        forbidden.insert(givenPosSeed);

        unordered_set<int> initial = autoStrategySeedSelection(G, min<unsigned int>(numberOfSeeds, G.getSize()));
        for (int f : forbidden) initial.erase(f);

        vector<int> ranking = rankNodesByHeuristic(G);
        unordered_set<int> result;
        result.reserve(numberOfSeeds * 2);

        for (int node : ranking) {
                if (result.size() == numberOfSeeds) break;
                if (forbidden.count(node)) continue;
                if (initial.count(node)) result.insert(node);
        }
        for (int node : ranking) {
                if (result.size() == numberOfSeeds) break;
                if (forbidden.count(node) || result.count(node)) continue;
                result.insert(node);
        }

        if (result.size() < numberOfSeeds) {
                for (int node : ranking) {
                        if (result.size() == numberOfSeeds) break;
                        if (forbidden.count(node)) continue;
                        result.insert(node);
                }
        }

        if (result.size() > numberOfSeeds) {
                vector<int> ordered(result.begin(), result.end());
                sort(ordered.begin(), ordered.end(), [&](int a, int b) {
                        auto ia = find(ranking.begin(), ranking.end(), a);
                        auto ib = find(ranking.begin(), ranking.end(), b);
                        return distance(ranking.begin(), ia) < distance(ranking.begin(), ib);
                });
                unordered_set<int> trimmed;
                for (int node : ordered) {
                        if (trimmed.size() == numberOfSeeds) break;
                        trimmed.insert(node);
                }
                result.swap(trimmed);
        }

        return result;
}

#endif
