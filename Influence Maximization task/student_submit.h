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
#include <random>

using namespace std;
using Clock = chrono::steady_clock;

enum SizeClass { SMALL, MEDIUM, LARGE };

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

double estimateSpreadMC(DirectedGraph& G, const unordered_set<int>& posSeeds, int rounds = 300) {
	double total = 0.0;
	unordered_set<int> actPos, actNeg;
	for (int i = 0; i < rounds; ++i) {
		actPos.clear(); actNeg.clear();
		diffuse_signed_all(&G, posSeeds, {}, actPos, actNeg);
		total += (double)actPos.size() - actNeg.size();
	}
	return total / rounds;
}

unordered_set<int> seedSelectionSmall(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	vector<int> nodes = G.getAllNodes();
	const int N = nodes.size();
	vector<double> score(N);
	for (int i = 0; i < N; ++i) {
		unordered_set<int> oneSeed = { nodes[i] };
		score[i] = estimateSpreadMC(G, oneSeed, 800);
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
	for (int i = 0; i < N; ++i) fastScore[i] = outSum[i] - 0.5 * posTh[i];

	vector<int> order(N);
	iota(order.begin(), order.end(), 0);
	sort(order.begin(), order.end(), [&](int a, int b) { return fastScore[a] > fastScore[b]; });

	const int simLimit = min((int)N, 500);
	vector<double> spread(N, 0.0);
	for (int i = 0; i < simLimit; ++i) {
		unordered_set<int> oneSeed = { nodes[order[i]] };
		spread[order[i]] = estimateSpreadMC(G, oneSeed, 300);
	}
	sort(order.begin(), order.end(), [&](int a, int b) { return spread[a] > spread[b]; });
	for (int i = 0; i < N && seeds.size() < numberOfSeeds; ++i) seeds.insert(nodes[order[i]]);
	return seeds;
}

unordered_set<int> seedSelectionLarge(DirectedGraph& G, unsigned int numberOfSeeds) {
	unordered_set<int> seeds;
	vector<int> nodes = G.getAllNodes();
	const int N = nodes.size();
	vector<double> score(N);

	const int simLimit = min((int)N, 800);
	vector<int> order(N);
	iota(order.begin(), order.end(), 0);
	vector<double> spread(N);
	for (int i = 0; i < simLimit; ++i) {
		unordered_set<int> s = { nodes[i] };
		spread[i] = estimateSpreadMC(G, s, 150);
	}
	sort(order.begin(), order.end(), [&](int a, int b) { return spread[a] > spread[b]; });

	struct Entry {
		int node;
		double gain;
		double total;
		int updatedAt;
	};
	struct Compare {
		bool operator()(const Entry& a, const Entry& b) const {
			return a.gain < b.gain;
		}
	};

	priority_queue<Entry, vector<Entry>, Compare> pq;
	for (int i = 0; i < simLimit; ++i) {
		pq.push({ nodes[order[i]], spread[order[i]], spread[order[i]], 0 });
	}

	int round = 0;
	unordered_set<int> workingSeeds;
	int maxCELFRounds = numberOfSeeds * 95 / 100;
	while ((int)seeds.size() < (int)numberOfSeeds && !pq.empty() && round < maxCELFRounds) {
		auto top = pq.top(); pq.pop();
		if (top.updatedAt == round) {
			seeds.insert(top.node);
			workingSeeds.insert(top.node);
			++round;
		}
		else {
			unordered_set<int> temp = workingSeeds;
			temp.insert(top.node);
			static unordered_map<string, double> spreadCache;
			string tempKey, workKey;
			for (int n : temp) tempKey += to_string(n) + ",";
			for (int n : workingSeeds) workKey += to_string(n) + ",";
			double base = spreadCache.count(workKey) ? spreadCache[workKey] : estimateSpreadMC(G, workingSeeds, 150);
			if (!spreadCache.count(workKey)) spreadCache[workKey] = base;
			double newSpread = spreadCache.count(tempKey) ? spreadCache[tempKey] : estimateSpreadMC(G, temp, 150);
			if (!spreadCache.count(tempKey)) spreadCache[tempKey] = newSpread;
			double marginalGain = newSpread - base;
			pq.push({ top.node, marginalGain, newSpread, round });
		}
	}
	return seeds;
}

unordered_set<int> autoStrategySeedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
	SizeClass sz = classifyGraphSize(G);
	if (sz == SMALL) return seedSelectionSmall(G, numberOfSeeds);
	if (sz == MEDIUM) return seedSelectionMedium(G, numberOfSeeds);
	return seedSelectionLarge(G, numberOfSeeds);
}

#endif
#pragma once
