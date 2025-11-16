// 0955
// optimized_student_style.h
#ifndef OPTIMIZED_STUDENT_STYLE_H
#define OPTIMIZED_STUDENT_STYLE_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <string>

using namespace std;

// �֨��� key �ʸ�
string setToKey(const unordered_set<int>& s) {
	vector<int> v(s.begin(), s.end());
	sort(v.begin(), v.end());
	string key;
	for (int x : v) key += to_string(x) + ",";
	return key;
}

double simulateSpread(const DirectedGraph& G, const unordered_set<int>& pos, const unordered_set<int>& neg, int rounds = 100) {
	double sum = 0.0;
	unordered_set<int> finalPos, finalNeg;
	for (int i = 0; i < rounds; ++i) {
		finalPos.clear(); finalNeg.clear();
		diffuse_signed_all(&G, pos, neg, finalPos, finalNeg);
		sum += finalPos.size() - finalNeg.size();
	}
	return sum / rounds;
}

unordered_set<int> studentCELFOptimized(DirectedGraph& G, unsigned int K, int given = -1, const unordered_set<int>& negs = {}) {
	unordered_set<int> result;
	if (K == 0) return result;

	vector<int> nodes = G.getAllNodes();
	unordered_map<int, int> id2idx;
	for (int i = 0; i < nodes.size(); ++i) id2idx[nodes[i]] = i;

	unordered_set<int> selected;
	if (given >= 0) selected.insert(given);

	vector<int> candidates;
	for (int u : nodes) {
		if (negs.count(u) || selected.count(u)) continue;
		candidates.push_back(u);
	}

	// fastScore �w��]��������ơ^
	unordered_map<int, double> fastScore;
	for (int u : candidates) {
		double s = G.getNodeOutNeighbors(u).size() - 0.4 * G.getNodeThreshold(u);
		fastScore[u] = s;
	}
	sort(candidates.begin(), candidates.end(), [&](int a, int b) {
		return fastScore[a] > fastScore[b];
	});
	if (candidates.size() > 1500) candidates.resize(1500);

	unordered_map<string, double> spreadCache;

	struct NodeGain {
		int id;
		double gain;
		double total;
		int updated;
	};

	auto cmp = [](const NodeGain& a, const NodeGain& b) { return a.gain < b.gain; };
	priority_queue<NodeGain, vector<NodeGain>, decltype(cmp)> pq(cmp);

	string baseKey = setToKey(selected);
	double baseSpread = spreadCache.count(baseKey) ? spreadCache[baseKey] : simulateSpread(G, selected, negs);
	if (!spreadCache.count(baseKey)) spreadCache[baseKey] = baseSpread;

	for (int u : candidates) {
		unordered_set<int> s = selected;
		s.insert(u);
		string key = setToKey(s);
		double sc = spreadCache.count(key) ? spreadCache[key] : simulateSpread(G, s, negs);
		if (!spreadCache.count(key)) spreadCache[key] = sc;
		pq.push({ u, sc - baseSpread, sc, 0 });
	}

	int round = 0;
	int maxCELFRound = int(K * 0.95);
	while (result.size() < K && !pq.empty() && round < maxCELFRound) {
		NodeGain top = pq.top(); pq.pop();
		if (selected.count(top.id)) continue;

		if (top.updated == round) {
			result.insert(top.id);
			selected.insert(top.id);
			++round;
			baseKey = setToKey(selected);
			baseSpread = spreadCache.count(baseKey) ? spreadCache[baseKey] : simulateSpread(G, selected, negs);
			if (!spreadCache.count(baseKey)) spreadCache[baseKey] = baseSpread;
		}
		else {
			unordered_set<int> s = selected;
			s.insert(top.id);
			string key = setToKey(s);
			double sc = spreadCache.count(key) ? spreadCache[key] : simulateSpread(G, s, negs);
			if (!spreadCache.count(key)) spreadCache[key] = sc;
			pq.push({ top.id, sc - baseSpread, sc, round });
		}
	}

	return result;
}

#endif#pragma once
