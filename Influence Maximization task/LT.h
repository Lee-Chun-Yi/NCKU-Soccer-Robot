#ifndef LT_H
#define LT_H

#include <unordered_set>
#include <vector>
#include "graph.h"
using namespace std;

// 計算正負活化鄰居對 target 的影響力
double influence_sum(DirectedGraph* G, int targetNode,
	const vector<int>& posActiveInNeighbors,
	const vector<int>& negActiveInNeighbors)
{
	double sum = 0.0;
	for (int u : posActiveInNeighbors)
		sum += G->getEdgeInfluence(u, targetNode);
	for (int u : negActiveInNeighbors)
		sum -= G->getEdgeInfluence(u, targetNode);
	return sum;
}

// 執行一輪 Signed LT 擴散
pair<unordered_set<int>, unordered_set<int>> diffuse_one_round_signed(
	DirectedGraph* G,
	const unordered_set<int>& pos_active,
	const unordered_set<int>& neg_active)
{
	unordered_set<int> new_pos, new_neg;

	for (int node : G->getAllNodes()) {
		if (pos_active.count(node) || neg_active.count(node)) continue;

		vector<int> pos_in, neg_in;
		for (int nbr : G->getNodeInNeighbors(node)) {
			if (pos_active.count(nbr)) pos_in.push_back(nbr);
			if (neg_active.count(nbr)) neg_in.push_back(nbr);
		}

		double infl = influence_sum(G, node, pos_in, neg_in);
		double pos_th = G->getNodeThreshold(node);
		double neg_th = G->getNodeThreshold2(node);

		if (infl >= pos_th) new_pos.insert(node);
		else if (infl <= neg_th) new_neg.insert(node);
	}

	return make_pair(new_pos, new_neg);
}

// 持續擴散直到穩定（安全的 C++14 版本）
void diffuse_signed_all(
	DirectedGraph* G,
	unordered_set<int> pos_active,
	unordered_set<int> neg_active,
	unordered_set<int>& final_pos,
	unordered_set<int>& final_neg)
{
	while (true) {
		pair<unordered_set<int>, unordered_set<int>> result =
			diffuse_one_round_signed(G, pos_active, neg_active);

		const unordered_set<int>& new_pos = result.first;
		const unordered_set<int>& new_neg = result.second;

		if (new_pos.empty() && new_neg.empty()) break;

		pos_active.insert(new_pos.begin(), new_pos.end());
		neg_active.insert(new_neg.begin(), new_neg.end());
	}

	final_pos = pos_active;
	final_neg = neg_active;
}

#endif
