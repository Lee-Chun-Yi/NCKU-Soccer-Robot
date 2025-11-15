#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

//-------------------------------------------
// CELF 結構
//-------------------------------------------
struct NodeGain {
	double score;
	int node_id;
	int round;
	bool operator<(const NodeGain& other) const {
		return score < other.score; // max-heap
	}
};

//-------------------------------------------
// 主函式
//-------------------------------------------
unordered_set<int> seedSelection(
	DirectedGraph& G,
	unsigned int numberOfSeeds,
	int givenPosSeed,
	const unordered_set<int>& givenNegSeeds)
{
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	vector<int> nodes = G.getAllNodes();
	size_t N = nodes.size();
	if (N == 0) return seeds;

	const double EPS = 1e-12;

	//-------------------------------------------
	// node id → index
	//-------------------------------------------
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(N * 2);
	for (size_t i = 0; i < N; i++) id2idx[nodes[i]] = i;

	//-------------------------------------------
	// 前處理：threshold、in-/out-strength
	//-------------------------------------------
	vector<double> pos_th(N), neg_th_mag(N);
	vector<double> in_pos_strength(N, 0.0);
	vector<double> in_neg_strength(N, 0.0);
	vector<double> out_pos_strength(N, 0.0);
	vector<double> out_neg_strength(N, 0.0);
	vector<int> outdeg(N, 0);

	for (size_t i = 0; i < N; i++)
	{
		int u = nodes[i];
		double pth = G.getNodeThreshold(u);
		double nth = G.getNodeThreshold2(u);

		pos_th[i] = max(EPS, pth);
		neg_th_mag[i] = max(EPS, fabs(nth));

		vector<int> innei = G.getNodeInNeighbors(u);
		for (int p : innei) {
			double w = G.getEdgeInfluence(p, u);
			if (w > 0) in_pos_strength[i] += w;
			else       in_neg_strength[i] += fabs(w);
		}

		vector<int> outnei = G.getNodeOutNeighbors(u);
		outdeg[i] = (int)outnei.size();
		for (int v : outnei) {
			double w = G.getEdgeInfluence(u, v);
			if (w > 0) out_pos_strength[i] += w;
			else       out_neg_strength[i] += fabs(w);
		}
	}

	//-------------------------------------------
	// 覆蓋率
	//-------------------------------------------
	vector<double> pos_cov(N, 0.0);
	vector<double> neg_cov(N, 0.0);

	//-------------------------------------------
	// ★★★ 重點策略 1：Hazard Score（負向爆炸源判定）
	//-------------------------------------------
	vector<double> hazard(N, 0.0);
	for (size_t i = 0; i < N; i++)
	{
		// risk: 能把負向往外噴
		double risk = out_neg_strength[i];

		// resistance: 自己不容易變負
		double resist = neg_th_mag[i] + 0.5 * pos_th[i];

		hazard[i] = risk / resist;  // 越大越危險
	}

	//-------------------------------------------
	// ★★★ 重點策略 2：Stability Score（局部穩定度）
	//-------------------------------------------
	vector<double> stability(N, 0.0);
	for (size_t i = 0; i < N; i++)
	{
		double safe = neg_th_mag[i];
		double danger = in_neg_strength[i] + out_neg_strength[i];
		stability[i] = safe / (1.0 + danger);  // 越大越不容易變負
	}

	//-------------------------------------------
	// forbidden nodes
	//-------------------------------------------
	auto is_forbidden = [&](int u) {
		if (u == givenPosSeed) return true;
		return (givenNegSeeds.count(u) > 0);
	};

	//-------------------------------------------
	// ★★★ 重點策略 3：Seed Interference 過濾
	//-------------------------------------------
	auto seed_interference = [&](int cand) {
		auto it = id2idx.find(cand);
		if (it == id2idx.end()) return 1e9; // 不合法
		size_t ic = it->second;

		double sum = 0.0;
		for (int s : seeds)
		{
			size_t is = id2idx.at(s);

			// cand -> s
			double ws = G.getEdgeInfluence(cand, s);
			if (ws < 0) sum += fabs(ws) / neg_th_mag[is];

			// s -> cand
			double w2 = G.getEdgeInfluence(s, cand);
			if (w2 < 0) sum += fabs(w2) / neg_th_mag[ic];
		}
		return sum; // 越大越不能選
	};

	//-------------------------------------------
	// Marginal Gain
	//-------------------------------------------
	const double LAMBDA = 1.25;
	const double PHI = 1.15;
	const double GAMMA = 0.22;
	const double DELTA = 0.18;
	const double ETA = 0.55;   // 強調負門檻
	const double BETA_S = 0.90;   // stability 權重
	const double BETA_H = -1.20;  // hazard 懲罰
	const double BETA_D = 0.05;   // out-degree 小獎勵
	const double INTERFERE_LIMIT = 0.35;

	auto marginal_gain = [&](int u) {
		if (seeds.count(u) || is_forbidden(u))
			return -numeric_limits<double>::infinity();

		size_t iu = id2idx[u];

		// ★ interference 過濾
		double inter = seed_interference(u);
		if (inter > INTERFERE_LIMIT)
			return -numeric_limits<double>::infinity();

		double score = 0.0;

		// node 特性
		score -= GAMMA * pos_th[iu];
		score -= DELTA * in_pos_strength[iu];
		score += ETA * neg_th_mag[iu];   // 越難變負越好

		// ★ 加入穩定度 / hazard
		score += BETA_S * stability[iu];
		score += BETA_H * hazard[iu];

		// 小幅 out-degree prior
		score += BETA_D * outdeg[iu];

		// 1-hop 傳播估計
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (int v : outnei)
		{
			size_t iv = id2idx[v];
			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				double need = max(0.0, 1.0 - pos_cov[iv]);
				if (need > 0.0) {
					double add = w / pos_th[iv];
					score += LAMBDA * min(need, add);
				}
			}
			else if (w < -EPS) {
				double need = max(0.0, 1.0 - neg_cov[iv]);
				if (need > 0.0) {
					double add = fabs(w) / neg_th_mag[iv];
					score -= PHI * min(need, add);
				}
			}
		}
		return score;
	};

	//-------------------------------------------
	// apply influence
	//-------------------------------------------
	auto apply_influence = [&](int u) {
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (int v : outnei) {
			size_t iv = id2idx[v];
			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				pos_cov[iv] = min(1.0, pos_cov[iv] + w / pos_th[iv]);
			}
			else if (w < -EPS) {
				neg_cov[iv] = min(1.0, neg_cov[iv] + fabs(w) / neg_th_mag[iv]);
			}
		}
	};

	//-------------------------------------------
	// initial apply: given positive seed
	//-------------------------------------------
	if (id2idx.count(givenPosSeed)) apply_influence(givenPosSeed);

	//-------------------------------------------
	// CELF
	//-------------------------------------------
	priority_queue<NodeGain> pq;
	for (int u : nodes)
		pq.push(NodeGain{ marginal_gain(u), u, 0 });

	for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); k++)
	{
		int bestNode = -1;
		while (true)
		{
			if (pq.empty()) break;

			NodeGain top = pq.top();
			pq.pop();

			if (top.round < (int)k) {
				pq.push(NodeGain{ marginal_gain(top.node_id), top.node_id, (int)k });
			}
			else {
				bestNode = top.node_id;
				break;
			}
		}

		if (bestNode == -1 || is_forbidden(bestNode))
			break;

		seeds.insert(bestNode);
		apply_influence(bestNode);
	}

	return seeds;
}

#endif
