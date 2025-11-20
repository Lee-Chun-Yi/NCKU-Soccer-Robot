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

/**
 * @brief CELF 用的節點增益結構
 */
struct NodeGain {
	double score;   // 邊際增益
	int    node_id; // 節點 ID
	int    round;   // 此分數是在第幾輪 k 計算

	bool operator<(const NodeGain& other) const {
		return score < other.score; // for max-heap
	}
};

/*
 * seedSelection:
 *   - G: DirectedGraph (loaded externally)
 *   - numberOfSeeds: how many positive seeds to select (should be 9)
 *   - givenPosSeed: provided positive seed (avoid duplicates)
 *   - givenNegSeeds: provided negative seeds (avoid)
 *
 * Return an unordered_set<int> containing exactly numberOfSeeds distinct ids.
 * No I/O; deterministic heuristic.
 */
unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds,
	int givenPosSeed, const unordered_set<int>& givenNegSeeds) {
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	// 取得所有節點
	vector<int> nodes = G.getAllNodes();
	if (nodes.empty()) return seeds;

	// å·²ç´å®ç¡æ³ç¨ä¹ç¨®å­
	unordered_set<int> forbidden;
	forbidden.insert(givenPosSeed);
	for (int neg : givenNegSeeds) {
		forbidden.insert(neg);
	}

	// node id -> index
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(nodes.size() * 2);
	for (size_t i = 0; i < nodes.size(); ++i) {
		id2idx[nodes[i]] = i;
	}

	const double EPS = 1e-12;
	const size_t N = nodes.size();

	// 預先計算：
	//  - 正門檻 pos_th
	//  - 負門檻絕對值 neg_th_mag
	//  - 入邊正向強度 in_strength
	vector<double> pos_th(N, 0.0);
	vector<double> neg_th_mag(N, 0.0);
	vector<double> in_strength(N, 0.0);

	for (size_t i = 0; i < N; ++i) {
		int u = nodes[i];
		double pth = G.getNodeThreshold(u);
		double nth = G.getNodeThreshold2(u);

		pos_th[i] = std::max(EPS, pth);
		neg_th_mag[i] = std::max(EPS, std::fabs(nth));

		double sum_in = 0.0;
		vector<int> innei = G.getNodeInNeighbors(u);
		for (size_t t = 0; t < innei.size(); ++t) {
			int p = innei[t];
			double w = G.getEdgeInfluence(p, u);
			if (w > 0.0) sum_in += w;
		}
		in_strength[i] = sum_in;
	}

	// 正向 / 負向覆蓋率
	vector<double> pos_coverage(N, 0.0);
	vector<double> neg_coverage(N, 0.0);

	// 啟發式權重（可以微調）
	const double LAMBDA = 1.6;   // 正覆蓋權重
	const double PHI = 0.8;   // 負覆蓋懲罰（略保守）
	const double GAMMA = 0.1;  // 自身正門檻懲罰
	const double DELTA = 0.03;  // 入邊懲罰（避免冗餘）
	const double ETA = 0.3;  // 獎勵負門檻大的節點（較不易被負翻）

	// 計算節點 u 的邊際啟發式分數
	auto marginal_gain = [&](int u) -> double {
		if (seeds.count(u) || forbidden.count(u)) {
			return -std::numeric_limits<double>::infinity();
		}

		unordered_map<int, size_t>::iterator it_u = id2idx.find(u);
		if (it_u == id2idx.end()) {
			return -std::numeric_limits<double>::infinity();
		}
		size_t iu = it_u->second;

		double score = 0.0;

		// 自身屬性：門檻高 / 入邊強 會扣分；負門檻高則加一點分
		score -= GAMMA * pos_th[iu];
		score -= DELTA * in_strength[iu];
		score += ETA * neg_th_mag[iu];

		// 看出鄰居
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (size_t t = 0; t < outnei.size(); ++t) {
			int v = outnei[t];
			unordered_map<int, size_t>::iterator it_v = id2idx.find(v);
			if (it_v == id2idx.end()) continue;
			size_t j = it_v->second;

			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				// 正向邊：對 v 的正覆蓋
				double need = 1.0 - pos_coverage[j];
				if (need < 0.0) need = 0.0;
				if (need > 0.0) {
					double add = w / pos_th[j];   // 此邊對 v 門檻的覆蓋比例
					if (add > need) add = need;
					score += LAMBDA * add;
				}
			}
			else if (w < -EPS) {
				// 負向邊：這個種子會對 v 增加變負的壓力
				double need_neg = 1.0 - neg_coverage[j];
				if (need_neg < 0.0) need_neg = 0.0;
				if (need_neg > 0.0) {
					double add_neg = std::fabs(w) / neg_th_mag[j];
					if (add_neg > need_neg) add_neg = need_neg;
					score -= PHI * add_neg;
				}
			}
		}

		return score;
	};

	// 將 u 真正選為種子後，更新覆蓋率
	auto apply_influence = [&](int u) {
		unordered_map<int, size_t>::iterator it_u = id2idx.find(u);
		if (it_u == id2idx.end()) return;

		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (size_t t = 0; t < outnei.size(); ++t) {
			int v = outnei[t];
			unordered_map<int, size_t>::iterator it_v = id2idx.find(v);
			if (it_v == id2idx.end()) continue;
			size_t j = it_v->second;

			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				double add = w / pos_th[j];
				if (add > 0.0) {
					pos_coverage[j] += add;
					if (pos_coverage[j] > 1.0) pos_coverage[j] = 1.0;
				}
			}
			else if (w < -EPS) {
				double add_neg = std::fabs(w) / neg_th_mag[j];
				if (add_neg > 0.0) {
					neg_coverage[j] += add_neg;
					if (neg_coverage[j] > 1.0) neg_coverage[j] = 1.0;
				}
			}
		}
	};

	// === CELF: 初始計算所有節點的分數，放入 max-heap ===
	priority_queue<NodeGain> pq;
	for (size_t i = 0; i < nodes.size(); ++i) {
		int u = nodes[i];
		if (forbidden.count(u)) continue;
		double sc = marginal_gain(u);
		NodeGain ng;
		ng.score = sc;
		ng.node_id = u;
		ng.round = 0;
		pq.push(ng);
	}

	// === CELF 主迴圈：選 numberOfSeeds 個種子 ===
	for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {
		int bestNode = -1;

		// 找到本輪真正最新、最高分的節點
		while (!pq.empty()) {
			NodeGain top = pq.top();
			pq.pop();

			// 若這個分數是前一輪以前算的，就需重算
			if (top.round < (int)k) {
				double new_sc = marginal_gain(top.node_id);
				NodeGain ng;
				ng.score = new_sc;
				ng.node_id = top.node_id;
				ng.round = (int)k;
				pq.push(ng);
				continue;
			}
			else {
				// round == k，為本輪有效分數
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
