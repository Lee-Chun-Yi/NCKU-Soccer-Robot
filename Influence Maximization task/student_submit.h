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
#include <queue>   // CELF 需要 priority_queue

using namespace std;

/**
 * @brief CELF 演算法所需的結構
 */
struct NodeGain {
	double score;   // 邊際增益
	int    node_id; // 節點 ID
	int    round;   // 此分數是在第幾輪 k 計算出來的

	// priority_queue 要成為 max-heap（score 越大越優先）
	bool operator<(const NodeGain &other) const {
		return score < other.score;
	}
};

/**
 * @brief 影響力最大化主函數
 *
 * @param G              評測系統傳入的圖
 * @param numberOfSeeds  要額外選擇的正向種子數量 K
 * @param givenPosSeed   資料集中原本給定的正向種子
 * @param givenNegSeeds  資料集中給定的負向種子集合
 * @return unordered_set<int>  回傳 K 個額外種子節點 ID
 */
unordered_set<int> seedSelection(DirectedGraph &G,
	unsigned int numberOfSeeds,
	int givenPosSeed,
	const unordered_set<int> &givenNegSeeds)
{
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	// 取得所有節點
	vector<int> nodes = G.getAllNodes();
	if (nodes.empty()) return seeds;

	const size_t N = nodes.size();
	const double EPS = 1e-12;

	// node id -> index (方便存取各種 vector)
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(N * 2);
	for (size_t i = 0; i < N; ++i) {
		id2idx[nodes[i]] = i;
	}

	// ------------------------------------------------------------
	// 前處理：各節點的門檻與 in-edge 強度
	// ------------------------------------------------------------
	vector<double> pos_th(N, 0.0);      // 正門檻
	vector<double> neg_th_mag(N, 0.0);  // 負門檻絕對值
	vector<double> in_strength(N, 0.0); // 正向 in-edge 權重總和

	for (size_t i = 0; i < N; ++i) {
		int u = nodes[i];

		double pth = G.getNodeThreshold(u);   // 正門檻
		double nth = G.getNodeThreshold2(u);  // 負門檻（多半為負）

		pos_th[i] = std::max(EPS, pth);
		neg_th_mag[i] = std::max(EPS, std::fabs(nth));

		double sum_in = 0.0;
		vector<int> innei = G.getNodeInNeighbors(u);
		for (size_t k = 0; k < innei.size(); ++k) {
			int p = innei[k];
			double w = G.getEdgeInfluence(p, u);
			if (w > 0.0) sum_in += w;
		}
		in_strength[i] = sum_in;
	}

	// ------------------------------------------------------------
	// 覆蓋率：目前所有種子對每個節點的正/負影響程度
	// ------------------------------------------------------------
	vector<double> pos_coverage(N, 0.0); // 0 ~ 1
	vector<double> neg_coverage(N, 0.0); // 0 ~ 1

	// ------------------------------------------------------------
	// 啟發式權重參數（可以微調）
	// ------------------------------------------------------------
	const double LAMBDA = 1.0;   // 正向覆蓋增益
	const double PHI = 1.0;   // 負向覆蓋懲罰
	const double GAMMA = 0.25;  // 自身正門檻懲罰
	const double DELTA = 0.20;  // in-edge 正向強度懲罰
	const double ETA = 0.20;  // 不易變負（負門檻大）獎勵

	// 2-hop 相關參數
	const double TWO_HOP_DECAY = 0.5; // 2-hop 路徑衰減
	const double BETA1 = 0.6; // one-hop 結構分數權重
	const double BETA2 = 0.4; // two-hop 結構分數權重

	// ------------------------------------------------------------
	// one-hop / two-hop 結構分數（與 coverage 無關的先驗）
	// ------------------------------------------------------------
	vector<double> one_hop_score(N, 0.0);
	vector<double> two_hop_score(N, 0.0);

	for (size_t iu = 0; iu < N; ++iu) {
		int u = nodes[iu];
		vector<int> out_u = G.getNodeOutNeighbors(u);

		for (size_t a = 0; a < out_u.size(); ++a) {
			int v = out_u[a];
			unordered_map<int, size_t>::const_iterator itv = id2idx.find(v);
			if (itv == id2idx.end()) continue;
			size_t iv = itv->second;

			double w_uv = G.getEdgeInfluence(u, v);
			if (w_uv <= EPS) continue;                 // 只看正向邊
			double contrib1 = w_uv / pos_th[iv];

			one_hop_score[iu] += contrib1;

			vector<int> out_v = G.getNodeOutNeighbors(v);
			for (size_t b = 0; b < out_v.size(); ++b) {
				int w = out_v[b];
				unordered_map<int, size_t>::const_iterator itw = id2idx.find(w);
				if (itw == id2idx.end()) continue;
				size_t iw = itw->second;

				double w_vw = G.getEdgeInfluence(v, w);
				if (w_vw <= EPS) continue;             // 同樣只看正向邊
				double contrib2 = w_vw / pos_th[iw];

				two_hop_score[iu] += TWO_HOP_DECAY * contrib1 * contrib2;
			}
		}
	}

	// ------------------------------------------------------------
	// 不能被選為種子的節點（原本的正/負種子）
	// ------------------------------------------------------------
	auto is_forbidden = [&](int node) -> bool {
		if (node == givenPosSeed) return true;
		return givenNegSeeds.find(node) != givenNegSeeds.end();
	};

	// ------------------------------------------------------------
	// Lambda: 計算節點 u 在目前 coverage 下的邊際增益
	// ------------------------------------------------------------
	auto marginal_gain = [&](int u) -> double {
		if (seeds.count(u) || is_forbidden(u)) {
			return -std::numeric_limits<double>::infinity();
		}

		unordered_map<int, size_t>::const_iterator it_u = id2idx.find(u);
		if (it_u == id2idx.end()) {
			return -std::numeric_limits<double>::infinity();
		}
		size_t iu = it_u->second;

		double score = 0.0;

		// 節點自身性質：門檻越高或 in-edge 越強，越不適合當種子
		score -= GAMMA * pos_th[iu];
		score -= DELTA * in_strength[iu];
		score += ETA * neg_th_mag[iu];

		// 全域結構：一、二層鄰居的 broadcast 能力
		score += BETA1 * one_hop_score[iu];
		score += BETA2 * two_hop_score[iu];

		// 1-hop 覆蓋增益 + 負向風險
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (size_t t = 0; t < outnei.size(); ++t) {
			int v = outnei[t];
			unordered_map<int, size_t>::const_iterator it_v = id2idx.find(v);
			if (it_v == id2idx.end()) continue;
			size_t j = it_v->second;

			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				// 正向邊：提高 v 變正的機率
				double need = std::max(0.0, 1.0 - pos_coverage[j]);
				if (need > 0.0) {
					double add = w / pos_th[j];
					double eff = std::min(need, add);
					score += LAMBDA * eff;
				}
			}
			else if (w < -EPS) {
				// 負向邊：增加 v 變負的風險
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

	// ------------------------------------------------------------
	// Lambda: 把 u 當作種子，更新其 1-hop 鄰居的覆蓋率
	// ------------------------------------------------------------
	auto apply_influence = [&](int u) {
		unordered_map<int, size_t>::const_iterator it_u = id2idx.find(u);
		if (it_u == id2idx.end()) return;

		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (size_t t = 0; t < outnei.size(); ++t) {
			int v = outnei[t];
			unordered_map<int, size_t>::const_iterator it_v = id2idx.find(v);
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

	// 先把 givenPosSeed 的影響灌進 coverage（但不算在本函式回傳的種子裡）
	if (id2idx.find(givenPosSeed) != id2idx.end()) {
		apply_influence(givenPosSeed);
	}

	// ------------------------------------------------------------
	// CELF 主流程
	// ------------------------------------------------------------
	std::priority_queue<NodeGain> pq;

	// 第 0 輪：計算所有節點的初始邊際增益
	for (size_t i = 0; i < N; ++i) {
		int u = nodes[i];
		double sc = marginal_gain(u);
		NodeGain ng;
		ng.score = sc;
		ng.node_id = u;
		ng.round = 0;
		pq.push(ng);
	}

	// 逐一選出 K 個種子
	for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {
		int bestNode = -1;

		// Lazy 更新：確保取出的分數是「在第 k 輪重新計算」過的
		while (!pq.empty()) {
			NodeGain top = pq.top();
			pq.pop();

			if (top.round < static_cast<int>(k)) {
				double new_score = marginal_gain(top.node_id);
				NodeGain ng;
				ng.score = new_score;
				ng.node_id = top.node_id;
				ng.round = static_cast<int>(k);
				pq.push(ng);
			}
			else {
				bestNode = top.node_id;
				break;
			}
		}

		if (bestNode == -1) {
			break;
		}

		// 正式選為種子，並更新覆蓋率
		seeds.insert(bestNode);
		apply_influence(bestNode);
	}

	return seeds;
}

#endif // YOUR_ALGORITHM_H
