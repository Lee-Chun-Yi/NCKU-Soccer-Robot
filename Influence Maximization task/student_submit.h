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
#include <queue> // CELF 最佳化需要優先佇列

using namespace std;

/**
 * @brief CELF 演算法所需的輔助結構
 * 用於 priority_queue，儲存節點的潛在邊際增益。
 */
struct NodeGain {
	double score;    // 邊際增益分數
	int node_id;  // 節點 ID
	int round;    // 標記這個分數是在第幾輪 (k) 計算的

	/**
	 * @brief 重載 < 運算子
	 * 使 std::priority_queue 預設成為「最大堆」(Max-Heap)，
	 * 永遠優先處理分數最高的節點。
	 */
	bool operator<(const NodeGain& other) const {
		return score < other.score;
	}
};


/**
 * @brief 影響力最大化主函數 (seedSelection)
 * 實作您設計的「1-hop 啟發式貪婪演算法」，並使用 CELF 最佳化 [1, 2, 3, 4]
 * 來避免 TLE (Time Limit Exceeded)。
 *
 * @param G 評測系統傳入的圖
 * @param numberOfSeeds 要選擇的種子數量 (K)
 * @param givenPosSeed given positive seed provided by the dataset
 * @param givenNegSeeds given negative seeds provided by the dataset
 * @return std::unordered_set<int> 包含 K 個節點 ID 的最佳種子集 (如圖 4 所示)
 */
unordered_set<int> seedSelection(DirectedGraph& G,
	unsigned int numberOfSeeds,
	int givenPosSeed,
	const unordered_set<int>& givenNegSeeds) {
	unordered_set<int> seeds;
	if (numberOfSeeds == 0) return seeds;

	// 取得所有節點
	vector<int> nodes = G.getAllNodes();
	if (nodes.empty()) return seeds;

	// node id -> index (用於快速存取 vector)
	unordered_map<int, size_t> id2idx;
	id2idx.reserve(nodes.size() * 2);
	for (size_t i = 0; i < nodes.size(); ++i) {
		id2idx[nodes[i]] = i;
	}

	const double EPS = 1e-12;
	const size_t N = nodes.size();

	// 預先計算每個節點的：
	//  - 正向門檻 pos_th
	//  - 負向門檻絕對值 neg_th_mag
	//  - 入邊正向影響總和 in_strength
	vector<double> pos_th(N, 0.0);
	vector<double> neg_th_mag(N, 0.0);
	vector<double> in_strength(N, 0.0);

	for (size_t i = 0; i < N; ++i) {
		int u = nodes[i];
		double pth = G.getNodeThreshold(u);  // 正門檻
		double nth = G.getNodeThreshold2(u); // 負門檻（通常為負值）

		pos_th[i] = std::max(EPS, pth);
		neg_th_mag[i] = std::max(EPS, std::fabs(nth));

		double sum_in = 0.0;
		// 只累計正向入邊影響力（代表這個點本來就很容易被啟動）
		vector<int> innei = G.getNodeInNeighbors(u);
		for (int p : innei) {
			double w = G.getEdgeInfluence(p, u);
			if (w > 0.0) sum_in += w;
		}
		in_strength[i] = sum_in;
	}

	// 覆蓋率：
	//  pos_coverage[v] ∈ ：目前已選種子對 v 的正向門檻覆蓋比例
	//  neg_coverage[v] ∈ ：目前已選種子對 v 的負向門檻覆蓋比例
	vector<double> pos_coverage(N, 0.0);
	vector<double> neg_coverage(N, 0.0);

	// (A) 啟發式權重微調
	const double LAMBDA = 1.0;   // 正向覆蓋增益
	const double PHI = 1.0;   // 負向覆蓋懲罰
	const double GAMMA = 0.25;  // 節點自身正門檻懲罰
	const double DELTA = 0.2;   // 入邊正向強度懲罰（避免冗餘種子）
	const double ETA = 0.15;  // 獎勵「不易變負」的強韌性 (從 0.0 調整)

	auto is_forbidden = [&](int node) -> bool {
		if (node == givenPosSeed) return true;
		return givenNegSeeds.find(node) != givenNegSeeds.end();
	};


	/**
	 * @brief [Lambda] 計算節點 u 的邊際增益分數
	 */
	auto marginal_gain = [&](int u) -> double {
		// 已經是種子就不再選
		if (seeds.count(u) || is_forbidden(u)) {
			return -std::numeric_limits<double>::infinity();
		}

		auto it_u = id2idx.find(u);
		if (it_u == id2idx.end()) return -std::numeric_limits<double>::infinity();
		size_t iu = it_u->second;

		// 基本分數：自己的門檻越高 / 入邊越強，分數越低
		double score = 0.0;
		score -= GAMMA * pos_th[iu];
		score -= DELTA * in_strength[iu];
		score += ETA * neg_th_mag[iu]; // (A) 獎勵強韌性

		// 貢獻：正向出邊對鄰居的新增覆蓋
		// 懲罰：負向出邊對鄰居的負向覆蓋
		vector<int> outnei = G.getNodeOutNeighbors(u);
		for (int v : outnei) {
			auto it_v = id2idx.find(v);
			if (it_v == id2idx.end()) continue;
			size_t j = it_v->second;

			double w = G.getEdgeInfluence(u, v);

			if (w > EPS) {
				// 正向邊：增加對 v 的正向覆蓋（以門檻做正規化）
				double need = std::max(0.0, 1.0 - pos_coverage[j]);
				if (need > 0.0) {
					double add = w / pos_th[j];      // 這條邊若單獨作種子可覆蓋的比例
					double eff = std::min(need, add); // 但不能超過尚未覆蓋的部分
					score += LAMBDA * eff;
				}
			}
			else if (w < -EPS) {
				// 負向邊：這個候選種子若被選中，會對 v 增加變負的風險
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

	/**
	 * @brief [Lambda] 選定某個 u 為種子時，更新所有鄰居的正/負覆蓋率
	 */
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


	if (id2idx.find(givenPosSeed) != id2idx.end()) {
		apply_influence(givenPosSeed);
	}
	// --- (B) CELF 最佳化演算法 [1, 2, 3, 5] ---

	// CELF 步驟 1: 初始計算 (k=0)
	// 建立一個最大堆 (Max-Heap) [6, 7]
	std::priority_queue<NodeGain> pq;
	// O(N * avg_degree)，計算所有節點的初始增益
	for (int u : nodes) {
		double sc = marginal_gain(u); // 此時 S 為空
		// C++14 修正：必須明確使用類型建構 NodeGain
		pq.push(NodeGain{ sc, u, 0 }); // 標記為第 0 輪計算
	}

	// CELF 步驟 2: 貪婪選出 K 個種子
	for (unsigned int k = 0; k < numberOfSeeds && !pq.empty(); ++k) {

		int bestNode = -1;

		// CELF 步驟 2a: 懶惰評估 (Lazy Evaluation) [1, 8]
		// 不斷從 pq 取出，直到找到一個「新鮮」的分數
		while (true) {
			// 取出目前分數最高的節點
			NodeGain top = pq.top();
			pq.pop();

			// 檢查分數是否「過期」
			// 如果 top.round < k，代表這是 k-1 或更早的分數，
			// 必須重新計算，因為 coverage 已經改變了。
			if (top.round < k) {
				// 重新計算真實的邊際增益
				double new_score = marginal_gain(top.node_id);

				// C++14 修正：必須明確使用類型建構 NodeGain
				// 將「更新」的分數和「本輪 (k)」的標記放回 pq
				pq.push(NodeGain{ new_score, top.node_id, (int)k });

				// 繼續迴圈，取出下一個最高分
				continue;

			}
			else {
				// top.round == k，這是在本輪 (k) 計算的
				// (或從 k-1 輪繼承且尚未被 re-calc 的最高分)
				// 這是本輪的真正最佳解。
				bestNode = top.node_id;
				break; // 結束 while(true)
			}
		}

		if (bestNode == -1) {
			// 佇列為空或發生錯誤
			break;
		}

		// CELF 步驟 2b: 選定種子並更新狀態
		seeds.insert(bestNode);

		// **關鍵**：更新鄰居的覆蓋率
		// 這會使其他節點的分數在 k+1 輪「過期」
		apply_influence(bestNode);
	}

	return seeds;
}

#endif // YOUR_ALGORITHM_H
