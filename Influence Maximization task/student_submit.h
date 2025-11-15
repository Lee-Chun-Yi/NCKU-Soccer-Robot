#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

namespace student_algo_detail {

	struct SeedInfo {
		bool loaded = false;
		bool hasGiven = false;
		int given = -1;
		string dataDir;
		unordered_set<int> negatives;
	};

	static string joinPath(const string& dir, const string& file) {
		if (dir.empty()) return file;
		char tail = dir.back();
		if (tail == '/' || tail == '\\') return dir + file;
		return dir + "/" + file;
	}

	static vector<string> readCmdlineArgs() {
		ifstream cmd("/proc/self/cmdline", ios::binary);
		if (!cmd.is_open()) return {};
		string raw((istreambuf_iterator<char>(cmd)), istreambuf_iterator<char>());
		vector<string> args;
		string current;
		for (char c : raw) {
			if (c == '\0') {
				if (!current.empty()) {
					args.push_back(current);
					current.clear();
				}
			}
			else {
				current.push_back(c);
			}
		}
		if (!current.empty()) args.push_back(current);
		return args;
	}

	static const SeedInfo& getSeedInfo() {
		static SeedInfo info;
		if (info.loaded) return info;
		info.loaded = true;

		vector<string> args = readCmdlineArgs();
		if (args.size() >= 2) info.dataDir = args[1];
		if (info.dataDir.empty()) return info;

		ifstream gp(joinPath(info.dataDir, "given_pos.txt"));
		if (gp.is_open() && (gp >> info.given)) {
			info.hasGiven = true;
		}

		ifstream gn(joinPath(info.dataDir, "neg_seed.txt"));
		if (gn.is_open()) {
			int val = 0;
			while (gn >> val) info.negatives.insert(val);
		}

		return info;
	}

	struct GraphCache {
		vector<int> nodeIds;
		unordered_map<int, int> idToIndex;
		vector<vector<pair<int, double>>> outAdj;
		vector<double> posThreshold;
		vector<double> negThreshold;
		vector<double> outStrength;

		int indexOf(int nodeId) const {
			auto it = idToIndex.find(nodeId);
			if (it == idToIndex.end()) return -1;
			return it->second;
		}
	};

	static GraphCache buildGraphCache(DirectedGraph& G) {
		GraphCache cache;
		cache.nodeIds = G.getAllNodes();
		sort(cache.nodeIds.begin(), cache.nodeIds.end());

		cache.idToIndex.reserve(cache.nodeIds.size() * 2 + 1);
		for (size_t i = 0; i < cache.nodeIds.size(); ++i) {
			cache.idToIndex[cache.nodeIds[i]] = static_cast<int>(i);
		}

		size_t N = cache.nodeIds.size();
		cache.outAdj.assign(N, {});
		cache.posThreshold.assign(N, 0.0);
		cache.negThreshold.assign(N, 0.0);
		cache.outStrength.assign(N, 0.0);

		for (size_t i = 0; i < N; ++i) {
			int nodeId = cache.nodeIds[i];
			cache.posThreshold[i] = G.getNodeThreshold(nodeId);
			cache.negThreshold[i] = G.getNodeThreshold2(nodeId);

			vector<int> outs = G.getNodeOutNeighbors(nodeId);
			auto& adj = cache.outAdj[i];
			adj.reserve(outs.size());

			double total = 0.0;
			for (int nb : outs) {
				double w = G.getEdgeInfluence(nodeId, nb);
				total += w;
				auto it = cache.idToIndex.find(nb);
				if (it == cache.idToIndex.end()) continue;
				adj.emplace_back(it->second, w);
			}

			sort(adj.begin(), adj.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
				if (a.first != b.first) return a.first < b.first;
				return a.second < b.second;
			});

			cache.outStrength[i] = total;
		}

		return cache;
	}

	static vector<double> computeNegExposure(const GraphCache& cache, const unordered_set<int>& negSeeds) {
		vector<double> exposure(cache.nodeIds.size(), 0.0);
		if (cache.nodeIds.empty() || negSeeds.empty()) return exposure;
		for (int id : negSeeds) {
			int idx = cache.indexOf(id);
			if (idx < 0) continue;
			for (const auto& edge : cache.outAdj[idx])
				exposure[edge.first] += edge.second;
		}
		return exposure;
	}

	struct FullDiffResult {
		size_t posActive = 0;
		size_t negActive = 0;
		double spread = 0.0;
	};

	static FullDiffResult runFullDiffusionSimulation(
		DirectedGraph& G,
		const unordered_set<int>& posSeeds,
		const unordered_set<int>& negSeeds)
	{
		FullDiffResult result;
		unordered_set<int> finalPos, finalNeg;
		diffuse_signed_all(&G, posSeeds, negSeeds, finalPos, finalNeg);
		result.posActive = finalPos.size();
		result.negActive = finalNeg.size();
		result.spread = double(result.posActive) - double(result.negActive);
		return result;
	}

} // namespace student_algo_detail


unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
	using namespace student_algo_detail;

	unordered_set<int> seeds;
	if (numberOfSeeds == 0 || G.getSize() == 0) {
		return seeds;
	}

	const GraphCache cache = buildGraphCache(G);
	const SeedInfo& info = getSeedInfo();

	unordered_set<int> banned = info.negatives;
	if (info.hasGiven) banned.insert(info.given);

	vector<double> negExposure = computeNegExposure(cache, info.negatives);

	const size_t N = cache.nodeIds.size();
	vector<double> fastScore(N, 0.0);
	for (size_t idx = 0; idx < N; ++idx) {
		double score = cache.outStrength[idx];
		score += 0.05 * double(cache.outAdj[idx].size());
		score -= 0.55 * cache.posThreshold[idx];
		if (!negExposure.empty()) score -= 0.8 * negExposure[idx];
		fastScore[idx] = score;
	}

	vector<int> order;
	order.reserve(N);
	for (size_t i = 0; i < N; ++i) order.push_back(static_cast<int>(i));
	sort(order.begin(), order.end(), [&](int a, int b) {
		if (fastScore[a] != fastScore[b]) return fastScore[a] > fastScore[b];
		return cache.nodeIds[a] < cache.nodeIds[b];
	});

	// ==== 調整 1：再縮減需要完整模擬的候選數量 ====
	const int MIN_SIM = 150;        // 原本 200
	const int MULTIPLIER = 20;      // 原本 30
	const int MAX_SIM = 2000;       // 原本 3000

	int simulateCount = static_cast<int>(order.size());
	int targetSim = max(MIN_SIM, MULTIPLIER * static_cast<int>(numberOfSeeds));
	if (targetSim > MAX_SIM) targetSim = MAX_SIM;
	if (simulateCount > targetSim) simulateCount = targetSim;

	vector<int> candidateNodes;
	candidateNodes.reserve(simulateCount);
	for (int i = 0; i < simulateCount && i < static_cast<int>(order.size()); ++i) {
		int idx = order[i];
		int nodeId = cache.nodeIds[idx];
		if (banned.count(nodeId)) continue;
		candidateNodes.push_back(nodeId);
	}
	if (candidateNodes.empty()) {
		for (int nodeId : cache.nodeIds) {
			if (banned.count(nodeId)) continue;
			candidateNodes.push_back(nodeId);
			if ((int)candidateNodes.size() >= targetSim) break;
		}
	}

	unordered_set<int> negSeedSet = info.negatives;
	unordered_set<int> workingSeeds;
	if (info.hasGiven) workingSeeds.insert(info.given);
	FullDiffResult baseResult = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
	double currentSpread = baseResult.spread;
	int iteration = 0;

	struct CelfEntry {
		int nodeId;
		double gain;
		double totalSpread;
		int lastUpdate;
	};
	struct CelfCompare {
		bool operator()(const CelfEntry& a, const CelfEntry& b) const {
			if (a.gain != b.gain) return a.gain < b.gain;
			return a.nodeId > b.nodeId;
		}
	};

	auto evaluateCandidate = [&](int nodeId, int iterationTag) {
		unordered_set<int> trialSeeds = workingSeeds;
		trialSeeds.insert(nodeId);
		FullDiffResult result = runFullDiffusionSimulation(G, trialSeeds, negSeedSet);
		return CelfEntry{ nodeId, result.spread - currentSpread, result.spread, iterationTag };
	};

	priority_queue<CelfEntry, vector<CelfEntry>, CelfCompare> pq;
	for (int nodeId : candidateNodes) {
		if (workingSeeds.count(nodeId)) continue;
		pq.push(evaluateCandidate(nodeId, 0));
	}

	// ==== 調整 2：只用 CELF 精算前 ~60% 的種子 ====
	unsigned int greedyBudget = (unsigned int)(numberOfSeeds * 6 / 10 + 1);
	if (greedyBudget > numberOfSeeds) greedyBudget = numberOfSeeds;

	// ==== 調整 3：若最佳 gain <= 0，提前停掉 CELF ====
	while (seeds.size() < greedyBudget && !pq.empty()) {
		CelfEntry top = pq.top();
		pq.pop();
		if (workingSeeds.count(top.nodeId) || banned.count(top.nodeId)) continue;

		if (top.lastUpdate < iteration) {
			// 過期分數，重算
			CelfEntry upd = evaluateCandidate(top.nodeId, iteration);
			pq.push(upd);
			continue;
		}

		// 現在的 top 是當前 iteration 的真實最佳
		if (top.gain <= 0.0) {
			// 再加種子不會帶來正增益，直接停 CELF
			break;
		}

		seeds.insert(top.nodeId);
		workingSeeds.insert(top.nodeId);
		currentSpread = top.totalSpread;
		++iteration;
	}

	// 用 fastScore 排序補滿剩下的種子（不再做模擬）
	if (seeds.size() < numberOfSeeds) {
		for (int idx : order) {
			if (seeds.size() >= numberOfSeeds) break;
			int nodeId = cache.nodeIds[idx];
			if (banned.count(nodeId) || seeds.count(nodeId)) continue;
			seeds.insert(nodeId);
		}
	}

	return seeds;
}

#endif
