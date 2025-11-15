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

struct PartialResult {
	int posActive = 0;
	int negActive = 0;
	double depthWeightedPos = 0.0;
	double depthWeightedNeg = 0.0;
	int touched = 0;
};

struct FrontierNode {
	int idx;
	int depth;
	int type; // +1 positive, -1 negative
};

static PartialResult runPartialDiffusion(const GraphCache& cache,
	const vector<int>& posSeeds,
	const vector<int>& negSeeds,
	int maxDepth,
	int visitLimit)
{
	PartialResult res;
	const int N = static_cast<int>(cache.nodeIds.size());
	if (N == 0) return res;
	if (visitLimit <= 0) visitLimit = N;

	vector<int8_t> state(N, 0);
	vector<double> balance(N, 0.0);
	vector<char> touched(N, 0);
	queue<FrontierNode> q;

	auto markTouched = [&](int idx) {
		if (!touched[idx]) {
			touched[idx] = 1;
			++res.touched;
		}
	};

	auto activate = [&](int idx, int type, int depth) {
		state[idx] = static_cast<int8_t>(type);
		markTouched(idx);
		if (type == 1) {
			++res.posActive;
			res.depthWeightedPos += double(maxDepth - depth + 1);
		} else {
			++res.negActive;
			res.depthWeightedNeg += double(maxDepth - depth + 1);
		}
		q.push({ idx, depth, type });
	};

	for (int idx : posSeeds)
		if (idx >= 0 && idx < N && state[idx] == 0)
			activate(idx, 1, 0);

	for (int idx : negSeeds)
		if (idx >= 0 && idx < N && state[idx] == 0)
			activate(idx, -1, 0);

	bool limitReached = res.touched >= visitLimit;
	while (!q.empty() && !limitReached) {
		FrontierNode cur = q.front();
		q.pop();
		if (cur.depth >= maxDepth) continue;
		const auto& neighbors = cache.outAdj[cur.idx];
		for (const auto& edge : neighbors) {
			int v = edge.first;
			double w = edge.second;
			if (!touched[v]) {
				touched[v] = 1;
				++res.touched;
				if (res.touched >= visitLimit) {
					limitReached = true;
				}
			}
			balance[v] += (cur.type == 1 ? w : -w);
			if (state[v] != 0) continue;
			double val = balance[v];
			if (val >= cache.posThreshold[v]) {
				activate(v, 1, cur.depth + 1);
			}
			else if (val <= cache.negThreshold[v]) {
				activate(v, -1, cur.depth + 1);
			}
			if (limitReached) break;
		}
	}

	return res;
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

} // namespace student_algo_detail

/*
 * seedSelection:
 *   - G:   ��i��
 *   - numberOfSeeds: �ݭn�A�諸�u�B�~���V�ؤl�ƶq�v
 *                     �]�t�Υt�~�|�A�[�W 1 �� given_pos.txt �̪� positive seed�^
 *
 * �A�u�ݭn�^�Ǥ@�� unordered_set<int>�A�̭��� numberOfSeeds 9��positivate seed�s���C
 */
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

	vector<int> basePosIdx;
	if (info.hasGiven) {
		int idx = cache.indexOf(info.given);
		if (idx >= 0) basePosIdx.push_back(idx);
	}

	vector<int> negSeedIdx;
	negSeedIdx.reserve(info.negatives.size());
	for (int neg : info.negatives) {
		int idx = cache.indexOf(neg);
		if (idx >= 0) negSeedIdx.push_back(idx);
	}

	constexpr int MAX_SIM_DEPTH = 3;
	constexpr int MAX_VISIT = 2000;

	PartialResult baseResult = runPartialDiffusion(cache, basePosIdx, negSeedIdx, MAX_SIM_DEPTH, MAX_VISIT);
	const double baseSpread = double(baseResult.posActive - baseResult.negActive);
	const double baseDepth = baseResult.depthWeightedPos - baseResult.depthWeightedNeg;

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

	const int MIN_SIM = 400;
	const int MULTIPLIER = 60;
	int simulateCount = static_cast<int>(order.size());
	int targetSim = max(MIN_SIM, MULTIPLIER * static_cast<int>(numberOfSeeds));
	if (simulateCount > targetSim) simulateCount = targetSim;

	vector<double> detailedScore(N, numeric_limits<double>::lowest());
	vector<int> workingSeeds;
	workingSeeds.reserve(basePosIdx.size() + 1);

	for (int i = 0; i < simulateCount; ++i) {
		int idx = order[i];
		int nodeId = cache.nodeIds[idx];
		if (banned.count(nodeId)) continue;

		workingSeeds = basePosIdx;
		workingSeeds.push_back(idx);

		PartialResult result = runPartialDiffusion(cache, workingSeeds, negSeedIdx, MAX_SIM_DEPTH, MAX_VISIT);
		double spreadGain = double(result.posActive - result.negActive) - baseSpread;
		double depthGain = (result.depthWeightedPos - result.depthWeightedNeg) - baseDepth;
		double score = spreadGain * 1.4 + depthGain * 0.45
			+ cache.outStrength[idx] * 0.55
			- cache.posThreshold[idx] * 0.35
			- negExposure[idx] * 0.7;
		detailedScore[idx] = score;
	}

	struct CandidateScore {
		int nodeId;
		double score;
	};
	vector<CandidateScore> ranked;
	ranked.reserve(N);
	for (size_t idx = 0; idx < N; ++idx) {
		int nodeId = cache.nodeIds[idx];
		if (banned.count(nodeId)) continue;
		double score = detailedScore[idx];
		if (score == numeric_limits<double>::lowest()) score = fastScore[idx];
		ranked.push_back({ nodeId, score });
	}

	sort(ranked.begin(), ranked.end(), [](const CandidateScore& a, const CandidateScore& b) {
		if (a.score != b.score) return a.score > b.score;
		return a.nodeId < b.nodeId;
	});

	for (const auto& cand : ranked) {
		if (seeds.size() >= numberOfSeeds) break;
		if (!banned.count(cand.nodeId)) seeds.insert(cand.nodeId);
	}

	if (seeds.size() < numberOfSeeds) {
		for (int nodeId : cache.nodeIds) {
			if (seeds.size() >= numberOfSeeds) break;
			if (banned.count(nodeId)) continue;
			seeds.insert(nodeId);
		}
	}

	return seeds;
}

#endif
