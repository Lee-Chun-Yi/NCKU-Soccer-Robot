#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <chrono>
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

        const auto startTime = chrono::steady_clock::now();
        const chrono::milliseconds softTimeBudget(570000); // 9.5 minutes

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

        const int MIN_SIM = 400;
        const int MULTIPLIER = 40;
        const int MAX_SIM = 6000;
        int simulateCount = static_cast<int>(order.size());
        long long targetSim = static_cast<long long>(numberOfSeeds) * MULTIPLIER;
        if (targetSim < MIN_SIM) targetSim = MIN_SIM;
        if (targetSim > MAX_SIM) targetSim = MAX_SIM;
        if (simulateCount > targetSim) simulateCount = static_cast<int>(targetSim);

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
			if (static_cast<int>(candidateNodes.size()) >= targetSim) break;
		}
	}

        unordered_set<int> negSeedSet = info.negatives;
        unordered_set<int> workingSeeds;
        if (info.hasGiven) workingSeeds.insert(info.given);

        auto timeExceeded = [&]() {
                return chrono::steady_clock::now() - startTime > softTimeBudget;
        };

        const size_t MIN_DIFF_RUNS = 800;
        const size_t PER_SEED_RUNS = 30;
        const size_t MAX_DIFF_RUNS = 6000;
        size_t diffRunLimit = max(MIN_DIFF_RUNS, PER_SEED_RUNS * static_cast<size_t>(numberOfSeeds));
        if (diffRunLimit > MAX_DIFF_RUNS) diffRunLimit = MAX_DIFF_RUNS;
        size_t diffRuns = 0;

        FullDiffResult baseResult = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
        ++diffRuns;
        double currentSpread = baseResult.spread;
        int iteration = 0;

        struct CandidateEntry {
                int nodeId;
                double key;
                double totalSpread;
                int lastUpdate;
                bool evaluated;
        };
        struct CandidateCompare {
                bool operator()(const CandidateEntry& a, const CandidateEntry& b) const {
                        if (a.key != b.key) return a.key < b.key;
                        return a.nodeId > b.nodeId;
                }
        };

        auto evaluateCandidate = [&](int nodeId, int iterationTag, CandidateEntry& out) {
                if (diffRuns >= diffRunLimit || timeExceeded()) return false;
                unordered_set<int> trialSeeds = workingSeeds;
                trialSeeds.insert(nodeId);
                FullDiffResult result = runFullDiffusionSimulation(G, trialSeeds, negSeedSet);
                ++diffRuns;
                out.nodeId = nodeId;
                out.key = result.spread - currentSpread;
                out.totalSpread = result.spread;
                out.lastUpdate = iterationTag;
                out.evaluated = true;
                return true;
        };

        priority_queue<CandidateEntry, vector<CandidateEntry>, CandidateCompare> pq;
        for (int nodeId : candidateNodes) {
                if (workingSeeds.count(nodeId)) continue;
                double heuristic = 0.0;
                int idx = cache.indexOf(nodeId);
                if (idx >= 0) heuristic = fastScore[idx];
                pq.push(CandidateEntry{ nodeId, heuristic, 0.0, -1, false });
        }

        bool budgetExhausted = false;
        while (seeds.size() < numberOfSeeds && !pq.empty()) {
                CandidateEntry top = pq.top();
                pq.pop();
                if (workingSeeds.count(top.nodeId) || banned.count(top.nodeId)) continue;
                if (!top.evaluated) {
                        CandidateEntry evaluated = top;
                        if (!evaluateCandidate(top.nodeId, iteration, evaluated)) {
                                budgetExhausted = true;
                                break;
                        }
                        pq.push(evaluated);
                        continue;
                }
                if (top.lastUpdate == iteration) {
                        seeds.insert(top.nodeId);
                        workingSeeds.insert(top.nodeId);
                        currentSpread = top.totalSpread;
                        ++iteration;
                }
                else {
                        CandidateEntry refreshed;
                        if (!evaluateCandidate(top.nodeId, iteration, refreshed)) {
                                budgetExhausted = true;
                                break;
                        }
                        pq.push(refreshed);
                }
        }

        if (budgetExhausted || timeExceeded()) {
                while (!pq.empty()) pq.pop();
        }

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
