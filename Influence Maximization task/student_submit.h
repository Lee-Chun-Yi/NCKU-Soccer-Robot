#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <cmath>
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
        vector<double> inStrength;

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
        cache.inStrength.assign(N, 0.0);

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
                        cache.inStrength[it->second] += w;
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

static vector<double> computeFastScores(const GraphCache& cache, const vector<double>& negExposure) {
        const size_t N = cache.nodeIds.size();
        vector<double> score(N, 0.0);
        for (size_t idx = 0; idx < N; ++idx) {
                        double outStrength = cache.outStrength[idx];
                        double inStrength = cache.inStrength[idx];
                        double outDegree = static_cast<double>(cache.outAdj[idx].size());
                        double exposure = negExposure.empty() ? 0.0 : negExposure[idx];
                        double safeDistance = 1.0 / (1.0 + exposure);
                        double posTh = cache.posThreshold[idx];
                        double negThAbs = std::fabs(cache.negThreshold[idx]);
                        score[idx] = outStrength + 0.3 * inStrength + 0.05 * outDegree +
                                0.4 * safeDistance - 0.55 * posTh - 1.2 * negThAbs - 0.8 * exposure;
        }
        return score;
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

        const GraphCache cache = buildGraphCache(G);
        const SeedInfo& info = getSeedInfo();

        unordered_set<int> banned = info.negatives;
        if (info.hasGiven) banned.insert(info.given);

        // Lightweight signals for every node. These are re-used across the adaptive branches
        // so the pre-processing stays O(N + E).
        vector<double> negExposure = computeNegExposure(cache, info.negatives);
        vector<double> fastScore = computeFastScores(cache, negExposure);

        const size_t N = cache.nodeIds.size();
        vector<int> order;
        order.reserve(N);
        for (size_t i = 0; i < N; ++i) order.push_back(static_cast<int>(i));
        sort(order.begin(), order.end(), [&](int a, int b) {
                if (fastScore[a] != fastScore[b]) return fastScore[a] > fastScore[b];
                return cache.nodeIds[a] < cache.nodeIds[b];
        });

        const int nodeCount = G.getSize();
        const int edgeCount = G.getEdgeNumber();
        // Adaptive policy: brute-force small graphs, classic CELF for medium, aggressive pruning on large.
        const bool isSmall = nodeCount <= 500;
        const bool isLarge = (nodeCount > 5000) || (edgeCount > 200000);
        const bool isMedium = !isSmall && !isLarge;

        size_t candidateLimit = order.size();
        if (isMedium) {
                size_t mediumTarget = max<size_t>(400, 40ull * numberOfSeeds);
                candidateLimit = min(order.size(), mediumTarget);
        }
        else if (isLarge) {
                size_t largeTarget = max<size_t>(200, 20ull * numberOfSeeds);
                candidateLimit = min(order.size(), largeTarget);
        }

        // Nodes dominated by negative exposure rarely become active, so skip them (medium/large only).
        const double exposureThreshold = isSmall ? 5.0 : (isLarge ? 0.8 : 1.5);

        vector<int> candidateNodes;
        candidateNodes.reserve(candidateLimit);
        for (int idx : order) {
                int nodeId = cache.nodeIds[idx];
                if (banned.count(nodeId)) continue;
                if (!isSmall && !negExposure.empty() && negExposure[idx] > exposureThreshold) continue;
                candidateNodes.push_back(nodeId);
                if (!isSmall && candidateNodes.size() >= candidateLimit) break;
        }
        if (candidateNodes.empty()) {
                for (int nodeId : cache.nodeIds) {
                        if (banned.count(nodeId)) continue;
                        candidateNodes.push_back(nodeId);
                        if (!isSmall && candidateNodes.size() >= candidateLimit) break;
                }
        }

        unordered_set<int> negSeedSet = info.negatives;
        unordered_set<int> workingSeeds;
        if (info.hasGiven) workingSeeds.insert(info.given);
        // Keep the CELF base spread for marginal-gain measurements.
        FullDiffResult baseResult = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
        double currentSpread = baseResult.spread;
        int iteration = 0;

        struct CelfEntry {
                int nodeId;
                double gain;
                double totalSpread;
                int lastUpdate;
                bool exact;
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
                return CelfEntry{ nodeId, result.spread - currentSpread, result.spread, iterationTag, true };
        };

        auto makeApproxEntry = [&](int nodeId) {
                int idx = cache.indexOf(nodeId);
                double heuristic = (idx >= 0 ? fastScore[idx] : 0.0);
                return CelfEntry{ nodeId, heuristic, currentSpread + heuristic, -1, false };
        };

        priority_queue<CelfEntry, vector<CelfEntry>, CelfCompare> pq;
        size_t initialEvalLimit = candidateNodes.size();
        if (isLarge) {
                size_t minEval = max<size_t>(50, 5ull * numberOfSeeds);
                initialEvalLimit = min(candidateNodes.size(), minEval);
        }
        for (size_t i = 0; i < candidateNodes.size(); ++i) {
                int nodeId = candidateNodes[i];
                if (workingSeeds.count(nodeId)) continue;
                // For large graphs, defer expensive simulations until a candidate floats to the top.
                if (isLarge && i >= initialEvalLimit) {
                        pq.push(makeApproxEntry(nodeId));
                }
                else {
                        pq.push(evaluateCandidate(nodeId, 0));
                }
        }

        while (seeds.size() < numberOfSeeds && !pq.empty()) {
                CelfEntry top = pq.top();
                pq.pop();
                if (workingSeeds.count(top.nodeId) || banned.count(top.nodeId)) continue;

                // Standard CELF lazy update: only recompute when the top entry is stale.
                if (!top.exact || top.lastUpdate != iteration) {
                        CelfEntry refreshed = evaluateCandidate(top.nodeId, iteration);
                        pq.push(refreshed);
                        continue;
                }

                seeds.insert(top.nodeId);
                workingSeeds.insert(top.nodeId);
                currentSpread = top.totalSpread;
                ++iteration;
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
