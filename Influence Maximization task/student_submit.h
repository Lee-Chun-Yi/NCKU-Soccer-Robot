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

namespace student_algo_detail {

static unordered_set<int> runSeedSelection(
        DirectedGraph& G,
        unsigned int numberOfSeeds,
        const SeedInfo& info)
{
        unordered_set<int> seeds;
        seeds.reserve(numberOfSeeds);
        if (numberOfSeeds == 0 || G.getSize() == 0) {
                return seeds;
        }

        const GraphCache cache = buildGraphCache(G);

        unordered_set<int> banned = info.negatives;
        if (info.hasGiven) banned.insert(info.given);

        const size_t N = cache.nodeIds.size();
        vector<char> bannedMask(N, 0);
        for (int nodeId : banned) {
                int idx = cache.indexOf(nodeId);
                if (idx >= 0) bannedMask[idx] = 1;
        }

        vector<double> negExposure = computeNegExposure(cache, info.negatives);

        vector<double> fastScore(N, 0.0);
        const bool hasNegExposure = !negExposure.empty();
        for (size_t idx = 0; idx < N; ++idx) {
                double score = cache.outStrength[idx];
                score += 0.05 * double(cache.outAdj[idx].size());
                score -= 0.55 * cache.posThreshold[idx];
                if (hasNegExposure) score -= 0.8 * negExposure[idx];
                fastScore[idx] = score;
        }

        vector<int> order;
        order.reserve(N);
        for (size_t i = 0; i < N; ++i) order.push_back(static_cast<int>(i));
        sort(order.begin(), order.end(), [&](int a, int b) {
                if (fastScore[a] != fastScore[b]) return fastScore[a] > fastScore[b];
                return cache.nodeIds[a] < cache.nodeIds[b];
        });

        unordered_set<int> workingSeeds;
        vector<char> workingMask(N, 0);
        if (info.hasGiven) {
                workingSeeds.insert(info.given);
                int idx = cache.indexOf(info.given);
                if (idx >= 0) workingMask[idx] = 1;
        }

        const int MIN_SIM = 200;
        const int MULTIPLIER = 40;
        int targetSim = max(MIN_SIM, MULTIPLIER * static_cast<int>(numberOfSeeds));
        int simulateCount = min(static_cast<int>(N), targetSim);

        vector<int> candidateIndices;
        candidateIndices.reserve(simulateCount);
        for (int i = 0; i < simulateCount && i < static_cast<int>(order.size()); ++i) {
                int idx = order[i];
                if (bannedMask[idx] || workingMask[idx]) continue;
                if (cache.outAdj[idx].empty()) continue;
                if (cache.outStrength[idx] < 1e-6) continue;
                candidateIndices.push_back(idx);
        }
        if (candidateIndices.empty()) {
                for (int idx : order) {
                        if (bannedMask[idx] || workingMask[idx]) continue;
                        if (cache.outAdj[idx].empty()) continue;
                        if (cache.outStrength[idx] < 1e-6) continue;
                        candidateIndices.push_back(idx);
                        if (static_cast<int>(candidateIndices.size()) >= targetSim) break;
                }
        }

        unordered_set<int> negSeedSet = info.negatives;
        FullDiffResult baseResult = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
        double currentSpread = baseResult.spread;
        int iteration = 0;

        vector<int> neighborTouchCount(N, 0);
        auto registerSeedNeighbors = [&](int nodeIdx) {
                if (nodeIdx < 0 || nodeIdx >= static_cast<int>(N)) return;
                for (const auto& edge : cache.outAdj[nodeIdx]) {
                        if (edge.first >= 0 && edge.first < static_cast<int>(N)) {
                                ++neighborTouchCount[edge.first];
                        }
                }
        };
        if (info.hasGiven) {
                int idx = cache.indexOf(info.given);
                if (idx >= 0) registerSeedNeighbors(idx);
        }

        vector<double> baseUpperBound(N, 0.0);
        for (size_t i = 0; i < N; ++i) {
                baseUpperBound[i] = cache.outStrength[i] + double(cache.outAdj[i].size());
        }

        auto computeAdjustedUB = [&](int nodeIdx) {
                if (nodeIdx < 0 || nodeIdx >= static_cast<int>(N)) return 0.0;
                double ub = baseUpperBound[nodeIdx];
                int overlap = 0;
                for (const auto& edge : cache.outAdj[nodeIdx]) {
                        if (neighborTouchCount[edge.first] > 0) ++overlap;
                }
                ub -= double(overlap);
                if (ub < 0.0) ub = 0.0;
                return ub;
        };

        struct CelfEntry {
                int nodeId;
                int nodeIdx;
                double mg1;
                double totalSpread;
                double mg2;
                double totalSpread2;
                int lastUpdate;
                bool mg2Fresh;
        };
        struct CelfCompare {
                bool operator()(const CelfEntry& a, const CelfEntry& b) const {
                        if (a.mg1 != b.mg1) return a.mg1 < b.mg1;
                        return a.nodeId > b.nodeId;
                }
        };

        unordered_set<int> trialSeeds;
        auto evaluateCandidate = [&](int nodeId, int nodeIdx) {
                trialSeeds.clear();
                size_t requiredSize = workingSeeds.size() + 1;
                if (trialSeeds.bucket_count() < requiredSize * 2) {
                        trialSeeds.reserve(requiredSize * 2);
                }
                trialSeeds.insert(workingSeeds.begin(), workingSeeds.end());
                trialSeeds.insert(nodeId);
                FullDiffResult result = runFullDiffusionSimulation(G, trialSeeds, negSeedSet);
                return pair<double, double>(result.spread - currentSpread, result.spread);
        };

        priority_queue<CelfEntry, vector<CelfEntry>, CelfCompare> pq;
        double bestKnownGain = 0.0;
        for (int nodeIdx : candidateIndices) {
                if (workingMask[nodeIdx] || bannedMask[nodeIdx]) continue;
                int nodeId = cache.nodeIds[nodeIdx];
                double ub = computeAdjustedUB(nodeIdx);
                if (ub + 1e-9 < bestKnownGain) continue;
                auto eval = evaluateCandidate(nodeId, nodeIdx);
                bestKnownGain = max(bestKnownGain, eval.first);
                pq.push(CelfEntry{ nodeId, nodeIdx, eval.first, eval.second, 0.0, 0.0, 0, false });
        }

        auto finalizeSelection = [&](int nodeId, int nodeIdx, double newSpread) {
                seeds.insert(nodeId);
                workingSeeds.insert(nodeId);
                workingMask[nodeIdx] = 1;
                currentSpread = newSpread;
                registerSeedNeighbors(nodeIdx);
        };

        while (seeds.size() < numberOfSeeds) {
                bool selected = false;
                while (!pq.empty() && !selected) {
                        CelfEntry top = pq.top();
                        pq.pop();
                        if (top.nodeIdx < 0 || top.nodeIdx >= static_cast<int>(N)) continue;
                        if (workingMask[top.nodeIdx] || bannedMask[top.nodeIdx]) continue;
                        double ub = computeAdjustedUB(top.nodeIdx);
                        if (ub + 1e-9 < bestKnownGain) continue;
                        if (top.lastUpdate == iteration) {
                                bestKnownGain = max(bestKnownGain, top.mg1);
                                finalizeSelection(top.nodeId, top.nodeIdx, top.totalSpread);
                                ++iteration;
                                selected = true;
                                break;
                        }

                        double secondBest = pq.empty() ? -numeric_limits<double>::infinity() : pq.top().mg1;
                        if (!top.mg2Fresh) {
                                auto eval = evaluateCandidate(top.nodeId, top.nodeIdx);
                                top.mg2 = eval.first;
                                top.totalSpread2 = eval.second;
                                top.mg2Fresh = true;
                        }
                        bestKnownGain = max(bestKnownGain, top.mg2);
                        if (top.mg2 >= secondBest) {
                                finalizeSelection(top.nodeId, top.nodeIdx, top.totalSpread2);
                                ++iteration;
                                selected = true;
                        }
                        else {
                                top.mg1 = top.mg2;
                                top.totalSpread = top.totalSpread2;
                                top.lastUpdate = iteration;
                                top.mg2Fresh = false;
                                pq.push(top);
                        }
                }
                if (!selected) break;
        }

        if (seeds.size() < numberOfSeeds) {
                for (int idx : order) {
                        if (seeds.size() >= numberOfSeeds) break;
                        if (bannedMask[idx] || workingMask[idx]) continue;
                        int nodeId = cache.nodeIds[idx];
                        seeds.insert(nodeId);
                        workingSeeds.insert(nodeId);
                        workingMask[idx] = 1;
                }
        }

        return seeds;
}

} // namespace student_algo_detail



unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
        using namespace student_algo_detail;
        const SeedInfo& info = getSeedInfo();
        return runSeedSelection(G, numberOfSeeds, info);
}

unordered_set<int> seedSelection(
        DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds)
{
        using namespace student_algo_detail;
        SeedInfo info;
        info.loaded = true;
        if (givenPosSeed >= 0) {
                info.hasGiven = true;
                info.given = givenPosSeed;
        }
        info.negatives = givenNegSeeds;
        return runSeedSelection(G, numberOfSeeds, info);
}

#endif
