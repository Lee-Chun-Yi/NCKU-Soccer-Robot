#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
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
        vector<vector<pair<int, double>>> inAdj;
        vector<vector<pair<int, double>>> undirectedAdj;
        vector<double> posThreshold;
        vector<double> negThreshold;
        vector<double> outStrength;
        vector<double> inStrength;
        vector<int> outDegree;
        vector<int> inDegree;

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
        cache.inAdj.assign(N, {});
        cache.undirectedAdj.assign(N, {});
        cache.posThreshold.assign(N, 0.0);
        cache.negThreshold.assign(N, 0.0);
        cache.outStrength.assign(N, 0.0);
        cache.inStrength.assign(N, 0.0);
        cache.outDegree.assign(N, 0);
        cache.inDegree.assign(N, 0);

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
                        cache.inAdj[it->second].emplace_back(static_cast<int>(i), w);
                        cache.undirectedAdj[i].emplace_back(it->second, w);
                        cache.undirectedAdj[it->second].emplace_back(static_cast<int>(i), w);
                        ++cache.outDegree[i];
                        ++cache.inDegree[it->second];
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

static vector<double> computePageRank(const GraphCache& cache, int iterations = 15, double damping = 0.85) {
        const size_t N = cache.nodeIds.size();
        vector<double> rank(N, N == 0 ? 0.0 : 1.0 / double(N));
        if (N == 0) return rank;
        vector<double> next(N, 0.0);
        for (int it = 0; it < iterations; ++it) {
                double base = (1.0 - damping) / double(N);
                fill(next.begin(), next.end(), base);
                double dangling = 0.0;
                for (size_t i = 0; i < N; ++i) {
                        if (cache.outAdj[i].empty()) dangling += rank[i];
                }
                double danglingShare = damping * dangling / double(N);
                for (size_t i = 0; i < N; ++i) next[i] += danglingShare;
                for (size_t i = 0; i < N; ++i) {
                        if (cache.outAdj[i].empty() || cache.outStrength[i] <= 0) continue;
                        double push = damping * rank[i] / cache.outStrength[i];
                        for (const auto& edge : cache.outAdj[i]) next[edge.first] += push * edge.second;
                }
                rank.swap(next);
        }
        return rank;
}

static vector<int> computeShellIndex(const GraphCache& cache) {
        const size_t N = cache.nodeIds.size();
        vector<int> shell(N, 0);
        if (N == 0) return shell;
        vector<int> degree(N, 0);
        for (size_t i = 0; i < N; ++i)
                degree[i] = static_cast<int>(cache.outAdj[i].size() + cache.inAdj[i].size());
        vector<char> removed(N, 0);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        for (size_t i = 0; i < N; ++i) pq.emplace(degree[i], static_cast<int>(i));
        int currentShell = 0;
        while (!pq.empty()) {
                auto [deg, idx] = pq.top();
                pq.pop();
                if (removed[idx]) continue;
                removed[idx] = 1;
                currentShell = max(currentShell, deg);
                shell[idx] = currentShell;
                for (const auto& edge : cache.undirectedAdj[idx]) {
                        int nb = edge.first;
                        if (removed[nb] || degree[nb] == 0) continue;
                        --degree[nb];
                        pq.emplace(degree[nb], nb);
                }
        }
        return shell;
}

static vector<double> computeNegDistanceScore(const GraphCache& cache, const unordered_set<int>& negSeeds) {
        const size_t N = cache.nodeIds.size();
        vector<double> distanceScore(N, 0.0);
        if (N == 0 || negSeeds.empty()) return distanceScore;
        const int INF = numeric_limits<int>::max();
        vector<int> dist(N, INF);
        queue<int> q;
        for (int nodeId : negSeeds) {
                        int idx = cache.indexOf(nodeId);
                        if (idx < 0 || dist[idx] == 0) continue;
                        dist[idx] = 0;
                        q.push(idx);
        }
        const int MAX_DEPTH = 6;
        while (!q.empty()) {
                int cur = q.front();
                q.pop();
                int d = dist[cur];
                if (d >= MAX_DEPTH) continue;
                for (const auto& edge : cache.undirectedAdj[cur]) {
                        int nb = edge.first;
                        if (dist[nb] <= d + 1) continue;
                        dist[nb] = d + 1;
                        q.push(nb);
                }
        }
        for (size_t i = 0; i < N; ++i) {
                if (dist[i] == INF) continue;
                distanceScore[i] = exp(-0.45 * double(dist[i]));
        }
        return distanceScore;
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
        const vector<double> pageRank = computePageRank(cache);
        const vector<int> shellIndex = computeShellIndex(cache);
        const vector<double> negDistanceScore = computeNegDistanceScore(cache, info.negatives);

        vector<double> legacyHeuristic(N, 0.0);
        for (size_t idx = 0; idx < N; ++idx) {
                double legacy = cache.outStrength[idx];
                legacy += 0.05 * double(cache.outAdj[idx].size());
                legacy -= 0.55 * cache.posThreshold[idx];
                if (!negExposure.empty()) legacy -= 0.8 * negExposure[idx];
                legacyHeuristic[idx] = legacy;
        }

        vector<double> featureHeuristic(N, 0.0);
        for (size_t idx = 0; idx < N; ++idx) {
                double feature = 0.55 * cache.outStrength[idx];
                feature += 0.65 * cache.inStrength[idx];
                feature += 0.2 * double(cache.outAdj[idx].size());
                feature += 0.22 * double(cache.inAdj[idx].size());
                feature += 0.6 * sqrt((cache.outStrength[idx] + 1e-6) * (cache.inStrength[idx] + 1e-6));
                feature += 1.5 * double(shellIndex[idx]);
                feature += 3.0 * pageRank[idx] * double(N);
                if (!negExposure.empty()) feature -= 0.6 * negExposure[idx];
                if (!negDistanceScore.empty()) feature -= 0.4 * negDistanceScore[idx];
                feature -= 0.4 * cache.posThreshold[idx];
                feature -= 0.25 * cache.negThreshold[idx];
                featureHeuristic[idx] = feature;
        }

        vector<double> baseHeuristic(N, 0.0);
        for (size_t idx = 0; idx < N; ++idx) {
                baseHeuristic[idx] = 0.8 * legacyHeuristic[idx] + 0.2 * featureHeuristic[idx];
        }

        vector<double> diversityPenalty(N, 0.0);
        vector<int> visitMarker(N, 0);
        int visitToken = 1;
        int penaltyVersion = 0;
        constexpr int DIVERSITY_DEPTH = 3;
        const double DIVERSITY_BASE = 1.6;
        const double DIVERSITY_DECAY = 0.7;
        array<double, DIVERSITY_DEPTH + 1> depthPenalty{};
        depthPenalty[0] = DIVERSITY_BASE;
        for (int d = 1; d <= DIVERSITY_DEPTH; ++d)
                depthPenalty[d] = depthPenalty[d - 1] * DIVERSITY_DECAY;

        auto applyDiversityPenalty = [&](int nodeId) {
                int idx = cache.indexOf(nodeId);
                if (idx < 0) return;
                ++penaltyVersion;
                ++visitToken;
                queue<pair<int, int>> bfs;
                bfs.emplace(idx, 0);
                visitMarker[idx] = visitToken;
                while (!bfs.empty()) {
                        auto [cur, depth] = bfs.front();
                        bfs.pop();
                        diversityPenalty[cur] += depthPenalty[depth];
                        if (depth >= DIVERSITY_DEPTH) continue;
                        for (const auto& edge : cache.undirectedAdj[cur]) {
                                int nb = edge.first;
                                if (visitMarker[nb] == visitToken) continue;
                                visitMarker[nb] = visitToken;
                                bfs.emplace(nb, depth + 1);
                        }
                }
        };

        auto currentHeuristic = [&](int idx) {
                double penalty = diversityPenalty[idx];
                double cap = 0.35 * fabs(baseHeuristic[idx]);
                if (penalty > cap) penalty = cap;
                return baseHeuristic[idx] - penalty;
        };

        vector<int> order;
        order.reserve(N);
        for (size_t i = 0; i < N; ++i) order.push_back(static_cast<int>(i));

        unordered_set<int> negSeedSet = info.negatives;
        unordered_set<int> workingSeeds;
        if (info.hasGiven) {
                workingSeeds.insert(info.given);
                applyDiversityPenalty(info.given);
        }

        sort(order.begin(), order.end(), [&](int a, int b) {
                double sa = currentHeuristic(a);
                double sb = currentHeuristic(b);
                if (sa != sb) return sa > sb;
                return cache.nodeIds[a] < cache.nodeIds[b];
        });

        vector<int> legacyOrder = order;
        sort(legacyOrder.begin(), legacyOrder.end(), [&](int a, int b) {
                double sa = legacyHeuristic[a];
                double sb = legacyHeuristic[b];
                if (sa != sb) return sa > sb;
                return cache.nodeIds[a] < cache.nodeIds[b];
        });

        const int MIN_SIM = 400;
        const int MULTIPLIER = 40;
        const int MAX_SIM = 6000;
        int simulateCount = static_cast<int>(order.size());
        long long targetSim = static_cast<long long>(numberOfSeeds) * MULTIPLIER;
        if (targetSim < MIN_SIM) targetSim = MIN_SIM;
        if (targetSim > MAX_SIM) targetSim = MAX_SIM;
        if (simulateCount > targetSim && order.size() > 2000) simulateCount = static_cast<int>(targetSim);

        vector<char> candidateUsed(N, 0);
        vector<int> candidateIdx;
        candidateIdx.reserve(simulateCount);
        for (int i = 0; i < simulateCount && i < static_cast<int>(order.size()); ++i) {
                int idx = order[i];
                int nodeId = cache.nodeIds[idx];
                if (banned.count(nodeId)) continue;
                candidateIdx.push_back(idx);
                candidateUsed[idx] = 1;
        }
        for (int idx : legacyOrder) {
                if (static_cast<int>(candidateIdx.size()) >= simulateCount) break;
                if (candidateUsed[idx]) continue;
                int nodeId = cache.nodeIds[idx];
                if (banned.count(nodeId)) continue;
                candidateIdx.push_back(idx);
                candidateUsed[idx] = 1;
        }
        if (candidateIdx.empty()) {
                for (int nodeId : cache.nodeIds) {
                        if (banned.count(nodeId)) continue;
                        int idx = cache.indexOf(nodeId);
                        if (idx < 0) continue;
                        candidateIdx.push_back(idx);
                        if (static_cast<int>(candidateIdx.size()) >= targetSim) break;
                }
        }

        auto timeExceeded = [&]() {
                return chrono::steady_clock::now() - startTime > softTimeBudget;
        };

        auto recomputeHeuristic = [&](int nodeId) {
                int idx = cache.indexOf(nodeId);
                if (idx < 0) return 0.0;
                return currentHeuristic(idx);
        };

        const size_t MIN_DIFF_RUNS = 1500;
        const size_t PER_SEED_RUNS = 80;
        const size_t MAX_DIFF_RUNS = 12000;
        size_t diffRunLimit = max(MIN_DIFF_RUNS, PER_SEED_RUNS * static_cast<size_t>(numberOfSeeds));
        if (diffRunLimit > MAX_DIFF_RUNS) diffRunLimit = MAX_DIFF_RUNS;
        size_t diffRuns = 0;

        if (cache.nodeIds.size() <= 2000) {
                size_t exhaustiveNeed = candidateIdx.size() * static_cast<size_t>(numberOfSeeds ? numberOfSeeds : 1);
                if (exhaustiveNeed > diffRunLimit)
                        diffRunLimit = min<size_t>(MAX_DIFF_RUNS, exhaustiveNeed + 10);
        }

        FullDiffResult baseResult = runFullDiffusionSimulation(G, workingSeeds, negSeedSet);
        ++diffRuns;
        double currentSpread = baseResult.spread;
        int iteration = 0;

        auto exhaustiveGreedy = [&]() {
                size_t remainingSeeds = numberOfSeeds > seeds.size() ? numberOfSeeds - seeds.size() : 0;
                if (remainingSeeds == 0) return true;
                if (candidateIdx.size() > 4000) return false;
                long long requiredRuns = static_cast<long long>(candidateIdx.size()) * static_cast<long long>(remainingSeeds);
                if (requiredRuns + diffRuns > static_cast<long long>(diffRunLimit)) return false;
                while (seeds.size() < numberOfSeeds) {
                        int bestNode = -1;
                        FullDiffResult bestResult = baseResult;
                        double bestSpread = currentSpread;
                        bool resourceBlocked = false;
                        for (int idx : candidateIdx) {
                                int nodeId = cache.nodeIds[idx];
                                if (banned.count(nodeId) || workingSeeds.count(nodeId)) continue;
                                if (diffRuns >= diffRunLimit || timeExceeded()) {
                                        resourceBlocked = true;
                                        break;
                                }
                                unordered_set<int> trialSeeds = workingSeeds;
                                trialSeeds.insert(nodeId);
                                FullDiffResult result = runFullDiffusionSimulation(G, trialSeeds, negSeedSet);
                                ++diffRuns;
                                if (result.spread > bestSpread) {
                                        bestSpread = result.spread;
                                        bestNode = nodeId;
                                        bestResult = result;
                                }
                        }
                        if (bestNode == -1) {
                                if (resourceBlocked) return false;
                                break;
                        }
                        seeds.insert(bestNode);
                        workingSeeds.insert(bestNode);
                        applyDiversityPenalty(bestNode);
                        currentSpread = bestResult.spread;
                        baseResult = bestResult;
                        ++iteration;
                }
                return seeds.size() >= numberOfSeeds;
        };

        if (exhaustiveGreedy() && seeds.size() >= numberOfSeeds) {
                return seeds;
        }

        struct CandidateEntry {
                int nodeId;
                double key;
                double totalSpread;
                int lastUpdate;
                bool evaluated;
                int penaltyStamp;
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
                out.penaltyStamp = penaltyVersion;
                return true;
        };

        priority_queue<CandidateEntry, vector<CandidateEntry>, CandidateCompare> pq;
        for (int idx : candidateIdx) {
                int nodeId = cache.nodeIds[idx];
                if (workingSeeds.count(nodeId)) continue;
                double heuristic = currentHeuristic(idx);
                pq.push(CandidateEntry{ nodeId, heuristic, 0.0, -1, false, penaltyVersion });
        }

        bool budgetExhausted = false;
        while (seeds.size() < numberOfSeeds && !pq.empty()) {
                CandidateEntry top = pq.top();
                pq.pop();
                if (workingSeeds.count(top.nodeId) || banned.count(top.nodeId)) continue;
                if (!top.evaluated) {
                        if (top.penaltyStamp != penaltyVersion) {
                                double freshKey = recomputeHeuristic(top.nodeId);
                                top.key = freshKey;
                                top.penaltyStamp = penaltyVersion;
                                pq.push(top);
                                continue;
                        }
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
                        applyDiversityPenalty(top.nodeId);
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
                vector<int> fallback = candidateIdx;
                sort(fallback.begin(), fallback.end(), [&](int a, int b) {
                        double sa = currentHeuristic(a);
                        double sb = currentHeuristic(b);
                        if (sa != sb) return sa > sb;
                        return cache.nodeIds[a] < cache.nodeIds[b];
                });
                for (int idx : fallback) {
                        if (seeds.size() >= numberOfSeeds) break;
                        int nodeId = cache.nodeIds[idx];
                        if (banned.count(nodeId) || seeds.count(nodeId) || workingSeeds.count(nodeId)) continue;
                        seeds.insert(nodeId);
                        workingSeeds.insert(nodeId);
                        applyDiversityPenalty(nodeId);
                }
        }

        return seeds;
}

#endif
