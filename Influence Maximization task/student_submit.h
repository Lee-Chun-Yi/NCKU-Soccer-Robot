#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <chrono>
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

struct SeedFiles {
        bool loaded = false;
        string dataDir;
        unordered_set<int> forbidden;
        vector<int> givenPos;
        vector<int> negSeeds;
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
        string cur;
        for (char c : raw) {
                if (c == '\0') {
                        if (!cur.empty()) {
                                args.push_back(cur);
                                cur.clear();
                        }
                }
                else {
                        cur.push_back(c);
                }
        }
        if (!cur.empty()) args.push_back(cur);
        return args;
}

static vector<int> readIntList(const string& path) {
        vector<int> values;
        ifstream in(path);
        if (!in.is_open()) return values;
        int v = 0;
        while (in >> v) values.push_back(v);
        return values;
}

static const SeedFiles& getSeedFiles() {
        static SeedFiles files;
        if (files.loaded) return files;
        files.loaded = true;

        vector<string> args = readCmdlineArgs();
        if (args.size() >= 2) files.dataDir = args[1];
        if (files.dataDir.empty()) return files;

        vector<int> given = readIntList(joinPath(files.dataDir, "given_pos.txt"));
        vector<int> neg = readIntList(joinPath(files.dataDir, "neg_seed.txt"));

        files.givenPos = given;
        files.negSeeds = neg;

        for (int v : given) files.forbidden.insert(v);
        for (int v : neg) files.forbidden.insert(v);

        return files;
}

struct SpreadStats {
        int pos = 0;
        int neg = 0;
};

struct ScoreContext {
        DirectedGraph* graph = nullptr;
        vector<int> nodes;
        unordered_map<int, int> nodeIndex;
        vector<double> posOutStrength;
        vector<double> negInRisk;
        vector<double> multiHopStrength;
        vector<double> negAdjPenalty;
        vector<double> negDistPenalty;
        unordered_set<int> baseNegSeeds;
        unordered_set<int> forbidden;
        bool ready = false;
};

static ScoreContext gScoreContext;

struct CandidateEntry {
        int node = -1;
        double gain = numeric_limits<double>::lowest();
        size_t last_update = numeric_limits<size_t>::max();
        SpreadStats stats;
};

struct CandidateComparator {
        bool operator()(const CandidateEntry& a, const CandidateEntry& b) const {
                if (a.gain == b.gain) return a.node > b.node;
                return a.gain < b.gain;
        }
};

static SpreadStats simulateSpread(
        DirectedGraph& G,
        const unordered_set<int>& basePos,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& selected,
        int extraNode)
{
        static unordered_set<int> posSeeds;
        static unordered_set<int> negSeeds;
        static unordered_set<int> finalPos;
        static unordered_set<int> finalNeg;

        posSeeds.clear();
        size_t posNeed = basePos.size() + selected.size() + (extraNode >= 0 ? 1u : 0u);
        posSeeds.reserve(posNeed * 2 + 1);
        posSeeds.insert(basePos.begin(), basePos.end());
        posSeeds.insert(selected.begin(), selected.end());
        if (extraNode >= 0) posSeeds.insert(extraNode);

        negSeeds.clear();
        negSeeds.reserve(baseNeg.size() * 2 + 1);
        negSeeds.insert(baseNeg.begin(), baseNeg.end());

        finalPos.clear();
        finalNeg.clear();
        finalPos.reserve(posSeeds.size() * 2 + 1);
        finalNeg.reserve(negSeeds.size() * 2 + 1);
        diffuse_signed_all(&G, posSeeds, negSeeds, finalPos, finalNeg);

        SpreadStats stats;
        stats.pos = static_cast<int>(finalPos.size());
        stats.neg = static_cast<int>(finalNeg.size());
        return stats;
}

static int getNodeIndex(int nodeId) {
        if (!gScoreContext.ready) return -1;
        auto it = gScoreContext.nodeIndex.find(nodeId);
        if (it == gScoreContext.nodeIndex.end()) return -1;
        return it->second;
}

static double getNodeRisk(int nodeId) {
        int idx = getNodeIndex(nodeId);
        if (idx < 0) return 0.0;
        double exposure = gScoreContext.negInRisk[idx] * 1.1 +
                gScoreContext.negAdjPenalty[idx] +
                gScoreContext.negDistPenalty[idx];
        return exposure;
}

static void prepareScoreContext(
        DirectedGraph& G,
        const unordered_set<int>& baseNeg,
        const unordered_set<int>& forbidden)
{
        ScoreContext ctx;
        ctx.graph = &G;
        ctx.baseNegSeeds = baseNeg;
        ctx.forbidden = forbidden;
        ctx.nodes = G.getAllNodes();
        const size_t n = ctx.nodes.size();
        ctx.nodeIndex.reserve(n * 2);
        for (size_t i = 0; i < n; ++i) ctx.nodeIndex[ctx.nodes[i]] = static_cast<int>(i);
        ctx.posOutStrength.assign(n, 0.0);
        ctx.negInRisk.assign(n, 0.0);
        ctx.multiHopStrength.assign(n, 0.0);
        ctx.negAdjPenalty.assign(n, 0.0);
        ctx.negDistPenalty.assign(n, 0.0);

        vector<vector<pair<int, double>>> positiveEdges(n);
        vector<vector<int>> adjacency(n);

for (size_t idx = 0; idx < n; ++idx) {
const int nodeId = ctx.nodes[idx];
const vector<int> outs = G.getNodeOutNeighbors(nodeId);
for (int nb : outs) {
double w = G.getEdgeInfluence(nodeId, nb);
auto nbIt = ctx.nodeIndex.find(nb);
if (w > 0) {
ctx.posOutStrength[idx] += w;
if (nbIt != ctx.nodeIndex.end()) positiveEdges[idx].emplace_back(nbIt->second, w);
if (baseNeg.count(nb)) ctx.negAdjPenalty[idx] += 0.2 + 0.15 * w;
}
else if (w < 0) {
ctx.negAdjPenalty[idx] += 0.1 * std::fabs(w);
}
                if (nbIt != ctx.nodeIndex.end()) {
                        adjacency[idx].push_back(nbIt->second);
                        adjacency[nbIt->second].push_back(static_cast<int>(idx));
                }
            }

            const vector<int> ins = G.getNodeInNeighbors(nodeId);
            for (int nb : ins) {
                double w = G.getEdgeInfluence(nb, nodeId);
if (w < 0) {
double absw = std::fabs(w);
ctx.negInRisk[idx] += absw;
if (baseNeg.count(nb)) ctx.negAdjPenalty[idx] += absw * 1.0 + 0.3;
}
else if (w > 0 && baseNeg.count(nb)) {
ctx.negAdjPenalty[idx] += 0.25;
}
            }
        }

        for (auto& adj : adjacency) {
                sort(adj.begin(), adj.end());
                adj.erase(unique(adj.begin(), adj.end()), adj.end());
        }

        if (!baseNeg.empty()) {
                vector<int> dist(n, -1);
                queue<int> q;
                for (int negNode : baseNeg) {
                        auto it = ctx.nodeIndex.find(negNode);
                        if (it == ctx.nodeIndex.end()) continue;
                        int idx = it->second;
                        dist[idx] = 0;
                        q.push(idx);
                }
                const int maxDepth = 3;
                while (!q.empty()) {
                        int idx = q.front();
                        q.pop();
                        if (dist[idx] >= maxDepth) continue;
                        for (int nbIdx : adjacency[idx]) {
                                if (dist[nbIdx] != -1) continue;
                                dist[nbIdx] = dist[idx] + 1;
                                q.push(nbIdx);
                        }
                }
for (size_t i = 0; i < n; ++i) {
if (dist[i] < 0) continue;
double penalty = 0.0;
switch (dist[i]) {
case 0: penalty = 2.5; break;
case 1: penalty = 1.5; break;
case 2: penalty = 0.9; break;
default: penalty = 0.4; break;
}
ctx.negDistPenalty[i] = penalty;
}
}

for (size_t idx = 0; idx < n; ++idx) {
double accum = 0.0;
for (const auto& edge : positiveEdges[idx]) {
double neighborStrength = ctx.posOutStrength[edge.first];
double bounded = std::min(neighborStrength, 15.0);
accum += edge.second * (0.3 + 0.01 * bounded);
}
ctx.multiHopStrength[idx] = accum;
}

        ctx.ready = true;
        gScoreContext = std::move(ctx);
}

static double computeNodeScore(DirectedGraph& G, int nodeId) {
        if (!gScoreContext.ready || gScoreContext.graph != &G) return 0.0;
        int idx = getNodeIndex(nodeId);
        if (idx < 0) return 0.0;

const double pos = gScoreContext.posOutStrength[idx];
const double multiHop = gScoreContext.multiHopStrength[idx];
const double negRisk = gScoreContext.negInRisk[idx];
const double adjPenalty = gScoreContext.negAdjPenalty[idx];
const double distPenalty = gScoreContext.negDistPenalty[idx];

const double safeBoost = 0.3 / (1.0 + negRisk);
const double flowBoost = 0.05 * multiHop;

double score = (pos * 1.2 + multiHop * 0.65) -
(negRisk * 0.85 + adjPenalty * 0.6 + distPenalty * 0.8) +
safeBoost + flowBoost;
        return score;
}

static void refineSelection(DirectedGraph& G,
	const unordered_set<int>& basePos,
	const unordered_set<int>& baseNeg,
	const unordered_set<int>& forbidden,
	const vector<pair<int, double>>& ranking,
	unordered_set<int>& selected)
{
	if (selected.empty()) return;
	SpreadStats bestStats = simulateSpread(G, basePos, baseNeg, selected, -1);
	double bestScore = double(bestStats.pos - bestStats.neg);
	vector<int> seedList(selected.begin(), selected.end());
	const size_t candidateChecks = min<size_t>(ranking.size(), 80);
	for (size_t idx = 0, tested = 0; idx < ranking.size() && tested < candidateChecks; ++idx) {
		int candidate = ranking[idx].first;
		if (forbidden.count(candidate)) continue;
		if (selected.count(candidate)) continue;
		++tested;
		bool improved = false;
		for (size_t s = 0; s < seedList.size(); ++s) {
			int removed = seedList[s];
			if (removed == candidate) { improved = true; break; }
			selected.erase(removed);
			SpreadStats stats = simulateSpread(G, basePos, baseNeg, selected, candidate);
			double score = double(stats.pos - stats.neg);
			if (score > bestScore) {
				bestScore = score;
				bestStats = stats;
				seedList[s] = candidate;
				selected.insert(candidate);
				improved = true;
				break;
			}
			selected.insert(removed);
		}
		if (!improved) continue;
	}
}



static void fillWithHeuristics(const vector<pair<int, double>>& ranking,
	const vector<int>& allNodes,
	const unordered_set<int>& forbidden,
	unordered_set<int>& selected,
	unsigned int numberOfSeeds)
{
	if (ranking.empty()) {
		for (int node : allNodes) {
			if (selected.size() >= numberOfSeeds) break;
			if (forbidden.count(node)) continue;
			if (selected.count(node)) continue;
			selected.insert(node);
		}
		return;
	}

const double topScore = ranking.front().second;
const double primaryCutoff = topScore * 0.15;
const double riskLimit = 5.0;
	const size_t targetSeeds = static_cast<size_t>(numberOfSeeds);

	for (size_t idx = 0; idx < ranking.size(); ++idx) {
		const auto& item = ranking[idx];
		if (selected.size() >= numberOfSeeds) break;
		if (forbidden.count(item.first)) continue;
		if (selected.count(item.first)) continue;
		size_t remainingNeed = selected.size() >= targetSeeds ? 0u : targetSeeds - selected.size();
		size_t remainingRank = ranking.size() - idx;
		if (item.second < primaryCutoff && remainingRank > remainingNeed + 4) continue;
		if (getNodeRisk(item.first) > riskLimit && remainingRank > remainingNeed + 6) continue;
		selected.insert(item.first);
	}

	if (selected.size() >= numberOfSeeds) return;

	for (const auto& item : ranking) {
		if (selected.size() >= numberOfSeeds) break;
		if (forbidden.count(item.first)) continue;
		if (selected.count(item.first)) continue;
		selected.insert(item.first);
	}

	if (selected.size() >= numberOfSeeds) return;

	for (int node : allNodes) {
		if (selected.size() >= numberOfSeeds) break;
		if (forbidden.count(node)) continue;
		if (selected.count(node)) continue;
		size_t remainingNeed = selected.size() >= targetSeeds ? 0u : targetSeeds - selected.size();
		if (getNodeRisk(node) > 6.0 && allNodes.size() > remainingNeed + 6) continue;
		selected.insert(node);
	}
}

static unordered_set<int> runSeedSelection(DirectedGraph& G,
	unsigned int numberOfSeeds,
	const unordered_set<int>& basePos,
	const unordered_set<int>& baseNeg,
	const unordered_set<int>& forbidden)
{
	if (numberOfSeeds == 0 || G.getSize() == 0) return {};

	prepareScoreContext(G, baseNeg, forbidden);

	vector<int> allNodes = gScoreContext.ready ? gScoreContext.nodes : G.getAllNodes();
	if (allNodes.empty()) return {};

	vector<pair<int, double>> ranking;
	ranking.reserve(allNodes.size());
	for (int node : allNodes) {
		double score = computeNodeScore(G, node);
		ranking.emplace_back(node, score);
	}
	sort(ranking.begin(), ranking.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
		if (a.second != b.second) return a.second > b.second;
		return a.first < b.first;
	});

	double avgDegree = 0.0;
	if (G.getSize() > 0) avgDegree = static_cast<double>(G.getEdgeNumber()) / static_cast<double>(G.getSize());
	const size_t targetSeeds = static_cast<size_t>(numberOfSeeds);
	size_t candidateLimit = max<size_t>(350, targetSeeds * 5);
	size_t adaptiveLimit = targetSeeds * (avgDegree > 6.0 ? 70 : (avgDegree > 3.0 ? 60 : 50));
	candidateLimit = max(candidateLimit, adaptiveLimit);
	candidateLimit = max(candidateLimit, static_cast<size_t>(avgDegree * 30.0));
	if (ranking.size() >= targetSeeds * 3u)
		candidateLimit = max(candidateLimit, targetSeeds * 3u);
	candidateLimit = min(candidateLimit, ranking.size());
	if (candidateLimit == 0) candidateLimit = ranking.size();

	vector<int> candidatePool;
	candidatePool.reserve(min(candidateLimit, ranking.size()));
	for (const auto& entry : ranking) {
		if (forbidden.count(entry.first)) continue;
		candidatePool.push_back(entry.first);
		if (candidatePool.size() >= candidateLimit) break;
	}

	unordered_set<int> selected;
	selected.reserve(numberOfSeeds * 2);

	SpreadStats currentStats = simulateSpread(G, basePos, baseNeg, selected, -1);

	priority_queue<CandidateEntry, vector<CandidateEntry>, CandidateComparator> heap;
	for (int node : candidatePool) {
		CandidateEntry entry;
		entry.node = node;
		heap.push(entry);
	}

	const auto startTime = chrono::steady_clock::now();
	const double timeLimitSeconds = 600.0;

	const double alpha = 1.45;
	const double beta = 2.5;

	while (selected.size() < numberOfSeeds && !heap.empty()) {
		double elapsed = chrono::duration<double>(chrono::steady_clock::now() - startTime).count();
		if (elapsed >= timeLimitSeconds) break;

		CandidateEntry entry = heap.top();
		heap.pop();

		if (selected.count(entry.node)) continue;

		if (entry.last_update == selected.size()) {
			selected.insert(entry.node);
			currentStats = entry.stats;
			continue;
		}

		SpreadStats withStats = simulateSpread(G, basePos, baseNeg, selected, entry.node);
		double dPos = double(withStats.pos - currentStats.pos);
		double dNeg = double(withStats.neg - currentStats.neg);

entry.gain = dPos * alpha - dNeg * beta;
if (dNeg > dPos * 1.4 && dNeg > 3.0) entry.gain -= dNeg * 1.0;
if (dNeg > dPos * 1.8 && dNeg > 6.0) entry.gain -= dNeg * 1.5;
entry.last_update = selected.size();
entry.stats = withStats;
bool shouldRequeue = true;
size_t remainingNeed = selected.size() >= targetSeeds ? 0u : targetSeeds - selected.size();
if (entry.gain < -80.0 && heap.size() > remainingNeed + 5) shouldRequeue = false;
if (shouldRequeue) heap.push(entry);
}

if (selected.size() < numberOfSeeds) {
fillWithHeuristics(ranking, allNodes, forbidden, selected, numberOfSeeds);
}

if (selected.size() >= numberOfSeeds) {
refineSelection(G, basePos, baseNeg, forbidden, ranking, selected);
}

return selected;
}


} // namespace student_algo_detail

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
        using namespace student_algo_detail;

        if (numberOfSeeds == 0 || G.getSize() == 0) return {};

        const SeedFiles& seedFiles = getSeedFiles();
        unordered_set<int> forbidden = seedFiles.forbidden;

        unordered_set<int> basePos;
        unordered_set<int> baseNeg;
        basePos.reserve(seedFiles.givenPos.size());
        baseNeg.reserve(seedFiles.negSeeds.size());

        for (int v : seedFiles.givenPos)
                if (G.isNodeExist(v)) {
                        basePos.insert(v);
                        forbidden.insert(v);
                }
        for (int v : seedFiles.negSeeds)
                if (G.isNodeExist(v)) {
                        baseNeg.insert(v);
                        forbidden.insert(v);
                }

        return runSeedSelection(G, numberOfSeeds, basePos, baseNeg, forbidden);
}

unordered_set<int> seedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds)
{
        using namespace student_algo_detail;

        if (numberOfSeeds == 0 || G.getSize() == 0) return {};

        const SeedFiles& seedFiles = getSeedFiles();
        unordered_set<int> forbidden = seedFiles.forbidden;

        unordered_set<int> basePos;
        unordered_set<int> baseNeg;
        basePos.reserve(seedFiles.givenPos.size() + 1);
        baseNeg.reserve(seedFiles.negSeeds.size() + givenNegSeeds.size());

        for (int v : seedFiles.givenPos)
                if (G.isNodeExist(v)) {
                        basePos.insert(v);
                        forbidden.insert(v);
                }

        if (G.isNodeExist(givenPosSeed)) {
                basePos.insert(givenPosSeed);
                forbidden.insert(givenPosSeed);
        }

        for (int v : seedFiles.negSeeds)
                if (G.isNodeExist(v)) {
                        baseNeg.insert(v);
                        forbidden.insert(v);
                }

        for (int v : givenNegSeeds)
                if (G.isNodeExist(v)) {
                        baseNeg.insert(v);
                        forbidden.insert(v);
                }

        return runSeedSelection(G, numberOfSeeds, basePos, baseNeg, forbidden);
}

#endif
