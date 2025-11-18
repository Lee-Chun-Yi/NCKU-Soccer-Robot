#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

namespace SeedSelectionImpl {

struct CELFEntry {
    double gain;
    int nodeIdx;
    int lastUpdate;
    bool operator<(const CELFEntry& other) const {
        if (gain == other.gain) return nodeIdx < other.nodeIdx;
        return gain < other.gain; // max-heap
    }
};

inline vector<vector<pair<int, double>>> buildAdjList(
    const DirectedGraph& G,
    const vector<int>& nodes,
    const unordered_map<int, int>& idToIdx,
    bool reverse) {
    vector<vector<pair<int, double>>> adj(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        int uId = nodes[i];
        vector<int> neighs = reverse ? G.getNodeInNeighbors(uId) : G.getNodeOutNeighbors(uId);
        adj[i].reserve(neighs.size());
        for (int vId : neighs) {
            auto it = idToIdx.find(vId);
            if (it == idToIdx.end()) continue;
            double w = reverse ? G.getEdgeInfluence(vId, uId) : G.getEdgeInfluence(uId, vId);
            if (w <= 0.0) continue;
            adj[i].emplace_back(it->second, w);
        }
    }
    return adj;
}

inline vector<double> computeOutWeightSum(const vector<vector<pair<int, double>>>& outAdj) {
    vector<double> sums(outAdj.size(), 0.0);
    for (size_t i = 0; i < outAdj.size(); ++i) {
        double acc = 0.0;
        for (const auto& p : outAdj[i]) acc += p.second;
        sums[i] = acc;
    }
    return sums;
}

inline vector<int> selectSmall(unsigned int k,
    const vector<int>& nodeIds,
    const vector<vector<pair<int, double>>>& outAdj,
    const vector<double>& outWeightSum,
    const vector<bool>& banned,
    const vector<bool>& negMask) {
    vector<pair<double, int>> scored;
    scored.reserve(nodeIds.size());
    for (size_t i = 0; i < nodeIds.size(); ++i) {
        if (banned[i]) continue;
        double score = outWeightSum[i];
        for (const auto& [nb, w] : outAdj[i]) {
            score += 0.5 * w * outWeightSum[nb];
            if (negMask[nb]) score += 0.2 * w;
        }
        scored.emplace_back(score, static_cast<int>(i));
    }
    sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        if (a.first == b.first) return a.second < b.second;
        return a.first > b.first;
    });
    vector<int> result;
    result.reserve(k);
    for (size_t i = 0; i < scored.size() && result.size() < k; ++i)
        result.push_back(scored[i].second);
    return result;
}

inline double estimateSpreadFromNode(int startIdx,
    const vector<vector<pair<int, double>>>& outAdj,
    const vector<bool>& banned,
    double eta) {
    const int n = static_cast<int>(outAdj.size());
    vector<int> nodeStack;
    vector<size_t> childIdx;
    vector<double> probStack;
    vector<char> onPath(n, 0);
    nodeStack.reserve(n);
    childIdx.reserve(n);
    probStack.reserve(n);
    nodeStack.push_back(startIdx);
    childIdx.push_back(0);
    probStack.push_back(1.0);
    onPath[startIdx] = 1;
    double spread = 1.0;
    while (!nodeStack.empty()) {
        int node = nodeStack.back();
        size_t idx = childIdx.back();
        double prob = probStack.back();
        if (idx >= outAdj[node].size()) {
            onPath[node] = 0;
            nodeStack.pop_back();
            childIdx.pop_back();
            probStack.pop_back();
            continue;
        }
        auto [nb, w] = outAdj[node][idx];
        childIdx.back()++;
        if (onPath[nb]) continue;
        if (banned[nb]) continue;
        double nextProb = prob * w;
        if (nextProb < eta) continue;
        spread += nextProb;
        nodeStack.push_back(nb);
        childIdx.push_back(0);
        probStack.push_back(nextProb);
        onPath[nb] = 1;
    }
    return spread;
}

inline vector<int> selectMedium(unsigned int k,
    const vector<vector<pair<int, double>>>& outAdj,
    const vector<bool>& initialBanned) {
    const double eta = 1e-4;
    vector<bool> banned = initialBanned;
    priority_queue<CELFEntry> pq;
    for (size_t i = 0; i < outAdj.size(); ++i) {
        if (banned[i]) continue;
        double gain = estimateSpreadFromNode(static_cast<int>(i), outAdj, banned, eta);
        pq.push({gain, static_cast<int>(i), 0});
    }
    vector<int> result;
    result.reserve(k);
    int iter = 0;
    while (!pq.empty() && result.size() < k) {
        CELFEntry cur = pq.top();
        pq.pop();
        if (banned[cur.nodeIdx]) continue;
        if (cur.lastUpdate < iter) {
            double gain = estimateSpreadFromNode(cur.nodeIdx, outAdj, banned, eta);
            pq.push({gain, cur.nodeIdx, iter});
            continue;
        }
        result.push_back(cur.nodeIdx);
        banned[cur.nodeIdx] = true;
        iter++;
    }
    return result;
}

inline vector<int> selectLarge(unsigned int k,
    const vector<vector<pair<int, double>>>& outAdj,
    const vector<vector<pair<int, double>>>& inAdj,
    const vector<bool>& initialBanned,
    mt19937& rng) {
    const int n = static_cast<int>(outAdj.size());
    int targetRR = max(20000, min(80000, n * static_cast<int>(max(1u, k))));
    vector<vector<int>> rrSets;
    rrSets.reserve(targetRR);
    vector<vector<int>> nodeToRR(n);
    vector<int> visitMark(n, 0);
    int visitToken = 0;
    uniform_int_distribution<int> nodeDist(0, n - 1);
    uniform_real_distribution<double> probDist(0.0, 1.0);
    queue<int> q;
    for (int i = 0; i < targetRR; ++i) {
        int start = nodeDist(rng);
        if (++visitToken == INT32_MAX) {
            fill(visitMark.begin(), visitMark.end(), 0);
            visitToken = 1;
        }
        vector<int> rr;
        rr.reserve(32);
        while (!q.empty()) q.pop();
        q.push(start);
        visitMark[start] = visitToken;
        rr.push_back(start);
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            for (const auto& [pred, w] : inAdj[node]) {
                if (visitMark[pred] == visitToken) continue;
                if (probDist(rng) <= w) {
                    visitMark[pred] = visitToken;
                    q.push(pred);
                    rr.push_back(pred);
                }
            }
        }
        int rrId = static_cast<int>(rrSets.size());
        rrSets.push_back(rr);
        for (int nodeIdx : rr) {
            nodeToRR[nodeIdx].push_back(rrId);
        }
    }
    vector<char> rrCovered(rrSets.size(), 0);
    vector<bool> banned = initialBanned;
    priority_queue<CELFEntry> pq;
    for (int i = 0; i < n; ++i) {
        if (banned[i]) continue;
        double gain = static_cast<double>(nodeToRR[i].size());
        pq.push({gain, i, 0});
    }
    vector<int> result;
    result.reserve(k);
    int iter = 0;
    while (!pq.empty() && result.size() < k) {
        CELFEntry cur = pq.top();
        pq.pop();
        if (banned[cur.nodeIdx]) continue;
        if (cur.lastUpdate < iter) {
            double gain = 0.0;
            for (int rrId : nodeToRR[cur.nodeIdx])
                if (!rrCovered[rrId]) gain += 1.0;
            pq.push({gain, cur.nodeIdx, iter});
            continue;
        }
        result.push_back(cur.nodeIdx);
        banned[cur.nodeIdx] = true;
        for (int rrId : nodeToRR[cur.nodeIdx]) {
            if (rrCovered[rrId]) continue;
            rrCovered[rrId] = 1;
        }
        iter++;
    }
    return result;
}

} // namespace SeedSelectionImpl

unordered_set<int> seedSelection(DirectedGraph& G,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int>& givenNegSeeds) {
    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;

    vector<int> nodeIds = G.getAllNodes();
    const int n = static_cast<int>(nodeIds.size());
    if (n == 0) return seeds;

    unordered_map<int, int> idToIdx;
    idToIdx.reserve(nodeIds.size());
    for (size_t i = 0; i < nodeIds.size(); ++i)
        idToIdx[nodeIds[i]] = static_cast<int>(i);

    auto outAdj = SeedSelectionImpl::buildAdjList(G, nodeIds, idToIdx, false);
    auto inAdj = SeedSelectionImpl::buildAdjList(G, nodeIds, idToIdx, true);
    vector<double> outWeightSum = SeedSelectionImpl::computeOutWeightSum(outAdj);

    vector<bool> banned(nodeIds.size(), false);
    vector<bool> negMask(nodeIds.size(), false);
    if (givenPosSeed != -1) {
        auto it = idToIdx.find(givenPosSeed);
        if (it != idToIdx.end()) banned[it->second] = true;
    }
    for (int negId : givenNegSeeds) {
        auto it = idToIdx.find(negId);
        if (it != idToIdx.end()) {
            banned[it->second] = true;
            negMask[it->second] = true;
        }
    }

    vector<int> selectedIdx;
    selectedIdx.reserve(numberOfSeeds);
    if (n <= 200) {
        selectedIdx = SeedSelectionImpl::selectSmall(numberOfSeeds, nodeIds, outAdj, outWeightSum, banned, negMask);
    } else if (n <= 2000) {
        selectedIdx = SeedSelectionImpl::selectMedium(numberOfSeeds, outAdj, banned);
    } else {
        static mt19937 rng(712367821);
        selectedIdx = SeedSelectionImpl::selectLarge(numberOfSeeds, outAdj, inAdj, banned, rng);
    }

    for (int idx : selectedIdx) {
        if (idx < 0 || idx >= n) continue;
        if (banned[idx]) continue;
        seeds.insert(nodeIds[idx]);
        if (seeds.size() >= numberOfSeeds) break;
    }

    if (seeds.size() < numberOfSeeds) {
        for (int i = 0; i < n && seeds.size() < numberOfSeeds; ++i) {
            if (banned[i]) continue;
            seeds.insert(nodeIds[i]);
        }
    }

    return seeds;
}

#endif
