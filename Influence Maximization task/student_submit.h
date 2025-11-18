#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

/*
 * seedSelection:
 *   - G:   整張圖
 *   - numberOfSeeds: 需要你選的「額外正向種子數量」
 *                     （系統另外會再加上 1 個 given_pos.txt 裡的 positive seed）
 *
 * 你只需要回傳一個 unordered_set<int>，裡面放 numberOfSeeds 9個positivate seed編號。
 */
namespace student_algo {

// Simple heuristic that prefers nodes with larger outgoing influence
// and penalizes strong negative neighbors.
inline double heuristicScore(DirectedGraph& G, int node,
        const unordered_set<int>& negSeeds) {
        double posScore = 0.0;
        for (int nbr : G.getNodeOutNeighbors(node))
                posScore += G.getEdgeInfluence(node, nbr);

        double negPenalty = 0.0;
        for (int nbr : G.getNodeInNeighbors(node))
                if (negSeeds.count(nbr))
                        negPenalty += std::fabs(G.getEdgeInfluence(nbr, node));

        return posScore - negPenalty - G.getNodeThreshold(node);
}

} // namespace student_algo

unordered_set<int> seedSelection(DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds) {
        unordered_set<int> seeds;
        if (numberOfSeeds == 0) return seeds;

        vector<int> nodes = G.getAllNodes();
        vector<pair<double, int>> ranked;
        ranked.reserve(nodes.size());
        for (int node : nodes) {
                if (node == givenPosSeed) continue;
                if (givenNegSeeds.count(node)) continue;
                double score = student_algo::heuristicScore(G, node, givenNegSeeds);
                ranked.emplace_back(score, node);
        }

        sort(ranked.begin(), ranked.end(),
                [](const pair<double,int>& a, const pair<double,int>& b) {
                        if (a.first == b.first) return a.second > b.second;
                        return a.first > b.first;
                });

        const size_t candidateLimit = std::max<size_t>(numberOfSeeds * 5u, 30u);
        if (ranked.size() > candidateLimit)
                ranked.resize(candidateLimit);

        for (size_t i = 0; i < ranked.size() && seeds.size() < numberOfSeeds; ++i)
                seeds.insert(ranked[i].second);

        if (seeds.size() < numberOfSeeds) {
                for (int node : nodes) {
                        if (node == givenPosSeed) continue;
                        if (givenNegSeeds.count(node)) continue;
                        seeds.insert(node);
                        if (seeds.size() == numberOfSeeds) break;
                }
        }

        return seeds;
}

#endif
