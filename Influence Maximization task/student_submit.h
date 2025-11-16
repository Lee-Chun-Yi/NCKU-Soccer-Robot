// student_submit.h - heuristic-based solution
#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

using namespace std;

namespace {

double computeNodeScore(DirectedGraph& G, int node) {
        const vector<int> outNeighbors = G.getNodeOutNeighbors(node);
        const vector<int> inNeighbors = G.getNodeInNeighbors(node);

        double outInfluence = 0.0;
        for (int nbr : outNeighbors) {
                double w = G.getEdgeInfluence(node, nbr);
                if (w >= 0) outInfluence += w;
                else outInfluence += 0.25 * w;
        }

        double inInfluence = 0.0;
        for (int nbr : inNeighbors) {
                double w = G.getEdgeInfluence(nbr, node);
                if (w >= 0) inInfluence += 0.6 * w;
                else inInfluence += 0.2 * w;
        }

        double degreeBonus = 0.15 * (outNeighbors.size() + inNeighbors.size());
        double thresholdPenalty = 0.35 * max(1e-6, G.getNodeThreshold(node));
        double negBonus = 0.1 * fabs(G.getNodeThreshold2(node));

        return outInfluence + inInfluence + degreeBonus - thresholdPenalty + negBonus;
}

unordered_set<int> selectSeeds(
        DirectedGraph& G,
        unsigned int numberOfSeeds,
        const unordered_set<int>& banned)
{
        unordered_set<int> seeds;
        if (numberOfSeeds == 0) return seeds;

        vector<int> nodes = G.getAllNodes();
        struct NodeMetric { int id; double score; };
        vector<NodeMetric> metrics;
        metrics.reserve(nodes.size());

        for (int node : nodes) {
                if (banned.count(node)) continue;
                metrics.push_back({ node, computeNodeScore(G, node) });
        }

        sort(metrics.begin(), metrics.end(), [](const NodeMetric& a, const NodeMetric& b) {
                if (a.score != b.score) return a.score > b.score;
                return a.id < b.id;
        });

        for (const auto& entry : metrics) {
                seeds.insert(entry.id);
                if (seeds.size() >= numberOfSeeds) break;
        }

        if (seeds.size() < numberOfSeeds) {
                for (int node : nodes) {
                        if (banned.count(node) || seeds.count(node)) continue;
                        seeds.insert(node);
                        if (seeds.size() >= numberOfSeeds) break;
                }
        }

        return seeds;
}

} // namespace

unordered_set<int> seedSelection(DirectedGraph& G, unsigned int numberOfSeeds) {
        static const unordered_set<int> empty;
        return selectSeeds(G, numberOfSeeds, empty);
}

unordered_set<int> seedSelection(
        DirectedGraph& G,
        unsigned int numberOfSeeds,
        int givenPosSeed,
        const unordered_set<int>& givenNegSeeds)
{
        unordered_set<int> banned = givenNegSeeds;
        if (givenPosSeed >= 0) banned.insert(givenPosSeed);
        return selectSeeds(G, numberOfSeeds, banned);
}

#endif
#pragma once
