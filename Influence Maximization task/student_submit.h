#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "LT.h"
#include "graph.h"

using namespace std;

namespace detail {
namespace small_strategy {
struct ParameterSet {
    double overlap_penalty;
    unsigned int max_defense_seeds;
    unsigned int defense_budget_divisor;
};

static const ParameterSet SMALL_BASE_PARAMS = { 0.7, 4, 2 };

// =====================================================================================
//  BASIC UTILITIES（�? v3�?
// =====================================================================================
static double sumOutInfluence(DirectedGraph &G, int node)
{
    double sum = 0.0;
    for (int v : G.getNodeOutNeighbors(node)) {
        sum += G.getEdgeInfluence(node, v);
    }
    return sum;
}

static void computeOutStats(
    DirectedGraph &G,
    const vector<int> &nodes,
    unordered_map<int,double> &outWeight)
{
    outWeight.clear();
    for (int u : nodes) {
        outWeight[u] = sumOutInfluence(G, u);
    }
}

// =====================================================================================
//  SIGNED LT for small deep simulation
// =====================================================================================
static void runSignedLT(
    DirectedGraph &G,
    const unordered_set<int> &posInit,
    const unordered_set<int> &negInit,
    int rounds,
    unordered_set<int> &posOut,
    unordered_set<int> &negOut)
{
    posOut = posInit;
    negOut = negInit;

    for (int t = 0; t < rounds; t++) {
        bool changed = false;
        unordered_set<int> np = posOut;
        unordered_set<int> nn = negOut;

        for (int v : G.getAllNodes()) {
            if (posOut.count(v) || negOut.count(v)) continue;

            double posInf = 0.0, negInf = 0.0;
            for (int u : G.getNodeInNeighbors(v)) {
                double w = G.getEdgeInfluence(u, v);
                if (posOut.count(u)) posInf += w;
                if (negOut.count(u)) negInf += w;
            }

            double thPos = G.getNodeThreshold(v);
            double thNeg = G.getNodeThreshold2(v);
            double net = posInf - negInf;

            if (net >= thPos) {
                np.insert(v);
                changed = true;
            } else if (net <= thNeg) {
                nn.insert(v);
                changed = true;
            }
        }

        posOut = np;
        negOut = nn;
        if (!changed) break;
    }
}

static int countPosAfterLT(
    DirectedGraph &G,
    const unordered_set<int> &posInit,
    const unordered_set<int> &negInit)
{
    unordered_set<int> posF, negF;
    runSignedLT(G, posInit, negInit, 4, posF, negF);
    return (int)posF.size();
}


// =====================================================================================
//  Version2 Defensive (medium/large) ??保�?不�?
// =====================================================================================
//  Version2 Offensive（�??��?變�?
// =====================================================================================
static void chooseOffensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params,
    const unordered_map<int,double> *trueOutWeight = nullptr,
    const unordered_map<int,double> *negPressure = nullptr,
    bool enablePairSynergy = false)
{
    if (chosenSeeds.size() >= totalBudget) return;

    unordered_set<int> touched;
    bool isSmallGraph = allNodes.size() <= 150;
    unordered_set<int> ltBasePos;
    unordered_set<int> ltCandidatePos;

    auto initCover = [&](int s){
        for (int v : G.getNodeOutNeighbors(s)) touched.insert(v);
    };
    for (int s : chosenSeeds) initCover(s);
    if (givenPosSeed >= 0) initCover(givenPosSeed);

    auto getOutWeight = [&](int node) -> double {
        if (!trueOutWeight) return 0.0;
        auto it = trueOutWeight->find(node);
        return it != trueOutWeight->end() ? it->second : 0.0;
    };

    vector<int> cand;
    for (int u : allNodes) {
        if (chosenSeeds.count(u)) continue;
        if (givenNegSeeds.count(u)) continue;
        if (u == givenPosSeed) continue;
        if (outWeight.at(u) <= 0) continue;
        cand.push_back(u);
    }

    while (chosenSeeds.size() < totalBudget && !cand.empty()) {
        int best = -1;
        double bestGain = -numeric_limits<double>::infinity();

        for (int u : cand) {
            if (chosenSeeds.count(u)) continue;
            double scenarioScore = outWeight.at(u);
            double baseGain = 0.0;
            int overlapCount = 0;
            vector<int> firstHop = G.getNodeOutNeighbors(u);
            for (int v : firstHop) {
                double w = G.getEdgeInfluence(u, v);
                if (w <= 0) continue;
                baseGain += w;
                if (touched.count(v)) overlapCount++;
            }

            double penaltyTerm = params.overlap_penalty * overlapCount;
            double heuristicScore;
            if (trueOutWeight && negPressure) {
                double expectedGain = 0.0;
                for (int v : firstHop) {
                    expectedGain += getOutWeight(v);
                    for (int w : G.getNodeOutNeighbors(v)) {
                        expectedGain += 0.5 * getOutWeight(w);
                    }
                }
                double negPress = 0.0;
                auto it = negPressure->find(u);
                if (it != negPressure->end()) negPress = it->second;
                expectedGain -= 0.7 * negPress;
                heuristicScore = 0.6 * baseGain + 0.4 * expectedGain - penaltyTerm;
            } else {
                heuristicScore = baseGain - penaltyTerm;
            }

            if (enablePairSynergy && overlapCount > 0) {
                heuristicScore += 0.05 * overlapCount;
            }

            double finalScore = heuristicScore;
            if (isSmallGraph) {
                // On small graphs, simulate signed LT to capture true marginal spread.
                ltBasePos = chosenSeeds;
                if (givenPosSeed >= 0) ltBasePos.insert(givenPosSeed);
                ltCandidatePos = ltBasePos;
                ltCandidatePos.insert(u);
                int baselinePosCount = countPosAfterLT(G, ltBasePos, givenNegSeeds);
                int withCandidate = countPosAfterLT(G, ltCandidatePos, givenNegSeeds);
                double marginal = static_cast<double>(withCandidate - baselinePosCount);
                finalScore = 0.6 * marginal + 0.4 * heuristicScore;
            }

            finalScore = 0.6 * finalScore + 0.4 * scenarioScore;

            if (finalScore > bestGain) {
                bestGain = finalScore;
                best = u;
            }
        }

        if (best == -1 || bestGain <= 0) break;

        chosenSeeds.insert(best);
        for (int v : G.getNodeOutNeighbors(best)) touched.insert(v);
    }
}

static void fillFallbackSeeds(
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> *negPressure = nullptr,
    bool preferLowPressure = false)
{
    if (seeds.size() >= totalBudget) return;
    vector<pair<int,double>> extra;
    extra.reserve(allNodes.size());
    for (int u : allNodes) {
        if (seeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        double score = outWeight.at(u);
        if (preferLowPressure && negPressure) {
            auto it = negPressure->find(u);
            if (it != negPressure->end()) {
                score -= 0.3 * it->second;
            }
        }
        extra.emplace_back(u, score);
    }
    sort(extra.begin(), extra.end(), [](auto &a, auto &b){ return a.second > b.second; });
    for (auto &p : extra) {
        if (seeds.size() >= totalBudget) break;
        seeds.insert(p.first);
    }
}

struct SmallGraphStats {
    unordered_map<int,double> approxTwoHop;
    unordered_map<int,double> negPressure;
    double maxNegHubOutWeight = 0.0;
    int maxNegHubNode = -1;
    bool anchorThreatened = false;
    int communityCount = 1;
};

static unordered_map<int,double> computeApproxTwoHop(
    DirectedGraph &G,
    const vector<int> &nodes,
    const unordered_map<int,double> &outWeight)
{
    unordered_map<int,double> approx;
    approx.reserve(nodes.size());
    for (int u : nodes) {
        double val = 0.0;
        auto it = outWeight.find(u);
        if (it != outWeight.end()) val = it->second;
        for (int v : G.getNodeOutNeighbors(u)) {
            auto jt = outWeight.find(v);
            if (jt != outWeight.end()) {
                val += 0.5 * jt->second;
            }
        }
        approx[u] = val;
    }
    return approx;
}

static unordered_map<int,double> computeNegativePressure(
    DirectedGraph &G,
    const vector<int> &nodes,
    const unordered_set<int> &givenNeg)
{
    unordered_map<int,double> pressure;
    pressure.reserve(nodes.size());
    for (int u : nodes) pressure[u] = 0.0;
    for (int neg : givenNeg) {
        for (int mid : G.getNodeOutNeighbors(neg)) {
            double wNeg = max(0.0, G.getEdgeInfluence(neg, mid));
            pressure[mid] += wNeg;
            for (int target : G.getNodeOutNeighbors(mid)) {
                double wMid = max(0.0, G.getEdgeInfluence(mid, target));
                pressure[target] += 0.5 * wNeg * wMid;
            }
        }
    }
    return pressure;
}

static vector<vector<int>> computeConnectedComponents(
    DirectedGraph &G,
    const vector<int> &nodes)
{
    vector<vector<int>> components;
    components.reserve(nodes.size());
    unordered_set<int> visited;
    visited.reserve(nodes.size());
    for (int start : nodes) {
        if (!visited.insert(start).second) continue;
        vector<int> component;
        component.push_back(start);
        queue<int> q;
        q.push(start);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            auto expand = [&](const vector<int> &nbrs){
                for (int v : nbrs) {
                    if (!visited.insert(v).second) continue;
                    component.push_back(v);
                    q.push(v);
                }
            };
            expand(G.getNodeOutNeighbors(u));
            expand(G.getNodeInNeighbors(u));
        }
        components.push_back(std::move(component));
    }
    if (components.empty()) components.push_back({});
    return components;
}

static int estimateCommunityCount(
    DirectedGraph &G,
    const vector<int> &nodes)
{
    auto comps = computeConnectedComponents(G, nodes);
    return max(static_cast<int>(comps.size()), 1);
}

static SmallGraphStats buildSmallGraphStats(
    DirectedGraph &G,
    const vector<int> &nodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    const unordered_map<int,double> &outWeight)
{
    SmallGraphStats stats;
    stats.approxTwoHop = computeApproxTwoHop(G, nodes, outWeight);
    stats.negPressure = computeNegativePressure(G, nodes, givenNeg);
    stats.communityCount = estimateCommunityCount(G, nodes);
    for (int neg : givenNeg) {
        double w = 0.0;
        auto it = outWeight.find(neg);
        if (it != outWeight.end()) w = it->second;
        if (w > stats.maxNegHubOutWeight) {
            stats.maxNegHubOutWeight = w;
            stats.maxNegHubNode = neg;
        }
    }
    if (givenPosSeed >= 0) {
        double pressure = 0.0;
        auto it = stats.negPressure.find(givenPosSeed);
        if (it != stats.negPressure.end()) pressure = it->second;
        if (pressure > 0.15) stats.anchorThreatened = true;
        if (!stats.anchorThreatened) {
            for (int neg : givenNeg) {
                for (int v : G.getNodeOutNeighbors(neg)) {
                    if (v == givenPosSeed) {
                        stats.anchorThreatened = true;
                        break;
                    }
                }
                if (stats.anchorThreatened) break;
            }
        }
    }
    return stats;
}

enum class OffenseFocus {
    LOCAL_ANCHOR,
    HUB_COMMUNITY,
    GLOBAL_FULL
};

enum class SmallGraphScenario {
    S1,
    S2,
    S3,
    S4
};

struct StrategyConfig {
    SmallGraphScenario scenario = SmallGraphScenario::S4;
    unsigned int anchorDefense = 0;
    unsigned int frontierDefense = 0;
    double overlapPenalty = 0.85;
    OffenseFocus offenseFocus = OffenseFocus::GLOBAL_FULL;
    bool maximizeCoverage = false;
    bool conservativeFallback = false;
    int focusHubNode = -1;
    double twoHopWeight = 0.5;
    bool enablePairSynergy = false;
};

static StrategyConfig deriveSmallStrategy(
    const SmallGraphStats &stats,
    unsigned int defenseBudget,
    bool hasNegative,
    int givenPosSeed)
{
    StrategyConfig cfg;
    cfg.focusHubNode = stats.maxNegHubNode;
    if (!hasNegative) {
        cfg.scenario = SmallGraphScenario::S3;
        cfg.overlapPenalty = 1.05;
        cfg.maximizeCoverage = true;
        cfg.enablePairSynergy = true;
        return cfg;
    }

    auto remainingBudget = [&](unsigned int used){
        if (used >= defenseBudget) return 0u;
        return defenseBudget - used;
    };

    if (stats.anchorThreatened) {
        cfg.scenario = SmallGraphScenario::S1;
        cfg.offenseFocus = OffenseFocus::LOCAL_ANCHOR;
        cfg.overlapPenalty = 0.15;
        cfg.anchorDefense = min(1u, defenseBudget);
        unsigned int remain = remainingBudget(cfg.anchorDefense);
        if (remain >= 2) cfg.frontierDefense = 2;
        else if (remain == 1) cfg.frontierDefense = 1;
        cfg.conservativeFallback = false;
        return cfg;
    }

    if (stats.communityCount >= 2) {
        cfg.scenario = SmallGraphScenario::S3;
        cfg.overlapPenalty = 1.05;
        cfg.maximizeCoverage = true;
        cfg.offenseFocus = OffenseFocus::GLOBAL_FULL;
        cfg.anchorDefense = min(1u, defenseBudget);
        unsigned int remain = defenseBudget > cfg.anchorDefense
            ? defenseBudget - cfg.anchorDefense
            : 0u;
        cfg.frontierDefense = remain;
        cfg.enablePairSynergy = true;
        return cfg;
    }

    if (stats.maxNegHubOutWeight >= 0.30) {
        cfg.scenario = SmallGraphScenario::S2;
        cfg.offenseFocus = OffenseFocus::HUB_COMMUNITY;
        cfg.overlapPenalty = 0.25;
        cfg.anchorDefense = min(1u, defenseBudget);
        unsigned int remain = remainingBudget(cfg.anchorDefense);
        if (remain >= 3) cfg.frontierDefense = 3;
        else if (remain == 2) cfg.frontierDefense = 2;
        else cfg.frontierDefense = remain;
        cfg.conservativeFallback = true;
        return cfg;
    }

    cfg.scenario = SmallGraphScenario::S4;
    cfg.offenseFocus = OffenseFocus::GLOBAL_FULL;
    cfg.overlapPenalty = 0.85;
    cfg.anchorDefense = min(1u, defenseBudget);
    unsigned int remain = defenseBudget > cfg.anchorDefense
        ? defenseBudget - cfg.anchorDefense
        : 0u;
    cfg.frontierDefense = min(1u, remain);
    cfg.twoHopWeight = 0.7;
    cfg.enablePairSynergy = true;
    return cfg;
}

static unordered_set<int> gatherNodesWithinHops(
    DirectedGraph &G,
    int start,
    int hops)
{
    unordered_set<int> visited;
    if (start < 0 || hops <= 0) return visited;
    queue<pair<int,int>> q;
    q.push({start, 0});
    visited.insert(start);
    while (!q.empty()) {
        pair<int,int> cur = q.front();
        int node = cur.first;
        int dist = cur.second;
        q.pop();
        if (dist >= hops) continue;
        auto expand = [&](const vector<int> &nbrs){
            for (int v : nbrs) {
                if (visited.insert(v).second) {
                    q.push({v, dist + 1});
                }
            }
        };
        expand(G.getNodeOutNeighbors(node));
        expand(G.getNodeInNeighbors(node));
    }
    return visited;
}

static unordered_set<int> buildOffenseMask(
    DirectedGraph &G,
    const StrategyConfig &strategy,
    int givenPosSeed)
{
    unordered_set<int> mask;
    if (strategy.offenseFocus == OffenseFocus::GLOBAL_FULL) return mask;
    if (strategy.offenseFocus == OffenseFocus::LOCAL_ANCHOR) {
        return gatherNodesWithinHops(G, givenPosSeed, 2);
    }
    return gatherNodesWithinHops(G, strategy.focusHubNode, 2);
}

static unordered_map<int,double> buildOffenseWeights(
    const vector<int> &nodes,
    const unordered_map<int,double> &baseOutWeight,
    const SmallGraphStats &stats,
    const StrategyConfig &strategy,
    const unordered_set<int> &mask)
{
    unordered_map<int,double> weights = baseOutWeight;
    for (int u : nodes) {
        double maskFactor = 1.0;
        if (!mask.empty() && !mask.count(u)) {
            maskFactor = 0.35;
        }
        double base = baseOutWeight.at(u);
        double approx = base;
        auto itApprox = stats.approxTwoHop.find(u);
        if (itApprox != stats.approxTwoHop.end()) approx = itApprox->second;
        double pressure = 0.0;
        auto itPress = stats.negPressure.find(u);
        if (itPress != stats.negPressure.end()) pressure = itPress->second;
        double value = base;
        switch (strategy.scenario) {
            case SmallGraphScenario::S1:
                value = 0.7 * base + 0.3 * approx - 0.5 * pressure;
                break;
            case SmallGraphScenario::S2:
                value = 0.8 * base + 0.2 * approx - 0.3 * pressure;
                break;
            case SmallGraphScenario::S3:
                value = approx + 0.2 * base;
                break;
            case SmallGraphScenario::S4:
            default:
                value = (1.0 - strategy.twoHopWeight) * base
                      + strategy.twoHopWeight * approx
                      - 0.1 * pressure;
                break;
        }
        if (strategy.maximizeCoverage) value += 0.1 * approx;
        value *= maskFactor;
        if (value < 0.0) value = 0.0;
        weights[u] = value;
    }
    return weights;
}

static int simulateDefenseScore(
    DirectedGraph &G,
    int candidate,
    int givenPos,
    const unordered_set<int> &givenNeg,
    const unordered_set<int> &existingPos)
{
    unordered_set<int> posInit = existingPos;
    posInit.insert(candidate);
    if (givenPos >= 0) posInit.insert(givenPos);

    unordered_set<int> posF, negF;
    runSignedLT(G, posInit, givenNeg, 4, posF, negF);

    return (int)negF.size();
}

static unordered_map<int,int> computeNegDistances(
    DirectedGraph &G,
    const unordered_set<int> &givenNeg)
{
    unordered_map<int,int> dist;
    queue<int> q;
    for (int neg : givenNeg) {
        if (dist.emplace(neg, 0).second) {
            q.push(neg);
        }
    }
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        int d = dist[node];
        for (int nxt : G.getNodeOutNeighbors(node)) {
            if (dist.emplace(nxt, d + 1).second) {
                q.push(nxt);
            }
        }
    }
    return dist;
}

static void chooseSmallDefense(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPos,
    unsigned int defBudget,
    unordered_set<int> &chosen)
{
    if (defBudget == 0) return;

    unordered_map<int,int> negDist = computeNegDistances(G, givenNeg);

    vector<pair<int,double>> negHub;
    for (int h : givenNeg) {
        negHub.push_back({h, sumOutInfluence(G, h)});
    }
    sort(negHub.begin(), negHub.end(), [](auto&a,auto&b){return a.second>b.second;});

    unordered_set<int> candidates;
    for (auto &hw : negHub) {
        int h = hw.first;
        for (int v : G.getNodeOutNeighbors(h)) {
            if (!givenNeg.count(v) && v != givenPos) {
                candidates.insert(v);
            }
            for (int u : G.getNodeInNeighbors(v)) {
                if (givenNeg.count(u)) continue;
                if (u == givenPos) continue;
                candidates.insert(u);
            }
        }
    }

    if (candidates.size() < defBudget) {
        vector<pair<int,double>> extra;
        for (int u : allNodes) {
            if (givenNeg.count(u) || u == givenPos) continue;
            extra.push_back({u, sumOutInfluence(G, u)});
        }
        sort(extra.begin(), extra.end(), [](auto&a,auto&b){return a.second>b.second;});
        for (auto &p : extra) {
            if (candidates.size() >= defBudget) break;
            candidates.insert(p.first);
        }
    }

    struct DefenseScore {
        int node;
        double rank;
        double negCount;
    };
    vector<DefenseScore> scored;
    for (int u : candidates) {
        int negCount = simulateDefenseScore(G, u, givenPos, givenNeg, chosen);
        double distFactor = 0.0;
        auto itDist = negDist.find(u);
        if (itDist != negDist.end()) {
            distFactor = 1.0 / (1.0 + (double)itDist->second);
        }
        double criticality = distFactor * sumOutInfluence(G, u);
        double rank = 0.6 * (-static_cast<double>(negCount)) + 0.4 * criticality;
        scored.push_back({u, rank, static_cast<double>(negCount)});
    }
    sort(scored.begin(), scored.end(), [](const DefenseScore &a, const DefenseScore &b){
        if (fabs(a.rank - b.rank) < 1e-9) return a.node < b.node;
        return a.rank > b.rank;
    });

    unsigned int added = 0;
    for (auto &entry : scored) {
        if (added >= defBudget) break;
        if (chosen.insert(entry.node).second) {
            ++added;
        }
    }
}

static void ensureMultiComponentCoverage(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight)
{
    if (seeds.size() >= totalBudget) return;
    auto components = computeConnectedComponents(G, allNodes);
    if (components.size() <= 1) return;

    unordered_map<int,int> nodeToComp;
    nodeToComp.reserve(allNodes.size());
    for (int idx = 0; idx < static_cast<int>(components.size()); ++idx) {
        for (int node : components[idx]) {
            nodeToComp[node] = idx;
        }
    }

    vector<int> seedCount(components.size(), 0);
    vector<int> negCount(components.size(), 0);
    if (givenPosSeed >= 0) {
        auto it = nodeToComp.find(givenPosSeed);
        if (it != nodeToComp.end()) seedCount[it->second]++;
    }
    for (int seed : seeds) {
        auto it = nodeToComp.find(seed);
        if (it != nodeToComp.end()) seedCount[it->second]++;
    }
    for (int neg : givenNeg) {
        auto it = nodeToComp.find(neg);
        if (it != nodeToComp.end()) negCount[it->second]++;
    }

    vector<vector<pair<int,double>>> compCandidates(components.size());
    vector<size_t> compCursor(components.size(), 0);
    for (int idx = 0; idx < static_cast<int>(components.size()); ++idx) {
        vector<pair<int,double>> bucket;
        for (int node : components[idx]) {
            if (givenNeg.count(node)) continue;
            if (node == givenPosSeed) continue;
            if (seeds.count(node)) continue;
            auto it = outWeight.find(node);
            if (it == outWeight.end()) continue;
            bucket.emplace_back(node, it->second);
        }
        sort(bucket.begin(), bucket.end(), [](auto &a, auto &b){
            if (fabs(a.second - b.second) < 1e-9) return a.first < b.first;
            return a.second > b.second;
        });
        compCandidates[idx] = std::move(bucket);
    }

    auto takeCandidate = [&](int compIdx) -> int {
        auto &bucket = compCandidates[compIdx];
        auto &cursor = compCursor[compIdx];
        while (cursor < bucket.size()) {
            int node = bucket[cursor].first;
            cursor++;
            if (seeds.count(node)) continue;
            return node;
        }
        return -1;
    };

    for (int idx = 0; idx < static_cast<int>(components.size()); ++idx) {
        int desired = 1;
        if (negCount[idx] > 0) {
            desired = min(3, negCount[idx] + 1);
        }
        if (components[idx].size() >= 40) {
            desired = max(desired, 2);
        }
        while (seedCount[idx] < desired && seeds.size() < totalBudget) {
            int node = takeCandidate(idx);
            if (node == -1) break;
            if (seeds.insert(node).second) {
                seedCount[idx]++;
            }
        }
    }
}
static void applyAnchorDefense(
    DirectedGraph &G,
    int givenPosSeed,
    unsigned int anchorBudget,
    unsigned int totalBudget,
    const unordered_set<int> &givenNeg,
    unordered_set<int> &chosen)
{
    if (anchorBudget == 0 || givenPosSeed < 0) return;
    if (chosen.size() >= totalBudget) return;
    unordered_map<int,double> guardScore;
    for (int in : G.getNodeInNeighbors(givenPosSeed)) {
        if (in == givenPosSeed) continue;
        if (givenNeg.count(in)) continue;
        guardScore[in] += max(0.0, G.getEdgeInfluence(in, givenPosSeed));
        for (int upstream : G.getNodeInNeighbors(in)) {
            if (upstream == givenPosSeed) continue;
            if (givenNeg.count(upstream)) continue;
            guardScore[upstream] += 0.5 * max(0.0, G.getEdgeInfluence(upstream, in));
        }
    }
    vector<pair<int,double>> ordered(guardScore.begin(), guardScore.end());
    sort(ordered.begin(), ordered.end(), [](auto &a, auto &b){
        if (fabs(a.second - b.second) < 1e-9) return a.first < b.first;
        return a.second > b.second;
    });
    unsigned int added = 0;
    for (auto &entry : ordered) {
        if (added >= anchorBudget) break;
        if (chosen.insert(entry.first).second) {
            ++added;
            if (chosen.size() >= totalBudget) break;
        }
    }
}

static void finalizeSeedsWithSignedLT(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure,
    bool conservativeFallback)
{
    unordered_set<int> posInit = seeds;
    if (givenPosSeed >= 0) posInit.insert(givenPosSeed);
    unordered_set<int> posOut, negOut;
    runSignedLT(G, posInit, givenNeg, 3, posOut, negOut);
    if (posOut.size() >= negOut.size()) return;

    auto pressureOf = [&](int node) {
        auto it = negPressure.find(node);
        return it != negPressure.end() ? it->second : 0.0;
    };

    double pressureWeight = conservativeFallback ? 1.0 : 0.4;
    int replacement = -1;
    double bestScore = -numeric_limits<double>::infinity();
    for (int u : allNodes) {
        if (seeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        double score = outWeight.at(u) - pressureWeight * pressureOf(u);
        if (score > bestScore) {
            bestScore = score;
            replacement = u;
        }
    }
    if (replacement == -1) return;

    int drop = -1;
    double worstScore = numeric_limits<double>::infinity();
    for (int u : seeds) {
        double score = outWeight.at(u) + pressureWeight * pressureOf(u);
        if (score < worstScore) {
            worstScore = score;
            drop = u;
        }
    }
    if (drop == -1 || drop == replacement) return;
    seeds.erase(drop);
    seeds.insert(replacement);
}

static int evaluateSignedScore(
    DirectedGraph &G,
    const unordered_set<int> &seeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg)
{
    unordered_set<int> posInit = seeds;
    if (givenPosSeed >= 0) posInit.insert(givenPosSeed);
    unordered_set<int> finalPos, finalNeg;
    diffuse_signed_all(&G, posInit, givenNeg, finalPos, finalNeg);
    return static_cast<int>(finalPos.size()) - static_cast<int>(finalNeg.size());
}

static unordered_set<int> runExactGreedy(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg)
{
    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;
    while (seeds.size() < numberOfSeeds) {
        int bestNode = -1;
        int bestScore = numeric_limits<int>::min();
        for (int node : allNodes) {
            if (givenNeg.count(node)) continue;
            if (node == givenPosSeed) continue;
            if (seeds.count(node)) continue;
            unordered_set<int> trial = seeds;
            trial.insert(node);
            int score = evaluateSignedScore(G, trial, givenPosSeed, givenNeg);
            if (score > bestScore) {
                bestScore = score;
                bestNode = node;
            }
        }
        if (bestNode == -1) break;
        seeds.insert(bestNode);
    }
    return seeds;
}

static void hillClimbSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    unordered_set<int> &seeds)
{
    if (seeds.empty() || numberOfSeeds == 0) return;
    bool improved = true;
    int bestScore = evaluateSignedScore(G, seeds, givenPosSeed, givenNeg);
    while (improved) {
        improved = false;
        int replaceOut = -1;
        int replaceIn = -1;
        int candidateScore = bestScore;
        vector<int> currentSeeds(seeds.begin(), seeds.end());
        for (int drop : currentSeeds) {
            for (int node : allNodes) {
                if (givenNeg.count(node)) continue;
                if (node == givenPosSeed) continue;
                if (seeds.count(node) && node != drop) continue;
                if (node == drop) continue;
                unordered_set<int> trial = seeds;
                trial.erase(drop);
                trial.insert(node);
                if (trial.size() > numberOfSeeds) continue;
                int score = evaluateSignedScore(G, trial, givenPosSeed, givenNeg);
                if (score > candidateScore) {
                    candidateScore = score;
                    replaceOut = drop;
                    replaceIn = node;
                }
            }
        }
        if (replaceOut != -1 && replaceIn != -1) {
            seeds.erase(replaceOut);
            seeds.insert(replaceIn);
            bestScore = candidateScore;
            improved = true;
        }
    }
}

static string encodeSeedKey(const unordered_set<int> &seeds)
{
    vector<int> ordered(seeds.begin(), seeds.end());
    sort(ordered.begin(), ordered.end());
    string key;
    key.reserve(ordered.size() * 4);
    for (int node : ordered) {
        key.append(to_string(node));
        key.push_back(',');
    }
    return key;
}

static unordered_set<int> runBeamSearch(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    size_t beamWidth = 32)
{
    struct BeamState {
        unordered_set<int> seeds;
        int score;
    };
    vector<BeamState> beam;
    beam.push_back({{}, evaluateSignedScore(G, {}, givenPosSeed, givenNeg)});
    for (unsigned int step = 0; step < numberOfSeeds; ++step) {
        vector<BeamState> next;
        unordered_set<string> seen;
        seen.reserve(beamWidth * allNodes.size());
        for (const auto &state : beam) {
            for (int node : allNodes) {
                if (state.seeds.count(node)) continue;
                if (givenNeg.count(node)) continue;
                if (node == givenPosSeed) continue;
                unordered_set<int> candidate = state.seeds;
                candidate.insert(node);
                string key = encodeSeedKey(candidate);
                if (!seen.insert(key).second) continue;
                int score = evaluateSignedScore(G, candidate, givenPosSeed, givenNeg);
                next.push_back({std::move(candidate), score});
            }
        }
        if (next.empty()) break;
        sort(next.begin(), next.end(), [](const BeamState &a, const BeamState &b){
            return a.score > b.score;
        });
        if (next.size() > beamWidth) next.resize(beamWidth);
        beam = std::move(next);
    }
    BeamState best = beam.front();
    for (const auto &state : beam) {
        if (state.score > best.score) best = state;
    }
    return best.seeds;
}

static unordered_set<int> runSimulatedAnnealing(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_set<int> &startSeeds)
{
    if (numberOfSeeds == 0) return {};
    vector<int> usable;
    usable.reserve(allNodes.size());
    for (int node : allNodes) {
        if (givenNeg.count(node)) continue;
        if (node == givenPosSeed) continue;
        usable.push_back(node);
    }
    if (usable.empty()) return {};

    uint64_t hash = 1469598103934665603ULL;
    for (int node : allNodes) {
        hash ^= static_cast<uint64_t>(node + 0x9e3779b9);
        hash *= 1099511628211ULL;
    }
    vector<int> sortedNeg(givenNeg.begin(), givenNeg.end());
    sort(sortedNeg.begin(), sortedNeg.end());
    for (int node : sortedNeg) {
        hash ^= static_cast<uint64_t>(node + 0x517cc1b7);
        hash *= 1099511628211ULL;
    }
    hash ^= static_cast<uint64_t>(givenPosSeed + 0x27d4eb2d);
    std::mt19937 rng(static_cast<uint32_t>((hash >> 32) ^ hash));
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    auto fillToBudget = [&](unordered_set<int> &seeds) {
        while (seeds.size() < numberOfSeeds) {
            vector<int> pool;
            pool.reserve(usable.size());
            for (int node : usable) {
                if (!seeds.count(node)) pool.push_back(node);
            }
            if (pool.empty()) break;
            std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
            seeds.insert(pool[pick(rng)]);
        }
        while (seeds.size() > numberOfSeeds) {
            vector<int> current(seeds.begin(), seeds.end());
            std::uniform_int_distribution<size_t> pick(0, current.size() - 1);
            seeds.erase(current[pick(rng)]);
        }
    };

    unordered_set<int> current = startSeeds;
    for (int node : vector<int>(current.begin(), current.end())) {
        if (givenNeg.count(node) || node == givenPosSeed) {
            current.erase(node);
        }
    }
    fillToBudget(current);
    int currentScore = evaluateSignedScore(G, current, givenPosSeed, givenNeg);
    unordered_set<int> best = current;
    int bestScore = currentScore;

    double temperature = 1.0;
    const int maxIterations = 2500;
    for (int iter = 0; iter < maxIterations; ++iter) {
        unordered_set<int> candidate = current;
        int swapCount = (iter % 7 == 0 && numberOfSeeds > 3) ? 2 : 1;
        for (int s = 0; s < swapCount; ++s) {
            if (candidate.empty()) break;
            vector<int> curVec(candidate.begin(), candidate.end());
            std::uniform_int_distribution<size_t> dropPick(0, curVec.size() - 1);
            int dropNode = curVec[dropPick(rng)];
            candidate.erase(dropNode);
            vector<int> pool;
            pool.reserve(usable.size());
            for (int node : usable) {
                if (!candidate.count(node)) pool.push_back(node);
            }
            if (pool.empty()) {
                candidate.insert(dropNode);
                continue;
            }
            std::uniform_int_distribution<size_t> addPick(0, pool.size() - 1);
            candidate.insert(pool[addPick(rng)]);
        }
        fillToBudget(candidate);
        int candScore = evaluateSignedScore(G, candidate, givenPosSeed, givenNeg);
        if (candScore > bestScore) {
            best = candidate;
            bestScore = candScore;
        }
        double delta = static_cast<double>(candScore - currentScore);
        if (delta >= 0.0 || prob(rng) < exp(delta / max(temperature, 1e-6))) {
            current = candidate;
            currentScore = candScore;
        }
        temperature *= 0.995;
        if (temperature < 0.05) temperature = 0.05;
    }
    return best;
}

static unordered_set<int> runExactCombinationSearch(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &offenseWeights,
    const unordered_set<int> &prioritySeeds)
{
    if (numberOfSeeds == 0) return {};
    const size_t MAX_EXACT_CANDIDATES = 22;

    vector<pair<int,double>> ranked;
    ranked.reserve(offenseWeights.size());
    for (auto &entry : offenseWeights) {
        int node = entry.first;
        if (givenNeg.count(node)) continue;
        if (node == givenPosSeed) continue;
        ranked.push_back(entry);
    }
    sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b){
        if (fabs(a.second - b.second) < 1e-9) return a.first < b.first;
        return a.second > b.second;
    });

    unordered_set<int> added;
    added.reserve(MAX_EXACT_CANDIDATES * 2);
    vector<int> candidates;
    candidates.reserve(MAX_EXACT_CANDIDATES);
    auto tryAdd = [&](int node) {
        if (givenNeg.count(node)) return;
        if (node == givenPosSeed) return;
        if (added.insert(node).second) {
            candidates.push_back(node);
        }
    };

    for (int node : prioritySeeds) {
        tryAdd(node);
    }
    for (auto &entry : ranked) {
        if (candidates.size() >= MAX_EXACT_CANDIDATES) break;
        tryAdd(entry.first);
    }
    if (candidates.size() < numberOfSeeds) {
        for (int node : allNodes) {
            if (candidates.size() >= max(numberOfSeeds, (unsigned int)MAX_EXACT_CANDIDATES)) break;
            tryAdd(node);
        }
    }
    if (candidates.size() > MAX_EXACT_CANDIDATES) {
        candidates.resize(MAX_EXACT_CANDIDATES);
    }
    if (candidates.size() < numberOfSeeds) {
        return {};
    }

    unordered_set<int> bestSeeds;
    int bestScore = numeric_limits<int>::min();
    vector<int> current;
    current.reserve(numberOfSeeds);
    const size_t MAX_COMBINATIONS = 200000;
    size_t evalCount = 0;
    bool aborted = false;

    function<void(size_t)> dfs = [&](size_t idx) {
        if (aborted) return;
        if (current.size() == numberOfSeeds) {
            unordered_set<int> seedSet(current.begin(), current.end());
            int score = evaluateSignedScore(G, seedSet, givenPosSeed, givenNeg);
            if (score > bestScore) {
                bestScore = score;
                bestSeeds = seedSet;
            }
            evalCount++;
            if (evalCount >= MAX_COMBINATIONS) aborted = true;
            return;
        }
        if (idx >= candidates.size()) return;
        size_t remaining = candidates.size() - idx;
        size_t needed = numberOfSeeds - current.size();
        if (remaining < needed) return;

        current.push_back(candidates[idx]);
        dfs(idx + 1);
        current.pop_back();

        dfs(idx + 1);
    };

    dfs(0);
    return bestSeeds;
}

static unordered_set<int> runSmallGraphStrategy(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &outWeight,
    const ParameterSet &baseParam)
{
    unordered_set<int> seeds;
    if (numberOfSeeds == 0 || allNodes.empty()) return seeds;

    SmallGraphStats stats = buildSmallGraphStats(G, allNodes, givenNeg, givenPosSeed, outWeight);

    ParameterSet params = baseParam;
    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int r = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / params.defense_budget_divisor;
        defBudget = min(params.max_defense_seeds, r);
        defBudget = min(defBudget, numberOfSeeds);
    }

    StrategyConfig strategy = deriveSmallStrategy(stats, defBudget, !givenNeg.empty(), givenPosSeed);
    params.overlap_penalty = strategy.overlapPenalty;

    unordered_set<int> offenseMask = buildOffenseMask(G, strategy, givenPosSeed);
    unordered_map<int,double> offenseWeights = buildOffenseWeights(
        allNodes, outWeight, stats, strategy, offenseMask);

    auto slotsLeft = [&]() -> unsigned int {
        size_t used = seeds.size();
        return used >= numberOfSeeds
            ? 0u
            : numberOfSeeds - static_cast<unsigned int>(used);
    };

    if (strategy.anchorDefense > 0) {
        unsigned int anchorQuota = min(strategy.anchorDefense, slotsLeft());
        if (anchorQuota > 0) {
            applyAnchorDefense(G, givenPosSeed, anchorQuota, numberOfSeeds, givenNeg, seeds);
        }
    }

    if (strategy.frontierDefense > 0 && !givenNeg.empty()) {
        unsigned int frontierQuota = min(strategy.frontierDefense, slotsLeft());
        if (frontierQuota > 0) {
            chooseSmallDefense(G, allNodes, givenNeg, givenPosSeed, frontierQuota, seeds);
        }
    }

    if (strategy.maximizeCoverage) {
        ensureMultiComponentCoverage(G, allNodes, givenNeg, givenPosSeed, numberOfSeeds,
                                     seeds, outWeight);
    }

    if (seeds.size() < numberOfSeeds) {
        chooseOffensiveSeeds(G, allNodes, offenseWeights, givenNeg,
                             givenPosSeed, numberOfSeeds, seeds, params,
                             &outWeight, &stats.negPressure, strategy.enablePairSynergy);
    }
    fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds, outWeight,
                      strategy.conservativeFallback ? &stats.negPressure : nullptr,
                      strategy.conservativeFallback);
    finalizeSeedsWithSignedLT(G, allNodes, givenNeg, givenPosSeed, seeds, outWeight,
                              stats.negPressure, strategy.conservativeFallback);

    unordered_set<int> heuristicSeeds = seeds;
    hillClimbSeeds(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, heuristicSeeds);

    unordered_set<int> greedySeeds = runExactGreedy(
        G, allNodes, numberOfSeeds, givenPosSeed, givenNeg);
    hillClimbSeeds(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, greedySeeds);

    unordered_set<int> beamSeeds = runBeamSearch(
        G, allNodes, numberOfSeeds, givenPosSeed, givenNeg);
    hillClimbSeeds(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, beamSeeds);

    unordered_set<int> annealedSeeds = runSimulatedAnnealing(
        G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, beamSeeds);
    hillClimbSeeds(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, annealedSeeds);

    unordered_set<int> prioritySeeds = heuristicSeeds;
    prioritySeeds.insert(greedySeeds.begin(), greedySeeds.end());
    prioritySeeds.insert(beamSeeds.begin(), beamSeeds.end());
    prioritySeeds.insert(annealedSeeds.begin(), annealedSeeds.end());
    unordered_set<int> exactSeeds = runExactCombinationSearch(
        G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, offenseWeights, prioritySeeds);
    if (!exactSeeds.empty()) {
        hillClimbSeeds(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, exactSeeds);
    }

    auto scoreOf = [&](const unordered_set<int> &setSeeds) {
        return evaluateSignedScore(G, setSeeds, givenPosSeed, givenNeg);
    };

    int heurScore = scoreOf(heuristicSeeds);
    int greedyScore = scoreOf(greedySeeds);
    int beamScore = scoreOf(beamSeeds);
    int annealScore = scoreOf(annealedSeeds);
    int exactScore = exactSeeds.empty() ? numeric_limits<int>::min() : scoreOf(exactSeeds);

    unordered_set<int> bestSeeds = heuristicSeeds;
    int bestScore = heurScore;
    if (greedyScore > bestScore) {
        bestScore = greedyScore;
        bestSeeds = greedySeeds;
    }
    if (beamScore > bestScore) {
        bestScore = beamScore;
        bestSeeds = beamSeeds;
    }
    if (annealScore > bestScore) {
        bestScore = annealScore;
        bestSeeds = annealedSeeds;
    }
    if (exactScore > bestScore) {
        bestScore = exactScore;
        bestSeeds = exactSeeds;
    }
    return bestSeeds;
}

// =====================================================================================
//  MASTER seedSelection(): small 使用?�模?�防守�??��?維�? Version2
// =====================================================================================
unordered_set<int> seedSelection(
    DirectedGraph &G,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg)
{
    if (numberOfSeeds == 0) return {};

    vector<int> allNodes = G.getAllNodes();
    if (allNodes.empty()) return {};

    unordered_map<int,double> outWeight;
    computeOutStats(G, allNodes, outWeight);

    return runSmallGraphStrategy(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg,
                                 outWeight, SMALL_BASE_PARAMS);
}
}

namespace medium_strategy {
// =====================================================================================
//  USER PARAMETER 
// =====================================================================================
struct ParameterSet {
    double risk_denom_eps;
    double def_sens_eps;
    double def_hub_priority_step;
    unsigned int max_defense_hubs;
    double overlap_penalty;
    unsigned int max_defense_seeds;
    unsigned int defense_budget_divisor;
    double neg_pressure_weight;
};

static constexpr ParameterSet MEDIUM_PARAMS = { 0.001, 0.005, 0.05, 6, 0.01, 4, 2, 0.18 };

// =====================================================================================
//  BASIC UTILITIES（�? v3�?
// =====================================================================================
static void computeOutStats(
    DirectedGraph &G,
    const vector<int> &nodes,
    unordered_map<int,int> &outDeg,
    unordered_map<int,double> &outWeight)
{
    outDeg.clear();
    outWeight.clear();
    for (int u : nodes) {
        auto outs = G.getNodeOutNeighbors(u);
        outDeg[u] = (int)outs.size();
        double sumW = 0.0;
        for (int v : outs) sumW += G.getEdgeInfluence(u, v);
        outWeight[u] = sumW;
    }
}

struct NegThreatSummary {
    unordered_map<int,double> nodePressure;
    double totalRisk = 0.0;
    double maxHubRisk = 0.0;
    double pressureSum = 0.0;
    size_t hubCount = 0;
    size_t severeHubs = 0;
    size_t moderateHubs = 0;
};

static NegThreatSummary analyzeNegativeThreats(
    DirectedGraph &G,
    const unordered_set<int> &givenNegSeeds,
    const ParameterSet &params)
{
    NegThreatSummary summary;
    for (int h : givenNegSeeds) {
        auto outs = G.getNodeOutNeighbors(h);
        if (outs.empty()) continue;
        double hubRisk = 0.0;
        for (int v : outs) {
            double w = G.getEdgeInfluence(h, v);
            if (w <= 0) continue;
            double denom = fabs(G.getNodeThreshold2(v));
            if (denom < params.risk_denom_eps) denom = params.risk_denom_eps;
            double contrib = w / denom;
            if (contrib <= 0) continue;
            hubRisk += contrib;
            summary.nodePressure[v] += contrib;
            summary.pressureSum += contrib;
        }
        if (hubRisk <= 0) continue;
        summary.hubCount++;
        if (hubRisk >= 3.0) summary.severeHubs++;
        else if (hubRisk >= 1.0) summary.moderateHubs++;
        summary.totalRisk += hubRisk;
        summary.maxHubRisk = max(summary.maxHubRisk, hubRisk);
    }
    return summary;
}

// =====================================================================================
//  Version2 Defensive (medium/large) ??保�?不�?
// =====================================================================================
static void chooseDefensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int defBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params)
{
    if (defBudget == 0 || givenNegSeeds.empty()) return;

    unordered_map<int,double> hubRisk;
    for (int h : givenNegSeeds) {
        auto outs = G.getNodeOutNeighbors(h);
        double r = 0.0;
        for (int v : outs) {
            double w = G.getEdgeInfluence(h, v);
            double neg_th = G.getNodeThreshold2(v);
            double denom = fabs(neg_th);
            if (denom < params.risk_denom_eps) denom = params.risk_denom_eps;
            r += w / denom;
        }
        hubRisk[h] = r;
    }

    vector<pair<int,double>> hubs(hubRisk.begin(), hubRisk.end());
    sort(hubs.begin(), hubs.end(), [](auto&a,auto&b){return a.second>b.second;});
    size_t hubLimit = params.max_defense_hubs;
    if (givenNegSeeds.size() > hubLimit) {
        hubLimit = min(givenNegSeeds.size(), hubLimit + givenNegSeeds.size() / 10 + 1);
    }

    unordered_map<int,double> defScore;
    for (size_t i=0;i<hubs.size() && i<hubLimit;i++){
        int h=hubs[i].first;
        auto outs=G.getNodeOutNeighbors(h);
        for (int v:outs){
            double neg_th=G.getNodeThreshold2(v);
            double sens=1.0/max(params.def_sens_eps,fabs(neg_th));
            for (int u:G.getNodeInNeighbors(v)){
                if(u==givenPosSeed) continue;
                if(givenNegSeeds.count(u)) continue;
                double w_uv=G.getEdgeInfluence(u,v);
                if(w_uv<=0) continue;
                double delta=w_uv*sens*(1.0+params.def_hub_priority_step*i);
                defScore[u]+=delta;
            }
        }
    }

    vector<pair<int,double>> cand(defScore.begin(), defScore.end());
    sort(cand.begin(), cand.end(), [](auto&a,auto&b){return a.second>b.second;});

    unsigned int added = 0;
    for (auto &p : cand) {
        if (added >= defBudget) break;
        int u = p.first;
        if (u == givenPosSeed) continue;
        if (givenNegSeeds.count(u)) continue;
        if (chosenSeeds.insert(u).second) {
            ++added;
        }
    }
}


// =====================================================================================
//  Version2 Offensive（�??��?變�?
// =====================================================================================
static void chooseOffensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params,
    const unordered_map<int,double> *negPressure = nullptr)
{
    if (chosenSeeds.size() >= totalBudget) return;

    unordered_set<int> touched;

    auto initCover = [&](int s){
        for (int v : G.getNodeOutNeighbors(s)) touched.insert(v);
    };
    for (int s : chosenSeeds) initCover(s);
    if (givenPosSeed >= 0) initCover(givenPosSeed);

    vector<int> cand;
    for (int u : allNodes) {
        if (chosenSeeds.count(u)) continue;
        if (givenNegSeeds.count(u)) continue;
        if (u == givenPosSeed) continue;
        if (outWeight.at(u) <= 0) continue;
        cand.push_back(u);
    }

    while (chosenSeeds.size() < totalBudget && !cand.empty()) {
        int best = -1;
        double bestGain = -1;

        for (int u : cand) {
            if (chosenSeeds.count(u)) continue;
            double gain = 0.0;
            double pressurePenalty = 0.0;
            if (negPressure) {
                auto it = negPressure->find(u);
                if (it != negPressure->end()) pressurePenalty = it->second;
            }
            for (int v : G.getNodeOutNeighbors(u)) {
                double w = G.getEdgeInfluence(u, v);
                if (w <= 0) continue;
                gain += touched.count(v) ? params.overlap_penalty * w : w;
            }
            if (negPressure) gain -= params.neg_pressure_weight * pressurePenalty;
            if (gain > bestGain) {
                bestGain = gain;
                best = u;
            }
        }

        if (best == -1 || bestGain <= 0) break;

        chosenSeeds.insert(best);
        for (int v : G.getNodeOutNeighbors(best)) touched.insert(v);
    }
}

static void fillFallbackSeeds(
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight,
    double negPressureWeight,
    const unordered_map<int,double> *negPressure = nullptr,
    bool preferLowPressure = false)
{
    if (seeds.size() >= totalBudget) return;
    vector<pair<int,double>> extra;
    extra.reserve(allNodes.size());
    for (int u : allNodes) {
        if (seeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        double score = outWeight.at(u);
        if (preferLowPressure && negPressure) {
            auto it = negPressure->find(u);
            if (it != negPressure->end()) {
                score -= negPressureWeight * it->second;
            }
        }
        extra.emplace_back(u, score);
    }
    sort(extra.begin(), extra.end(), [](auto &a, auto &b){ return a.second > b.second; });
    for (auto &p : extra) {
        if (seeds.size() >= totalBudget) break;
        seeds.insert(p.first);
    }
}

static unordered_set<int> runMediumGraphStrategy(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &outWeight,
    const ParameterSet &params)
{
    unordered_set<int> seeds;
    if (numberOfSeeds == 0 || allNodes.empty()) return seeds;

    NegThreatSummary negSummary = analyzeNegativeThreats(G, givenNeg, params);
    const auto &negPressure = negSummary.nodePressure;
    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int nominal = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / max(1u, params.defense_budget_divisor);
        nominal = min(params.max_defense_seeds, nominal);
        double coverageRatio = allNodes.empty() ? 0.0 :
            static_cast<double>(negPressure.size()) / static_cast<double>(allNodes.size());
        double avgHubRisk = (negSummary.hubCount > 0)
            ? (negSummary.totalRisk / static_cast<double>(negSummary.hubCount))
            : 0.0;
        double severity = max(negSummary.maxHubRisk, avgHubRisk + coverageRatio * 3.0);

        defBudget = nominal;

        if (negSummary.severeHubs >= 2 || severity >= 5.0) {
            if (defBudget == 0) {
                defBudget = min(2u, params.max_defense_seeds);
            }
        } else if (negSummary.severeHubs == 1 || negSummary.moderateHubs >= 3 || coverageRatio > 0.25) {
            if (defBudget > 2) {
                defBudget -= 1;
            } else if (defBudget == 0) {
                defBudget = 1;
            }
        } else if (negSummary.moderateHubs > 0 || coverageRatio > 0.12 || severity >= 1.0) {
            if (defBudget > 1) {
                defBudget = 1;
            } else if (defBudget == 0) {
                defBudget = (givenNeg.size() >= 2) ? 1u : 0u;
            }
        }
        if (givenNeg.size() >= 2 && defBudget == 0) {
            defBudget = 1;
        }
        defBudget = min(defBudget, min(numberOfSeeds, params.max_defense_seeds));
    }

    if (defBudget > 0) {
        chooseDefensiveSeeds(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds, params);
    }
    if (seeds.size() < numberOfSeeds) {
        chooseOffensiveSeeds(G, allNodes, outWeight, givenNeg,
                             givenPosSeed, numberOfSeeds, seeds, params, &negPressure);
    }
    fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds,
                      outWeight, params.neg_pressure_weight, &negPressure, true);
    return seeds;
}

// =====================================================================================
//  MASTER seedSelection(): small 使用?�模?�防守�??��?維�? Version2
// =====================================================================================
unordered_set<int> seedSelection(
    DirectedGraph &G,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg)
{
    unordered_set<int> empty;
    if (numberOfSeeds == 0) return empty;

    vector<int> allNodes = G.getAllNodes();
    if (allNodes.empty()) return empty;

    unordered_map<int,int> outDeg;
    unordered_map<int,double> outWeight;
    computeOutStats(G, allNodes, outDeg, outWeight);

    return runMediumGraphStrategy(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, MEDIUM_PARAMS);
}
}

namespace large_strategy {
// =====================================================================================
//  USER PARAMETER 
// =====================================================================================
struct ParameterSet {
    double risk_denom_eps;
    double def_sens_eps;
    double def_hub_priority_step;
    unsigned int max_defense_hubs;
    double overlap_penalty;
    unsigned int max_defense_seeds;
    unsigned int defense_budget_divisor;
};

struct ParameterProfile {
    size_t max_nodes;
    ParameterSet params;
};

static constexpr ParameterProfile PARAMETER_PROFILES[] = {
    { SIZE_MAX, { 0.001, 0.001, 0.15, 4, 0.4, 1, 2 } } // large fixed
};

// =====================================================================================
//  PARAMETER PROFILE (single large graph configuration)
// =====================================================================================
static const ParameterSet& getProfileForSize(size_t nodeCount) {
    for (const auto &profile : PARAMETER_PROFILES) {
        if (nodeCount <= profile.max_nodes) {
            return profile.params;
        }
    }
    static const ParameterSet fallback = PARAMETER_PROFILES[sizeof(PARAMETER_PROFILES) / sizeof(PARAMETER_PROFILES[0]) - 1].params;
    return fallback;
}

// =====================================================================================
//  BASIC UTILITIES（�? v3�?
// =====================================================================================
static void computeOutStats(
    DirectedGraph &G,
    const vector<int> &nodes,
    unordered_map<int,int> &outDeg,
    unordered_map<int,double> &outWeight)
{
    outDeg.clear();
    outWeight.clear();
    for (int u : nodes) {
        auto outs = G.getNodeOutNeighbors(u);
        outDeg[u] = (int)outs.size();
        double sumW = 0.0;
        for (int v : outs) sumW += G.getEdgeInfluence(u, v);
        outWeight[u] = sumW;
    }
}

static void computeNegPressure(
    DirectedGraph &G,
    const unordered_set<int> &givenNeg,
    unordered_map<int,double> &negPressure)
{
    negPressure.clear();
    if (givenNeg.empty()) return;
    for (int neg : givenNeg) {
        for (int v : G.getNodeOutNeighbors(neg)) {
            double w = G.getEdgeInfluence(neg, v);
            if (w <= 0) continue;
            negPressure[v] += w;
        }
    }
}

// =====================================================================================
//  SIGNED LT for small deep simulation
// =====================================================================================
static void runSignedLT(
    DirectedGraph &G,
    const unordered_set<int> &posInit,
    const unordered_set<int> &negInit,
    int rounds,
    unordered_set<int> &posOut,
    unordered_set<int> &negOut)
{
    posOut = posInit;
    negOut = negInit;

    for (int t = 0; t < rounds; t++) {
        bool changed = false;
        unordered_set<int> np = posOut;
        unordered_set<int> nn = negOut;

        for (int v : G.getAllNodes()) {
            if (posOut.count(v) || negOut.count(v)) continue;

            double posInf = 0.0, negInf = 0.0;
            for (int u : G.getNodeInNeighbors(v)) {
                double w = G.getEdgeInfluence(u, v);
                if (posOut.count(u)) posInf += w;
                if (negOut.count(u)) negInf += w;
            }

            double thPos = G.getNodeThreshold(v);
            double thNeg = fabs(G.getNodeThreshold2(v));

            if (posInf >= thPos && posInf - negInf > 0) {
                np.insert(v);
                changed = true;
            } else if (negInf >= thNeg && negInf - posInf > 0) {
                nn.insert(v);
                changed = true;
            }
        }

        posOut = np;
        negOut = nn;
        if (!changed) break;
    }
}

static double evaluateSeedSet(
    DirectedGraph &G,
    const unordered_set<int> &studentSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    int rounds);

static void collectTopOutCandidates(
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    const unordered_set<int> &currentSeeds,
    size_t limit,
    vector<int> &out)
{
    vector<pair<double,int>> ranked;
    ranked.reserve(allNodes.size());
    for (int u : allNodes) {
        if (currentSeeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        auto it = outWeight.find(u);
        if (it == outWeight.end()) continue;
        if (it->second <= 0) continue;
        ranked.emplace_back(it->second, u);
    }
    sort(ranked.begin(), ranked.end(), [](auto &a, auto &b){ return a.first > b.first; });
    out.clear();
    for (size_t i = 0; i < ranked.size() && i < limit; ++i) {
        out.push_back(ranked[i].second);
    }
}

static void hillClimbSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unordered_set<int> &seeds,
    double &bestScore,
    unsigned int maxIterations = 2,
    size_t candidateLimit = 25)
{
    if (seeds.empty()) return;
    for (unsigned int iter = 0; iter < maxIterations; ++iter) {
        vector<int> candidates;
        collectTopOutCandidates(allNodes, outWeight, givenNeg, givenPosSeed, seeds, candidateLimit, candidates);
        if (candidates.empty()) break;
        vector<int> seedList(seeds.begin(), seeds.end());
        bool improved = false;
        for (int removeSeed : seedList) {
            for (int addSeed : candidates) {
                if (addSeed == removeSeed) continue;
                if (seeds.count(addSeed)) continue;
                unordered_set<int> trial = seeds;
                trial.erase(removeSeed);
                trial.insert(addSeed);
                double score = evaluateSeedSet(G, trial, givenPosSeed, givenNeg, 4);
                if (score > bestScore) {
                    seeds = std::move(trial);
                    bestScore = score;
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
        if (!improved) break;
    }
}
static double evaluateSeedSet(
    DirectedGraph &G,
    const unordered_set<int> &studentSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    int rounds = 4)
{
    unordered_set<int> posInit = studentSeeds;
    if (givenPosSeed >= 0) posInit.insert(givenPosSeed);
    unordered_set<int> posOut, negOut;
    runSignedLT(G, posInit, givenNeg, rounds, posOut, negOut);
    return static_cast<double>(posOut.size()) - static_cast<double>(negOut.size());
}

// =====================================================================================
//  Version2 Defensive (medium/large) ??保�?不�?
// =====================================================================================
static void chooseDefensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int defBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params)
{
    if (defBudget == 0 || givenNegSeeds.empty()) return;

    unordered_map<int,double> hubRisk;
    for (int h : givenNegSeeds) {
        auto outs = G.getNodeOutNeighbors(h);
        double r = 0.0;
        for (int v : outs) {
            double w = G.getEdgeInfluence(h, v);
            double neg_th = G.getNodeThreshold2(v);
            double denom = fabs(neg_th);
            if (denom < params.risk_denom_eps) denom = params.risk_denom_eps;
            r += w / denom;
        }
        hubRisk[h] = r;
    }

    vector<pair<int,double>> hubs(hubRisk.begin(), hubRisk.end());
    sort(hubs.begin(), hubs.end(), [](auto&a,auto&b){return a.second>b.second;});

    unordered_map<int,double> defScore;
    for (size_t i=0;i<hubs.size() && i<params.max_defense_hubs;i++){
        int h=hubs[i].first;
        auto outs=G.getNodeOutNeighbors(h);
        for (int v:outs){
            double neg_th=G.getNodeThreshold2(v);
            double sens=1.0/max(params.def_sens_eps,fabs(neg_th));
            for (int u:G.getNodeInNeighbors(v)){
                if(u==givenPosSeed) continue;
                if(givenNegSeeds.count(u)) continue;
                double w_uv=G.getEdgeInfluence(u,v);
                if(w_uv<=0) continue;
                double delta=w_uv*sens*(1.0+params.def_hub_priority_step*i);
                defScore[u]+=delta;
            }
        }
    }

    vector<pair<int,double>> cand(defScore.begin(), defScore.end());
    sort(cand.begin(), cand.end(), [](auto&a,auto&b){return a.second>b.second;});

    unsigned int added = 0;
    for (auto &p : cand) {
        if (added >= defBudget) break;
        int u = p.first;
        if (u == givenPosSeed) continue;
        if (givenNegSeeds.count(u)) continue;
        if (chosenSeeds.insert(u).second) {
            ++added;
        }
    }
}


// =====================================================================================
//  Version2 Offensive（�??��?變�?
// =====================================================================================
static void chooseOffensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params,
    const unordered_map<int,double> *negPressure = nullptr)
{
    (void)negPressure;
    if (chosenSeeds.size() >= totalBudget) return;

    unordered_set<int> touched;

    auto initCover = [&](int s){
        for (int v : G.getNodeOutNeighbors(s)) touched.insert(v);
    };
    for (int s : chosenSeeds) initCover(s);
    if (givenPosSeed >= 0) initCover(givenPosSeed);

    vector<int> cand;
    for (int u : allNodes) {
        if (chosenSeeds.count(u)) continue;
        if (givenNegSeeds.count(u)) continue;
        if (u == givenPosSeed) continue;
        if (outWeight.at(u) <= 0) continue;
        cand.push_back(u);
    }

    while (chosenSeeds.size() < totalBudget && !cand.empty()) {
        int best = -1;
        double bestGain = -1;

        for (int u : cand) {
            if (chosenSeeds.count(u)) continue;
            double gain = 0.0;
            for (int v : G.getNodeOutNeighbors(u)) {
                double w = G.getEdgeInfluence(u, v);
                if (w <= 0) continue;
                gain += touched.count(v) ? params.overlap_penalty * w : w;
            }
            if (gain > bestGain) {
                bestGain = gain;
                best = u;
            }
        }

        if (best == -1 || bestGain <= 0) break;

        chosenSeeds.insert(best);
        for (int v : G.getNodeOutNeighbors(best)) touched.insert(v);
    }
}


static void fillFallbackSeeds(
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> *negPressure = nullptr,
    bool preferLowPressure = false)
{
    if (seeds.size() >= totalBudget) return;
    vector<pair<int,double>> extra;
    extra.reserve(allNodes.size());
    for (int u : allNodes) {
        if (seeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        double score = outWeight.at(u);
        if (preferLowPressure && negPressure) {
            auto it = negPressure->find(u);
            if (it != negPressure->end()) {
                score -= 0.3 * it->second;
            }
        }
        extra.emplace_back(u, score);
    }
    sort(extra.begin(), extra.end(), [](auto &a, auto &b){ return a.second > b.second; });
    for (auto &p : extra) {
        if (seeds.size() >= totalBudget) break;
        seeds.insert(p.first);
    }
}

static unordered_set<int> runLargeGraphStrategy(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &outWeight,
    const ParameterSet &params)
{
    unordered_set<int> seeds;
    if (numberOfSeeds == 0 || allNodes.empty()) return seeds;

    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int r = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / params.defense_budget_divisor;
        defBudget = min(params.max_defense_seeds, r);
    }

    unordered_map<int,double> negPressure;
    computeNegPressure(G, givenNeg, negPressure);
    const unordered_map<int,double> *negPressurePtr = negPressure.empty() ? nullptr : &negPressure;

    if (defBudget > 0) {
        chooseDefensiveSeeds(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds, params);
    }

    unordered_set<int> greedySeeds = seeds;
    if (greedySeeds.size() < numberOfSeeds) {
        chooseOffensiveSeeds(G, allNodes, outWeight, givenNeg,
                             givenPosSeed, numberOfSeeds, greedySeeds, params, negPressurePtr);
    }
    fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, greedySeeds, outWeight,
                      negPressurePtr, true);

    unordered_set<int> altSeeds = seeds;
    fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, altSeeds, outWeight,
                      nullptr, false);

    double greedyScore = evaluateSeedSet(G, greedySeeds, givenPosSeed, givenNeg);
    double altScore = evaluateSeedSet(G, altSeeds, givenPosSeed, givenNeg);
    unordered_set<int> bestSeeds = greedySeeds;
    double bestScore = greedyScore;
    if (altScore > bestScore) {
        bestScore = altScore;
        bestSeeds = altSeeds;
    }

    hillClimbSeeds(G, allNodes, outWeight, givenNeg, givenPosSeed, bestSeeds, bestScore);
    return bestSeeds;
}

// =====================================================================================
//  MASTER seedSelection(): unified large-graph heuristic
// =====================================================================================
unordered_set<int> seedSelection(
    DirectedGraph &G,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg)
{
    unordered_set<int> empty;
    if (numberOfSeeds == 0) return empty;

    vector<int> allNodes = G.getAllNodes();
    if (allNodes.empty()) return empty;

    size_t N = allNodes.size();
    unordered_map<int,int> outDeg;
    unordered_map<int,double> outWeight;
    computeOutStats(G, allNodes, outDeg, outWeight);

    ParameterSet params = getProfileForSize(N);
    return runLargeGraphStrategy(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, params);
}
}
} // namespace detail

/*
 * seedSelection:
 *   - G:   整張圖
 *   - numberOfSeeds: 需要你選的「額外正向種子數量」
 *                     （系統另外會再加上 1 個 given_pos.txt 裡的 positive seed）
 *
 * 你只需要回傳一個 unordered_set<int>，裡面放 numberOfSeeds 9個positivate seed編號。
 */
unordered_set<int> seedSelection(DirectedGraph& G,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int>& givenNegSeeds) {
    unordered_set<int> seeds;

    /* Put selected users into unordered_set seeds */
    /* Implement your seed selection algorithm below */

    // 你的演算法實作在這裡

    if (numberOfSeeds > 0) {
        vector<int> nodes = G.getAllNodes();
        size_t nodeCount = nodes.size();
        if (nodeCount > 0) {
            constexpr size_t SMALL_NODE_THRESHOLD = 400;
            constexpr size_t MEDIUM_NODE_THRESHOLD = 4000;
            if (nodeCount <= SMALL_NODE_THRESHOLD) {
                seeds = detail::small_strategy::seedSelection(G, numberOfSeeds, givenPosSeed, givenNegSeeds);
            } else if (nodeCount <= MEDIUM_NODE_THRESHOLD) {
                seeds = detail::medium_strategy::seedSelection(G, numberOfSeeds, givenPosSeed, givenNegSeeds);
            } else {
                seeds = detail::large_strategy::seedSelection(G, numberOfSeeds, givenPosSeed, givenNegSeeds);
            }
        }
    }

    return seeds;
}

#endif
