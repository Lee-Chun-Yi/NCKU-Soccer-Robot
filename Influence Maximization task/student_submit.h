#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <tuple>

#include "LT.h"
#include "graph.h"

using namespace std;

static bool isSmallDebugEnabled() {
    static int cached = -1;
    if (cached == -1) {
        const char *env = std::getenv("SMALL_DEBUG");
        cached = (env && env[0] != '\0') ? 1 : 0;
    }
    return cached == 1;
}

static constexpr double TWO_HOP_WEIGHT = 0.4;
static constexpr double NEG_PRESSURE_BONUS = 1.0;
static constexpr double PRESSURE_GUARD_THRESHOLD = 0.35;
static constexpr double NEG_OUT_DEFENSIVE_THRESHOLD = 0.6;

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

struct OffensiveDebugInfo {
    bool enabled = false;
    size_t rankingLimit = 10;
    vector<pair<int,double>> initialRanking;
    vector<pair<int,double>> pickedContribution;
};

static const unordered_map<int,double> EMPTY_NEG_PRESSURE = {};

static constexpr ParameterProfile PARAMETER_PROFILES[] = {
    { 500, { 0.001, 0.0008, 0.12, 12, 0.25, 1, 8 } },            // small baseline (will tune dynamically)
    { 2000, { 0.0008, 0.005, 0.05, 6, 0.01, 4, 2 } },           // medium fixed
    { SIZE_MAX, { 0.001, 0.001, 0.15, 4, 0.4, 1, 2 } }           // large fixed
};

// =====================================================================================
//  SMALL graph tuning (+-10%, +-20%, baseline) = 5 組�???
// =====================================================================================
static vector<double> genVariants(double base) {
    return {
        base * 0.8,
        base * 0.9,
        base,
        base * 1.1,
        base * 1.2
    };
}

static vector<unsigned int> genVariantsInt(unsigned int base) {
    auto clampVal = [](double x) {
        return (unsigned int)max(1.0, floor(x + 0.5));
    };
    return {
        clampVal(base * 0.8),
        clampVal(base * 0.9),
        base,
        clampVal(base * 1.1),
        clampVal(base * 1.2)
    };
}

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

// =====================================================================================
//  Approximate 2-hop spread score (CELF-Lite style)
// =====================================================================================
static unordered_map<int,double> approxSpread(DirectedGraph &G, const vector<int> &nodes) {
    unordered_map<int,double> score;
    score.reserve(nodes.size());
    for (int u : nodes) {
        double acc = 0.0;
        for (int v : G.getNodeOutNeighbors(u)) {
            acc += G.getEdgeInfluence(u, v);
            for (int w : G.getNodeOutNeighbors(v)) {
                acc += 0.5 * G.getEdgeInfluence(v, w);
            }
        }
        score[u] = acc;
    }
    return score;
}

static double simulateSmallScore(
    DirectedGraph &G,
    const unordered_set<int> &posSeeds,
    int givenPosSeed,
    const unordered_set<int> &negSeeds,
    unordered_set<int> *posFinal = nullptr,
    unordered_set<int> *negFinal = nullptr)
{
    unordered_set<int> posInit = posSeeds;
    if (givenPosSeed >= 0) posInit.insert(givenPosSeed);
    unordered_set<int> negInit = negSeeds;

    unordered_set<int> finalPos, finalNeg;
    diffuse_signed_all(&G, std::move(posInit), std::move(negInit), finalPos, finalNeg);

    const size_t totalNodes = G.getAllNodes().size();
    double score = 0.0;
    if (totalNodes > 0) {
        score = (static_cast<double>(finalPos.size()) - static_cast<double>(finalNeg.size()))
            / static_cast<double>(totalNodes);
    }
    if (posFinal) *posFinal = finalPos;
    if (negFinal) *negFinal = finalNeg;
    return score;
}

static double maxNegHubOutWeight(DirectedGraph &G, const unordered_set<int> &givenNeg) {
    double best = 0.0;
    for (int neg : givenNeg) {
        double sum = 0.0;
        for (int v : G.getNodeOutNeighbors(neg)) {
            sum += G.getEdgeInfluence(neg, v);
        }
        if (sum > best) best = sum;
    }
    return best;
}

static double maxNegPressureValue(const unordered_map<int,double> &negPressure) {
    double best = 0.0;
    for (auto &kv : negPressure) {
        if (kv.second > best) best = kv.second;
    }
    return best;
}

static void guardHighPressureTargets(
    const unordered_map<int,double> &negPressure,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int guardBudget,
    const unordered_map<int,double> &outWeight,
    unordered_set<int> &chosenSeeds,
    vector<int> *debugLog = nullptr,
    unordered_map<int,char> *seedSource = nullptr)
{
    if (guardBudget == 0) return;
    vector<tuple<double,double,int>> cand;
    cand.reserve(negPressure.size());
    for (auto &kv : negPressure) {
        if (kv.second <= 0) continue;
        int node = kv.first;
        if (givenNeg.count(node)) continue;
        if (node == givenPosSeed) continue;
        if (chosenSeeds.count(node)) continue;
        auto owIt = outWeight.find(node);
        double out = (owIt != outWeight.end()) ? owIt->second : 0.0;
        cand.emplace_back(kv.second, out, node);
    }
    sort(cand.begin(), cand.end(), [](const auto &a, const auto &b){
        if (get<0>(a) != get<0>(b)) return get<0>(a) > get<0>(b);
        return get<1>(a) > get<1>(b);
    });
    unsigned int added = 0;
    for (auto &entry : cand) {
        if (added >= guardBudget) break;
        int node = get<2>(entry);
        if (chosenSeeds.count(node)) continue;
        chosenSeeds.insert(node);
        if (debugLog) debugLog->push_back(node);
        if (seedSource) (*seedSource)[node] = 'G';
        added++;
    }
}

static double scoreSmallAnchor(
    DirectedGraph &G,
    const unordered_set<int> &posInit,
    const unordered_set<int> &negInit,
    int givenPos)
{
    return simulateSmallScore(G, posInit, givenPos, negInit);
}

// =====================================================================================
//  Small: Anchor-defense seed selection
// =====================================================================================
static void chooseAnchorDefense(
    DirectedGraph &G,
    int givenPos,
    const unordered_set<int> &givenNeg,
    unsigned int want,
    unordered_set<int> &chosen)
{
    if (want == 0 || givenPos < 0) return;

    vector<int> candidates;
    for (int u : G.getNodeInNeighbors(givenPos)) candidates.push_back(u);
    for (int u : G.getNodeOutNeighbors(givenPos)) candidates.push_back(u);
    sort(candidates.begin(), candidates.end());
    candidates.erase(unique(candidates.begin(), candidates.end()), candidates.end());

    vector<pair<int,double>> ranked;
    for (int u : candidates) {
        if (u == givenPos || givenNeg.count(u) || chosen.count(u)) continue;
        unordered_set<int> posInit = {u};
        double s = scoreSmallAnchor(G, posInit, givenNeg, givenPos);
        ranked.push_back({u, s});
    }
    sort(ranked.begin(), ranked.end(), [](auto &a, auto &b){ return a.second > b.second; });

    unsigned int added = 0;
    for (auto &p : ranked) {
        if (added >= want) break;
        if (chosen.count(p.first)) continue;
        chosen.insert(p.first);
        ++added;
    }
}

// =====================================================================================
//  Small: Global defense scoring (retains previous structure)
// =====================================================================================
static void chooseSmallGlobalDefense(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPos,
    unsigned int want,
    unordered_set<int> &chosen)
{
    if (want == 0) return;

    vector<pair<int,double>> negHub;
    for (int h : givenNeg) {
        double w = 0.0;
        for (int v : G.getNodeOutNeighbors(h)) w += G.getEdgeInfluence(h, v);
        negHub.push_back({h, w});
    }
    sort(negHub.begin(), negHub.end(), [](auto &a, auto &b){ return a.second > b.second; });

    unordered_set<int> candidates;
    for (auto &hw : negHub) {
        int h = hw.first;
        for (int v : G.getNodeOutNeighbors(h)) {
            for (int u : G.getNodeInNeighbors(v)) {
                if (u == givenPos || givenNeg.count(u) || chosen.count(u)) continue;
                candidates.insert(u);
            }
        }
    }

    if (candidates.size() < want) {
        vector<pair<int,double>> extra;
        for (int u : allNodes) {
            if (u == givenPos || givenNeg.count(u) || chosen.count(u) || candidates.count(u)) continue;
            double w = 0.0;
            for (int v : G.getNodeOutNeighbors(u)) w += G.getEdgeInfluence(u, v);
            extra.push_back({u, w});
        }
        sort(extra.begin(), extra.end(), [](auto &a, auto &b){ return a.second > b.second; });
        for (auto &p : extra) {
            candidates.insert(p.first);
            if (candidates.size() >= want) break;
        }
    }

    vector<pair<int,double>> scored;
    for (int u : candidates) {
        unordered_set<int> posInit = {u};
        double s = scoreSmallAnchor(G, posInit, givenNeg, givenPos);
        scored.push_back({u, s});
    }
    sort(scored.begin(), scored.end(), [](auto &a, auto &b){ return a.second > b.second; });

    unsigned int added = 0;
    for (auto &p : scored) {
        if (added >= want) break;
        int u = p.first;
        if (chosen.count(u)) continue;
        chosen.insert(u);
        ++added;
    }
}

// =====================================================================================
//  SMALL DEFENSE: Anchor + Global blend
// =====================================================================================
static void chooseSmallDefense(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPos,
    unsigned int defBudget,
    unordered_set<int> &chosen)
{
    if (defBudget == 0) return;

    size_t initialSize = chosen.size();
    unsigned int anchorBudget = (givenPos >= 0) ? min(defBudget, 2u) : 0u;
    if (anchorBudget > 0) {
        chooseAnchorDefense(G, givenPos, givenNeg, anchorBudget, chosen);
    }
    size_t used = chosen.size() - initialSize;
    unsigned int remaining = defBudget > used ? static_cast<unsigned int>(defBudget - used) : 0u;
    if (remaining > 0) {
        chooseSmallGlobalDefense(G, allNodes, givenNeg, givenPos, remaining, chosen);
    }
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

    for (auto &p:cand){
        if (chosenSeeds.size()>=defBudget) break;
        int u=p.first;
        if (u==givenPosSeed) continue;
        if (givenNegSeeds.count(u)) continue;
        chosenSeeds.insert(u);
    }
}


// =====================================================================================
//  Version2 Offensive（�??��?變�?
// =====================================================================================
static void chooseOffensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &approx,
    const unordered_map<int,double> &negPressure,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params,
    OffensiveDebugInfo *debugInfo = nullptr)
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

    auto computeHeuristic = [&](int u) -> double {
        double twoHop = 0.0;
        auto it = approx.find(u);
        if (it != approx.end()) twoHop = it->second;
        double gain = 0.0;
        for (int v : G.getNodeOutNeighbors(u)) {
            double w = G.getEdgeInfluence(u, v);
            if (w <= 0) continue;
            gain += touched.count(v) ? params.overlap_penalty * w : (w + TWO_HOP_WEIGHT * twoHop);
        }
        auto negIt = negPressure.find(u);
        if (negIt != negPressure.end()) {
            gain += NEG_PRESSURE_BONUS * negIt->second;
        }
        return gain;
    };

    auto simulateScore = [&](const unordered_set<int> &seedSet) -> double {
        return simulateSmallScore(G, seedSet, givenPosSeed, givenNegSeeds);
    };

    double currentScore = simulateScore(chosenSeeds);
    bool storedRanking = false;
    if (debugInfo && debugInfo->enabled) {
        debugInfo->pickedContribution.clear();
        debugInfo->initialRanking.clear();
    }

    while (chosenSeeds.size() < totalBudget && !cand.empty()) {
        vector<pair<int,double>> heurList;
        heurList.reserve(cand.size());
        for (int u : cand) {
            if (chosenSeeds.count(u)) continue;
            double h = computeHeuristic(u);
            heurList.emplace_back(u, h);
        }
        if (heurList.empty()) break;
        sort(heurList.begin(), heurList.end(),
             [](const auto &a, const auto &b){ return a.second > b.second; });
        size_t limit = min<size_t>(heurList.size(), 40);

        vector<pair<int,double>> deltaList;
        deltaList.reserve(limit);

        int bestNode = -1;
        double bestDelta = -1e100;
        double bestScore = currentScore;

        for (size_t i = 0; i < limit; ++i) {
            int u = heurList[i].first;
            unordered_set<int> temp = chosenSeeds;
            temp.insert(u);
            double newScore = simulateScore(temp);
            double delta = newScore - currentScore;
            deltaList.emplace_back(u, delta);
            if (delta > bestDelta + 1e-9) {
                bestDelta = delta;
                bestNode = u;
                bestScore = newScore;
            }
        }

        if (debugInfo && debugInfo->enabled && !storedRanking) {
            auto ranked = deltaList;
            sort(ranked.begin(), ranked.end(),
                 [](const auto &a, const auto &b){ return a.second > b.second; });
            if (ranked.size() > debugInfo->rankingLimit) {
                ranked.resize(debugInfo->rankingLimit);
            }
            debugInfo->initialRanking = ranked;
            storedRanking = true;
        }

        if (bestNode == -1 || bestDelta <= 1e-9) break;

        chosenSeeds.insert(bestNode);
        if (debugInfo && debugInfo->enabled) {
            debugInfo->pickedContribution.emplace_back(bestNode, bestDelta);
        }
        for (int v : G.getNodeOutNeighbors(bestNode)) touched.insert(v);
        currentScore = bestScore;
    }
}

// =====================================================================================
//  MEDIUM: Negative pressure + community helpers
// =====================================================================================
static unordered_map<int,double> computeNegPressure(
    DirectedGraph &G,
    const unordered_set<int> &givenNegSeeds,
    int maxDepth)
{
    unordered_map<int,double> pressure;
    if (givenNegSeeds.empty() || maxDepth <= 0) return pressure;

    for (int neg : givenNegSeeds) {
        unordered_map<int,int> bestDepth;
        queue<int> q;
        q.push(neg);
        bestDepth[neg] = 0;

        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            int depth = bestDepth[cur];
            if (depth >= maxDepth) continue;

            for (int nxt : G.getNodeOutNeighbors(cur)) {
                int nextDepth = depth + 1;
                if (nextDepth > maxDepth) continue;

                double decay = 1.0 / static_cast<double>(nextDepth);
                double w = G.getEdgeInfluence(cur, nxt);
                pressure[nxt] += w * decay;

                auto it = bestDepth.find(nxt);
                if (it == bestDepth.end() || nextDepth < it->second) {
                    bestDepth[nxt] = nextDepth;
                    q.push(nxt);
                }
            }
        }
    }

    return pressure;
}

static unordered_map<int,int> computeCommunityLabels(
    DirectedGraph &G,
    const vector<int> &allNodes)
{
    unordered_map<int,int> component;
    unordered_set<int> visited;
    int compId = 0;

    for (int start : allNodes) {
        if (visited.count(start)) continue;
        queue<int> q;
        q.push(start);
        visited.insert(start);

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            component[u] = compId;

            vector<int> neighbors = G.getNodeOutNeighbors(u);
            auto inNeigh = G.getNodeInNeighbors(u);
            neighbors.insert(neighbors.end(), inNeigh.begin(), inNeigh.end());

            for (int v : neighbors) {
                if (visited.insert(v).second) {
                    q.push(v);
                }
            }
        }
        ++compId;
    }

    return component;
}

static unordered_set<int> buildAnchorNeighborhood(
    DirectedGraph &G,
    int anchor)
{
    unordered_set<int> neighbors;
    if (anchor < 0) return neighbors;
    neighbors.insert(anchor);
    for (int v : G.getNodeOutNeighbors(anchor)) neighbors.insert(v);
    for (int v : G.getNodeInNeighbors(anchor)) neighbors.insert(v);
    return neighbors;
}

static double getNegPressureValue(
    const unordered_map<int,double> &pressure,
    int node)
{
    auto it = pressure.find(node);
    return (it == pressure.end()) ? 0.0 : it->second;
}

static vector<pair<int,double>> rankOffenseCandidates(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure)
{
    vector<pair<int,double>> ranked;
    ranked.reserve(allNodes.size());

    for (int u : allNodes) {
        if (u == givenPosSeed) continue;
        if (givenNegSeeds.count(u)) continue;
        auto it = outWeight.find(u);
        if (it == outWeight.end()) continue;
        double base = it->second - getNegPressureValue(negPressure, u);
        if (base <= 0) continue;
        ranked.emplace_back(u, base);
    }

    sort(ranked.begin(), ranked.end(),
         [](const pair<int,double> &a, const pair<int,double> &b){
             return a.second > b.second;
         });
    return ranked;
}

static void blockStrongNegativeHubs(
    DirectedGraph &G,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int defenseBudget,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const unordered_map<int,double> &outWeight,
    unsigned int maxHubs)
{
    if (defenseBudget == 0 || totalBudget == 0 || givenNegSeeds.empty()) return;
    if (chosenSeeds.size() >= totalBudget) return;

    vector<pair<int,double>> negHub;
    for (int h : givenNegSeeds) {
        double w = 0.0;
        for (int v : G.getNodeOutNeighbors(h)) w += G.getEdgeInfluence(h, v);
        negHub.push_back({h, w});
    }
    sort(negHub.begin(), negHub.end(), [](auto &a, auto &b){ return a.second > b.second; });

    unordered_map<int,double> candidateScore;
    size_t limit = min<size_t>(maxHubs, negHub.size());
    for (size_t idx = 0; idx < limit; ++idx) {
        int hub = negHub[idx].first;
        for (int v : G.getNodeOutNeighbors(hub)) {
            if (v == givenPosSeed) continue;
            if (givenNegSeeds.count(v)) continue;
            if (chosenSeeds.count(v)) continue;
            double edgeW = G.getEdgeInfluence(hub, v);
            double score = edgeW;
            auto it = outWeight.find(v);
            if (it != outWeight.end()) score += 0.35 * it->second;
            candidateScore[v] = max(candidateScore[v], score);
        }
    }

    vector<pair<int,double>> ranked(candidateScore.begin(), candidateScore.end());
    sort(ranked.begin(), ranked.end(), [](auto &a, auto &b){ return a.second > b.second; });

    for (auto &p : ranked) {
        if (chosenSeeds.size() >= totalBudget) break;
        if (defenseBudget == 0) break;
        int node = p.first;
        if (chosenSeeds.count(node)) continue;
        chosenSeeds.insert(node);
        --defenseBudget;
    }
}

static bool isAnchorThreatened(
    DirectedGraph &G,
    int anchor,
    const unordered_set<int> &givenNegSeeds,
    unsigned int strongHubCount,
    int hopLimit,
    const unordered_set<int> &anchorNeighborhood,
    double weightThreshold)
{
    if (anchor < 0 || givenNegSeeds.empty()) return false;
    if (anchorNeighborhood.empty()) return false;

    double totalThreatWeight = 0.0;

    vector<pair<int,double>> negHub;
    for (int h : givenNegSeeds) {
        double w = 0.0;
        for (int v : G.getNodeOutNeighbors(h)) w += G.getEdgeInfluence(h, v);
        negHub.push_back({h, w});
    }
    sort(negHub.begin(), negHub.end(), [](auto &a, auto &b){ return a.second > b.second; });

    strongHubCount = static_cast<unsigned int>(min<size_t>(strongHubCount, negHub.size()));
    for (unsigned int i = 0; i < strongHubCount; ++i) {
        int hub = negHub[i].first;
        queue<pair<int,int>> q;
        unordered_set<int> visited;
        q.push({hub, 0});
        visited.insert(hub);

        while (!q.empty()) {
            auto [node, depth] = q.front();
            q.pop();
            if (depth >= hopLimit) continue;
            for (int nxt : G.getNodeOutNeighbors(node)) {
                int nextDepth = depth + 1;
                if (nextDepth > hopLimit) continue;
                double w = G.getEdgeInfluence(node, nxt);
                if (nextDepth <= 1 && nxt == anchor) {
                    return true;
                }
                if (anchorNeighborhood.count(nxt)) {
                    totalThreatWeight += w;
                }
                if (visited.insert(nxt).second) {
                    q.push({nxt, nextDepth});
                }
            }
        }
    }
    return totalThreatWeight >= weightThreshold;
}

static bool assignAnchorBodyguard(
    DirectedGraph &G,
    int anchor,
    const unordered_set<int> &givenNegSeeds,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure,
    const unordered_set<int> &anchorNeighborhood)
{
    if (anchor < 0) return false;
    if (chosenSeeds.size() >= totalBudget) return false;

    vector<int> neighbors;
    neighbors.reserve(anchorNeighborhood.size());
    for (int v : anchorNeighborhood) {
        if (v != anchor) neighbors.push_back(v);
    }
    sort(neighbors.begin(), neighbors.end());

    vector<pair<int,double>> ranked;
    for (int u : neighbors) {
        if (u == anchor) continue;
        if (givenNegSeeds.count(u)) continue;
        if (chosenSeeds.count(u)) continue;
        double weight = 0.0;
        auto it = outWeight.find(u);
        if (it != outWeight.end()) weight = it->second;
        double pressure = getNegPressureValue(negPressure, u);
        double directEdge = G.getEdgeInfluence(u, anchor) + G.getEdgeInfluence(anchor, u);
        double score = max(0.0, weight - pressure) + 0.5 * directEdge;
        ranked.push_back({u, score});
    }
    sort(ranked.begin(), ranked.end(), [](auto &a, auto &b){ return a.second > b.second; });

    if (!ranked.empty()) {
        chosenSeeds.insert(ranked.front().first);
        return true;
    }
    return false;
}

static void selectCommunityOffense(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure,
    const unordered_map<int,int> &componentOf,
    int anchorCommunity,
    bool ensureAnchorPresence,
    double communityPenaltyScale,
    double coverageWeight)
{
    if (chosenSeeds.size() >= totalBudget) return;

    unordered_set<int> touched;
    auto addTouched = [&](int s){
        for (int v : G.getNodeOutNeighbors(s)) {
            touched.insert(v);
        }
    };
    for (int s : chosenSeeds) addTouched(s);
    if (givenPosSeed >= 0) addTouched(givenPosSeed);

    unordered_map<int,int> componentUsage;
    for (int s : chosenSeeds) {
        auto it = componentOf.find(s);
        if (it != componentOf.end()) componentUsage[it->second]++;
    }

    auto getComp = [&](int node) {
        auto it = componentOf.find(node);
        return (it == componentOf.end()) ? -1 : it->second;
    };

    while (chosenSeeds.size() < totalBudget) {
        bool needAnchor = ensureAnchorPresence && anchorCommunity >= 0
            && componentUsage[anchorCommunity] == 0;

        int bestNode = -1;
        double bestScore = -1.0;

        for (int u : allNodes) {
            if (chosenSeeds.count(u)) continue;
            if (givenNegSeeds.count(u)) continue;
            if (u == givenPosSeed) continue;

            auto it = outWeight.find(u);
            if (it == outWeight.end()) continue;
            double baseScore = it->second - getNegPressureValue(negPressure, u);
            if (baseScore <= 0) continue;

            int compId = getComp(u);
            if (needAnchor && compId != anchorCommunity) continue;

            double compPenalty = 1.0;
            if (communityPenaltyScale > 0) {
                compPenalty = 1.0 / (1.0 + communityPenaltyScale * componentUsage[compId]);
            }

            double coverageGain = 0.0;
            for (int v : G.getNodeOutNeighbors(u)) {
                if (!touched.count(v)) coverageGain += 1.0;
            }

            double score = baseScore * compPenalty + coverageWeight * coverageGain;
            if (score > bestScore) {
                bestScore = score;
                bestNode = u;
            }
        }

        if (bestNode == -1) break;

        chosenSeeds.insert(bestNode);
        int compId = getComp(bestNode);
        componentUsage[compId]++;
        addTouched(bestNode);
    }
}

static void strategyMedium177(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int totalBudget,
    int givenPosSeed,
    const unordered_set<int> &givenNegSeeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure,
    const unordered_map<int,int> &componentOf,
    unordered_set<int> &chosenSeeds)
{
    if (totalBudget == 0) return;
    unsigned int defenseBudget = (givenNegSeeds.empty())
        ? 0u
        : min(2u, totalBudget);
    blockStrongNegativeHubs(G, givenNegSeeds, givenPosSeed, defenseBudget,
                            totalBudget, chosenSeeds, outWeight, 2);

    int anchorCommunity = -1;
    auto it = componentOf.find(givenPosSeed);
    if (it != componentOf.end()) anchorCommunity = it->second;

    selectCommunityOffense(G, allNodes, givenNegSeeds, givenPosSeed,
                           totalBudget, chosenSeeds, outWeight, negPressure,
                           componentOf, anchorCommunity, false, 1.5, 0.15);
}

static void strategyMedium807(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int totalBudget,
    int givenPosSeed,
    const unordered_set<int> &givenNegSeeds,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &negPressure,
    const unordered_map<int,int> &componentOf,
    unordered_set<int> &chosenSeeds)
{
    if (totalBudget == 0) return;

    auto anchorNeighborhood = buildAnchorNeighborhood(G, givenPosSeed);
    auto baseRanking = rankOffenseCandidates(G, allNodes, givenNegSeeds,
                                             givenPosSeed, outWeight, negPressure);

    const double THREAT_WEIGHT_THRESHOLD = 0.45;
    bool guardUsed = false;
    bool needGuard = isAnchorThreatened(G, givenPosSeed, givenNegSeeds,
                                        3, 2, anchorNeighborhood,
                                        THREAT_WEIGHT_THRESHOLD);
    if (needGuard && chosenSeeds.size() < totalBudget) {
        const size_t TOP_K_GUARD = 5;
        size_t limit = min(TOP_K_GUARD, baseRanking.size());
        for (size_t i = 0; i < limit && !guardUsed; ++i) {
            int candidate = baseRanking[i].first;
            if (candidate == givenPosSeed) continue;
            if (!anchorNeighborhood.count(candidate)) continue;
            if (givenNegSeeds.count(candidate)) continue;
            if (chosenSeeds.count(candidate)) continue;
            chosenSeeds.insert(candidate);
            guardUsed = true;
        }
        if (!guardUsed) {
            guardUsed = assignAnchorBodyguard(G, givenPosSeed, givenNegSeeds,
                                              totalBudget, chosenSeeds, outWeight,
                                              negPressure, anchorNeighborhood);
        }
    }

    if (chosenSeeds.size() < totalBudget && !givenNegSeeds.empty()) {
        unsigned int remaining = static_cast<unsigned int>(totalBudget - chosenSeeds.size());
        unsigned int defenseAllowance = guardUsed ? 2u : 3u;
        unsigned int defenseBudget = min(defenseAllowance, remaining);
        if (defenseBudget > 0) {
            unsigned int hubLimit = guardUsed ? 2u : 3u;
            blockStrongNegativeHubs(G, givenNegSeeds, givenPosSeed, defenseBudget,
                                    totalBudget, chosenSeeds, outWeight, hubLimit);
        }
    }

    int anchorCommunity = -1;
    auto it = componentOf.find(givenPosSeed);
    if (it != componentOf.end()) anchorCommunity = it->second;

    selectCommunityOffense(G, allNodes, givenNegSeeds, givenPosSeed,
                           totalBudget, chosenSeeds, outWeight, negPressure,
                           componentOf, anchorCommunity, true, 1.4, 0.18);
}

static void runGenericDefenseOffense(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNegSeeds,
    const unordered_map<int,double> &outWeight,
    unordered_set<int> &chosenSeeds,
    const ParameterSet &params)
{
    if (numberOfSeeds == 0) return;
    unordered_map<int,double> approxScore;
    bool approxReady = false;

    unsigned int defBudget = 0;
    if (!givenNegSeeds.empty()) {
        unsigned int r = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / params.defense_budget_divisor;
        defBudget = min(params.max_defense_seeds, r);
    }

    if (defBudget > 0) {
        chooseDefensiveSeeds(G, allNodes, givenNegSeeds, givenPosSeed,
                             defBudget, chosenSeeds, params);
    }
    if (chosenSeeds.size() < numberOfSeeds) {
        if (!approxReady) {
            approxScore = approxSpread(G, allNodes);
            approxReady = true;
        }
        chooseOffensiveSeeds(G, allNodes, outWeight, approxScore, EMPTY_NEG_PRESSURE,
                             givenNegSeeds, givenPosSeed, numberOfSeeds, chosenSeeds, params);
    }
}

static void fillFallbackSeeds(
    const vector<int> &allNodes,
    const unordered_set<int> &givenNeg,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &seeds,
    const unordered_map<int,double> &outWeight)
{
    if (seeds.size() >= totalBudget) return;
    vector<pair<int,double>> extra;
    extra.reserve(allNodes.size());
    for (int u : allNodes) {
        if (seeds.count(u)) continue;
        if (givenNeg.count(u)) continue;
        if (u == givenPosSeed) continue;
        extra.emplace_back(u, outWeight.at(u));
    }
    sort(extra.begin(), extra.end(), [](auto &a, auto &b){ return a.second > b.second; });
    for (auto &p : extra) {
        if (seeds.size() >= totalBudget) break;
        seeds.insert(p.first);
    }
}

static double evaluateSmallRun(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &approx,
    const unordered_map<int,double> &negPressure,
    const ParameterSet &params)
{
    unordered_set<int> seeds;
    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int r = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / params.defense_budget_divisor;
        defBudget = min(params.max_defense_seeds, r);
        double heavyHub = maxNegHubOutWeight(G, givenNeg);
        if (heavyHub > NEG_OUT_DEFENSIVE_THRESHOLD) {
            defBudget = max(defBudget, 1u);
        }
    }
    if (defBudget > 0) {
        chooseSmallDefense(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds);
    }
    if (!givenNeg.empty() && seeds.size() < numberOfSeeds) {
        double peakPressure = maxNegPressureValue(negPressure);
        if (peakPressure > PRESSURE_GUARD_THRESHOLD) {
            unsigned int remaining = numberOfSeeds - static_cast<unsigned int>(seeds.size());
            unsigned int guardBudget = min(remaining,
                max(1u, min(2u, static_cast<unsigned int>(numberOfSeeds / 4 + 1))));
            guardHighPressureTargets(negPressure, givenNeg, givenPosSeed, guardBudget, outWeight, seeds);
        }
    }
    if (seeds.size() < numberOfSeeds) {
        chooseOffensiveSeeds(G, allNodes, outWeight, approx, negPressure, givenNeg, givenPosSeed, numberOfSeeds, seeds, params);
    }
    fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds, outWeight);

    return scoreSmallAnchor(G, seeds, givenNeg, givenPosSeed);
}

static ParameterSet hillClimbSmallParameters(
    DirectedGraph &G,
    const vector<int> &allNodes,
    unsigned int numberOfSeeds,
    int givenPosSeed,
    const unordered_set<int> &givenNeg,
    const unordered_map<int,double> &outWeight,
    const unordered_map<int,double> &approx,
    const unordered_map<int,double> &negPressure,
    const ParameterSet &baseParam)
{
    const int MAX_ITERS = 10;
    const double EPS = 1e-9;

    ParameterSet current = baseParam;
    double currentScore = evaluateSmallRun(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, approx, negPressure, current);

    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        bool improved = false;
        ParameterSet iterBest = current;
        double iterScore = currentScore;

        auto considerDouble = [&](double ParameterSet::*member){
            double baseVal = current.*member;
            for (double candVal : genVariants(baseVal)) {
                if (fabs(candVal - baseVal) < 1e-12) continue;
                ParameterSet cand = current;
                cand.*member = candVal;
                double score = evaluateSmallRun(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, approx, negPressure, cand);
                if (score > iterScore + EPS) {
                    iterScore = score;
                    iterBest = cand;
                    improved = true;
                }
            }
        };

        auto considerInt = [&](unsigned int ParameterSet::*member){
            unsigned int baseVal = current.*member;
            for (unsigned int candVal : genVariantsInt(baseVal)) {
                if (candVal == baseVal) continue;
                ParameterSet cand = current;
                cand.*member = candVal;
                double score = evaluateSmallRun(G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, approx, negPressure, cand);
                if (score > iterScore + EPS) {
                    iterScore = score;
                    iterBest = cand;
                    improved = true;
                }
            }
        };

        considerDouble(&ParameterSet::risk_denom_eps);
        considerDouble(&ParameterSet::def_sens_eps);
        considerDouble(&ParameterSet::def_hub_priority_step);
        considerInt(&ParameterSet::max_defense_hubs);
        considerDouble(&ParameterSet::overlap_penalty);
        considerInt(&ParameterSet::max_defense_seeds);
        considerInt(&ParameterSet::defense_budget_divisor);

        if (!improved) break;
        current = iterBest;
        currentScore = iterScore;
    }

    return current;
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
    unordered_set<int> seeds;
    if (numberOfSeeds == 0) return seeds;

    vector<int> allNodes = G.getAllNodes();
    if (allNodes.empty()) return seeds;

    size_t N = allNodes.size();
    unordered_map<int,int> outDeg;
    unordered_map<int,double> outWeight;
    computeOutStats(G, allNodes, outDeg, outWeight);

    bool isSmallGraph = (N <= 500);
    bool isMediumGraph = (N > 500 && N <= 2000);

    if (!isSmallGraph) {
        ParameterSet params = getProfileForSize(N);

        if (isMediumGraph) {
            auto communities = computeCommunityLabels(G, allNodes);
            auto negPressure = computeNegPressure(G, givenNeg, 2);

            if (givenPosSeed == 177) {
                strategyMedium177(G, allNodes, numberOfSeeds, givenPosSeed,
                                  givenNeg, outWeight, negPressure, communities, seeds);
            } else if (givenPosSeed == 807) {
                strategyMedium807(G, allNodes, numberOfSeeds, givenPosSeed,
                                  givenNeg, outWeight, negPressure, communities, seeds);
            } else {
                runGenericDefenseOffense(G, allNodes, numberOfSeeds, givenPosSeed,
                                         givenNeg, outWeight, seeds, params);
            }
        } else {
            runGenericDefenseOffense(G, allNodes, numberOfSeeds, givenPosSeed,
                                     givenNeg, outWeight, seeds, params);
        }

        fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds, outWeight);
        return seeds;
    }

    ParameterSet baseParam = getProfileForSize(N);
    auto approxScore = approxSpread(G, allNodes);
    unordered_map<int,double> negPressure;
    if (!givenNeg.empty()) {
        negPressure = computeNegPressure(G, givenNeg, 2);
    }
    ParameterSet bestParam = hillClimbSmallParameters(
        G, allNodes, numberOfSeeds, givenPosSeed, givenNeg, outWeight, approxScore, negPressure, baseParam);

    const bool debugSmall = isSmallDebugEnabled();
    OffensiveDebugInfo offenseDebug;
    if (debugSmall) {
        offenseDebug.enabled = true;
        offenseDebug.rankingLimit = 10;
    }
    vector<int> defenseSeedsDebug;
    vector<int> guardSeedsDebug;
    vector<int> offenseSeedsDebug;
    vector<int> fallbackSeedsDebug;
    unordered_map<int,char> seedSource;

    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int r = (bestParam.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / bestParam.defense_budget_divisor;
        defBudget = min(bestParam.max_defense_seeds, r);
        double heavyHub = maxNegHubOutWeight(G, givenNeg);
        if (heavyHub > NEG_OUT_DEFENSIVE_THRESHOLD) {
            defBudget = max(defBudget, 1u);
        }
    }

    if (defBudget > 0) {
        unordered_set<int> before = seeds;
        chooseSmallDefense(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds);
        if (debugSmall) {
            for (int x : seeds) if (!before.count(x)) {
                defenseSeedsDebug.push_back(x);
                seedSource[x] = 'D';
            }
        }
    }
    if (!givenNeg.empty() && seeds.size() < numberOfSeeds) {
        double peakPressure = maxNegPressureValue(negPressure);
        if (peakPressure > PRESSURE_GUARD_THRESHOLD) {
            unsigned int remaining = numberOfSeeds - static_cast<unsigned int>(seeds.size());
            unsigned int guardBudget = min(remaining,
                max(1u, min(2u, static_cast<unsigned int>(numberOfSeeds / 4 + 1))));
            guardHighPressureTargets(negPressure, givenNeg, givenPosSeed,
                                     guardBudget, outWeight, seeds,
                                     debugSmall ? &guardSeedsDebug : nullptr,
                                     debugSmall ? &seedSource : nullptr);
        }
    }
    if (seeds.size() < numberOfSeeds) {
        unordered_set<int> before = seeds;
        chooseOffensiveSeeds(G, allNodes, outWeight, approxScore, negPressure, givenNeg,
                             givenPosSeed, numberOfSeeds, seeds, bestParam, debugSmall ? &offenseDebug : nullptr);
        if (debugSmall) {
            for (int x : seeds) if (!before.count(x)) {
                offenseSeedsDebug.push_back(x);
                seedSource[x] = 'O';
            }
        }
    }
    if (seeds.size() < numberOfSeeds) {
        unordered_set<int> before = seeds;
        fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds, outWeight);
        if (debugSmall) {
            for (int x : seeds) if (!before.count(x)) {
                fallbackSeedsDebug.push_back(x);
                seedSource[x] = 'F';
            }
        }
    } else {
        fillFallbackSeeds(allNodes, givenNeg, givenPosSeed, numberOfSeeds, seeds, outWeight);
    }

    if (debugSmall) {
        vector<int> finalSeeds(seeds.begin(), seeds.end());
        sort(finalSeeds.begin(), finalSeeds.end());

        unordered_set<int> posOut, negOut;
        double finalScore = simulateSmallScore(G, seeds, givenPosSeed, givenNeg, &posOut, &negOut);

        unordered_map<int,double> contribMap;
        for (auto &p : offenseDebug.pickedContribution) contribMap[p.first] = p.second;

        cout << "\n===== SMALL DEBUG REPORT =====\n";
        cout << "[第5層] Graph summary: nodes=" << N
             << ", edges=" << G.getEdgeNumber()
             << ", avg out-degree=" << fixed << setprecision(2)
             << (N ? static_cast<double>(G.getEdgeNumber()) / static_cast<double>(N) : 0.0)
             << "\n";

        if (!givenNeg.empty()) {
            cout << "[第4層] NEG hub 壓力分布:\n";
            for (int neg : givenNeg) {
                auto outs = G.getNodeOutNeighbors(neg);
                vector<pair<int,double>> neigh;
                double total = 0.0;
                for (int v : outs) {
                    double w = G.getEdgeInfluence(neg, v);
                    total += w;
                    neigh.emplace_back(v, w);
                }
                sort(neigh.begin(), neigh.end(),
                     [](const auto &a, const auto &b){return a.second > b.second;});
                size_t show = min<size_t>(neigh.size(), 5);
                cout << "  NEG " << neg << " deg=" << outs.size()
                     << " totalOut=" << total << " ->";
                for (size_t i = 0; i < show; ++i) {
                    cout << " (" << neigh[i].first << "," << fixed << setprecision(3) << neigh[i].second << ")";
                }
                cout << "\n";
            }
        } else {
            cout << "[第4層] NEG hub 壓力分布: 無負面種子\n";
        }
        if (!guardSeedsDebug.empty()) {
            cout << "[Guard 層] 防守 seed:";
            for (size_t i = 0; i < guardSeedsDebug.size(); ++i) {
                cout << " " << guardSeedsDebug[i];
            }
            cout << "\n";
        }

        cout << "[第3層] Seed 預期貢獻 (真實 Δscore):\n";
        for (int s : finalSeeds) {
            double val = contribMap.count(s) ? contribMap[s] : 0.0;
            char origin = seedSource.count(s) ? seedSource[s] : '-';
            cout << "  seed " << s << " (" << origin << ") -> gain=" << fixed << setprecision(4) << val << "\n";
        }

        cout << "[第2層] diffuse_signed_all result: pos=" << posOut.size()
             << ", neg=" << negOut.size()
             << ", score=" << fixed << setprecision(4) << finalScore << "\n";

        cout << "[第1層] 最終 small seeds (D=defense, G=guard, O=offense, F=fallback): ";
        for (size_t i = 0; i < finalSeeds.size(); ++i) {
            if (i) cout << ", ";
            cout << finalSeeds[i];
        }
        cout << "\n";

        cout << "最後 9 顆 small seeds: ";
        for (size_t i = 0; i < finalSeeds.size(); ++i) {
            if (i) cout << " ";
            cout << finalSeeds[i];
        }
        cout << "\n";

        cout << "Offense ranking (top 10, 真實 Δscore):";
        if (offenseDebug.initialRanking.empty()) {
            cout << " (none)\n";
        } else {
            cout << "\n";
            for (auto &p : offenseDebug.initialRanking) {
                cout << "  cand " << p.first << " gain~" << fixed << setprecision(4) << p.second << "\n";
            }
        }

        cout << "diffuse_signed_all 終態: pos=" << posOut.size()
             << " nodes, neg=" << negOut.size() << " nodes, score=" << fixed << setprecision(4) << finalScore << "\n";
        cout.unsetf(std::ios::floatfield);
        cout << setprecision(6);
        cout << "================================\n\n";
    }

    return seeds;
}

#endif


