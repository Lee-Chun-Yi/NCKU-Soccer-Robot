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

#include "LT.h"
#include "graph.h"

using namespace std;

// =====================================================================================
//  USER PARAMETER (保持你的 Version2 設計)
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
    { 500, { 0.002, 0.001, 0.05, 6, 0.7, 4, 2 } },        // small graphs
    { 2000, { 0.001, 0.005, 0.05, 6, 0.01, 4, 2 } },        // medium graphs
    { SIZE_MAX, { 0.001, 0.001, 0.15, 4, 0.4, 1, 2 } }       // fallback / large
};

enum { PARAMETER_PROFILE_COUNT = sizeof(PARAMETER_PROFILES)/sizeof(PARAMETER_PROFILES[0]) };

static ParameterSet selectParameterSet(size_t n) {
    for (size_t i = 0; i < PARAMETER_PROFILE_COUNT; i++) {
        if (n <= PARAMETER_PROFILES[i].max_nodes)
            return PARAMETER_PROFILES[i].params;
    }
    return PARAMETER_PROFILES[PARAMETER_PROFILE_COUNT - 1].params;
}

// =====================================================================================
//  BASIC UTILITIES
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

static unordered_map<int,double> computeNegHubRisk(
    DirectedGraph &G,
    const unordered_set<int> &givenNegSeeds,
    const ParameterSet& params)
{
    unordered_map<int,double> risk;
    for (int h : givenNegSeeds) {
        auto outs = G.getNodeOutNeighbors(h);
        if (outs.empty()) continue;
        double r = 0.0;
        for (int v : outs) {
            double w = G.getEdgeInfluence(h, v);
            double neg_th = G.getNodeThreshold2(v);
            double denom = std::fabs(neg_th);
            if (denom < params.risk_denom_eps)
                denom = params.risk_denom_eps;
            r += w / denom;
        }
        if (r > 0.0) risk[h] = r;
    }
    return risk;
}

// =====================================================================================
//  SIGNED LT QUICK SIMULATION (Small graph專用)
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

static int simulateDefenseScore(
    DirectedGraph &G,
    int candidate,
    int givenPos,
    const unordered_set<int> &givenNeg)
{
    unordered_set<int> posInit = { candidate };
    if (givenPos >= 0) posInit.insert(givenPos);

    unordered_set<int> posF, negF;
    runSignedLT(G, posInit, givenNeg, 4, posF, negF);

    return (int)negF.size();
}

// =====================================================================================
//  SMALL GRAPH DEFENSE: 完全替換 Version2 defensive 部分
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

    // 尋找負向 hub（依 out-weight 排序）
    vector<pair<int,double>> negHub;
    for (int h : givenNeg) {
        double w = 0.0;
        for (int v : G.getNodeOutNeighbors(h)) w += G.getEdgeInfluence(h, v);
        negHub.push_back({h, w});
    }
    sort(negHub.begin(), negHub.end(), [](auto &a, auto &b){return a.second > b.second;});

    // 收集防守候選：所有負向 hub 第一層的 in-neighbors
    unordered_set<int> candidates;
    for (auto &hw : negHub) {
        int h = hw.first;
        for (int v : G.getNodeOutNeighbors(h)) {
            for (int u : G.getNodeInNeighbors(v)) {
                if (givenNeg.count(u)) continue;
                if (u == givenPos) continue;
                candidates.insert(u);
            }
        }
    }

    // 若候選太少，加入 outWeight top-k
    if (candidates.size() < defBudget) {
        vector<pair<int,double>> extra;
        for (int u : allNodes) {
            if (givenNeg.count(u) || u == givenPos) continue;
            double w = 0.0;
            for (int v : G.getNodeOutNeighbors(u)) w += G.getEdgeInfluence(u, v);
            extra.push_back({u, w});
        }
        sort(extra.begin(), extra.end(), [](auto&a,auto&b){return a.second>b.second;});
        for (auto &p : extra) {
            if (candidates.size() >= defBudget) break;
            candidates.insert(p.first);
        }
    }

    // 深度模擬：選出對負向最具壓制力的 seed
    vector<pair<int,int>> scored;
    for (int u : candidates) {
        int negCount = simulateDefenseScore(G, u, givenPos, givenNeg);
        scored.push_back({u, negCount});
    }
    sort(scored.begin(), scored.end(), [](auto&a,auto&b){return a.second < b.second;});

    for (auto &p : scored) {
        if (chosen.size() >= defBudget) break;
        chosen.insert(p.first);
    }
}

// =====================================================================================
//  GENERAL DEFENSE FOR MEDIUM/LARGE GRAPHS
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

    unordered_map<int,double> hubRisk =
        computeNegHubRisk(G, givenNegSeeds, params);
    if (hubRisk.empty()) return;

    vector<pair<int,double>> hubs(hubRisk.begin(), hubRisk.end());
    sort(hubs.begin(), hubs.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    unordered_map<int,double> defScore;
    const size_t maxHubs = std::min<size_t>(
        hubs.size(), static_cast<size_t>(params.max_defense_hubs));

    for (size_t i = 0; i < maxHubs; ++i) {
        int h = hubs[i].first;
        double hRisk = hubs[i].second;
        if (hRisk <= 0.0) continue;

        auto outs = G.getNodeOutNeighbors(h);
        if (outs.empty()) continue;

        for (int v : outs) {
            double neg_th = G.getNodeThreshold2(v);
            double sens = 1.0 / std::max(params.def_sens_eps, std::fabs(neg_th));

            auto inNbrs = G.getNodeInNeighbors(v);
            for (int u : inNbrs) {
                if (u == givenPosSeed) continue;
                if (givenNegSeeds.count(u)) continue;
                if (chosenSeeds.count(u)) continue;

                double w_uv = G.getEdgeInfluence(u, v);
                if (w_uv <= 0.0) continue;

                double delta = w_uv * sens *
                    (1.0 + params.def_hub_priority_step * i);
                defScore[u] += delta;
            }
        }
    }

    if (defScore.empty()) return;

    vector<pair<int,double>> cand(defScore.begin(), defScore.end());
    sort(cand.begin(), cand.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });

    for (size_t i = 0; i < cand.size() && chosenSeeds.size() < defBudget; ++i) {
        int u = cand[i].first;
        if (chosenSeeds.count(u)) continue;
        if (givenNegSeeds.count(u)) continue;
        if (u == givenPosSeed) continue;
        chosenSeeds.insert(u);
    }
}

// =====================================================================================
//  Version2 Offensive（保持不變）
// =====================================================================================
static void chooseOffensiveSeeds(
    DirectedGraph &G,
    const vector<int> &allNodes,
    const unordered_map<int,double> &outWeight,
    const unordered_set<int> &givenNegSeeds,
    int givenPosSeed,
    unsigned int totalBudget,
    unordered_set<int> &chosenSeeds,
    const ParameterSet& params)
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

// =====================================================================================
//  MASTER seedSelection(): small 使用全模擬防守，其餘維持 Version2
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
    ParameterSet params = selectParameterSet(N);

    unordered_map<int,int> outDeg;
    unordered_map<int,double> outWeight;
    computeOutStats(G, allNodes, outDeg, outWeight);

    unsigned int defBudget = 0;
    if (!givenNeg.empty()) {
        unsigned int r = (params.defense_budget_divisor == 0)
            ? numberOfSeeds
            : numberOfSeeds / params.defense_budget_divisor;
        defBudget = min(params.max_defense_seeds, r);
    }

    // ==========================================================
    //   SMALL: 使用深度模擬防守
    // ==========================================================
    if (N <= 500) {
        if (defBudget > 0)
            chooseSmallDefense(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds);
    } else {
        // ======================================================
        //   MEDIUM/LARGE: 保留 Version2 原本邏輯
        // ======================================================
        chooseDefensiveSeeds(G, allNodes, givenNeg, givenPosSeed, defBudget, seeds, params);
    }

    // ==========================================================
    //   Offensive phase (共用 Version2)
    // ==========================================================
    if (seeds.size() < numberOfSeeds) {
        chooseOffensiveSeeds(G, allNodes, outWeight, givenNeg,
                             givenPosSeed, numberOfSeeds, seeds, params);
    }

    // ==========================================================
    //   Fallback: 若不足，補強 outWeight 高的點
    // ==========================================================
    if (seeds.size() < numberOfSeeds) {
        vector<pair<int,double>> extra;
        for (int u : allNodes) {
            if (seeds.count(u)) continue;
            if (givenNeg.count(u)) continue;
            if (u == givenPosSeed) continue;
            extra.emplace_back(u, outWeight[u]);
        }
        sort(extra.begin(), extra.end(), [](auto&a,auto&b){return a.second > b.second;});
        for (auto &p : extra) {
            if (seeds.size() >= numberOfSeeds) break;
            seeds.insert(p.first);
        }
    }

    return seeds;
}

#endif
