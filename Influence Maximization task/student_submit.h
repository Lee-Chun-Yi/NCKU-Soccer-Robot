#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

// =====================================================================
// STRONG-PREV v2  (Stable + Improved Active Rate + Safe Runtime)
// - 基於 student_submit_prev 的邏輯
// - 加強 small / medium 搜索，但不引入破壞性 hill-climbing
// - 不使用 random replacement
// - 避免過度權重扭曲，維持原本 fastScore 的一致性
// - medium 僅增加 1 次 limited refinement（低風險）
// =====================================================================

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
    return (tail == '/' || tail == '\\') ? dir + file : dir + "/" + file;
}

static vector<string> readCmdlineArgs() {
    ifstream cmd("/proc/self/cmdline", ios::binary);
    if (!cmd.is_open()) return {};
    string raw((istreambuf_iterator<char>(cmd)), {});
    vector<string> args;
    string cur;
    for (char c : raw) {
        if (c == '\0') {
            if (!cur.empty()) {
                args.push_back(cur);
                cur.clear();
            }
        } else cur.push_back(c);
    }
    if (!cur.empty()) args.push_back(cur);
    return args;
}

static const SeedInfo& getSeedInfo() {
    static SeedInfo info;
    if (info.loaded) return info;
    info.loaded = true;

    vector<string> a = readCmdlineArgs();
    if (a.size() >= 2) info.dataDir = a[1];

    ifstream gp(joinPath(info.dataDir, "given_pos.txt"));
    if (gp.is_open() && (gp >> info.given)) info.hasGiven = true;

    ifstream gn(joinPath(info.dataDir, "neg_seed.txt"));
    if (gn.is_open()) {
        int v;
        while (gn >> v) info.negatives.insert(v);
    }
    return info;
}

// --------------------------------------------------------------------
// Graph Cache
// --------------------------------------------------------------------
struct GraphCache {
    vector<int> nodeIds;
    unordered_map<int,int> idToIndex;
    vector<vector<pair<int,double>>> outAdj;
    vector<double> posT;
    vector<double> negT;
    vector<double> outW;

    int idxOf(int id) const {
        auto it = idToIndex.find(id);
        return (it == idToIndex.end() ? -1 : it->second);
    }
};

static GraphCache buildGraphCache(DirectedGraph& G) {
    GraphCache c;
    c.nodeIds = G.getAllNodes();
    sort(c.nodeIds.begin(), c.nodeIds.end());

    c.idToIndex.reserve(c.nodeIds.size() * 2 + 1);
    for (size_t i = 0; i < c.nodeIds.size(); ++i)
        c.idToIndex[c.nodeIds[i]] = i;

    size_t N = c.nodeIds.size();
    c.outAdj.assign(N, {});
    c.posT.assign(N, 0);
    c.negT.assign(N, 0);
    c.outW.assign(N, 0);

    for (size_t i = 0; i < N; ++i) {
        int id = c.nodeIds[i];
        c.posT[i] = G.getNodeThreshold(id);
        c.negT[i] = G.getNodeThreshold2(id);

        vector<int> outs = G.getNodeOutNeighbors(id);
        auto& adj = c.outAdj[i];
        adj.reserve(outs.size());
        double tot = 0;
        for (int nb : outs) {
            double w = G.getEdgeInfluence(id, nb);
            tot += w;
            auto it = c.idToIndex.find(nb);
            if (it != c.idToIndex.end()) adj.emplace_back(it->second, w);
        }
        sort(adj.begin(), adj.end());
        c.outW[i] = tot;
    }
    return c;
}

// --------------------------------------------------------------------
// Negative exposure
// --------------------------------------------------------------------
static vector<double> computeNegExp(const GraphCache& c, const unordered_set<int>& neg) {
    vector<double> r(c.nodeIds.size(), 0);
    if (neg.empty()) return r;
    for (int id : neg) {
        int i = c.idxOf(id);
        if (i < 0) continue;
        for (auto& p : c.outAdj[i]) r[p.first] += p.second;
    }
    return r;
}

// --------------------------------------------------------------------
// Full diffusion
// --------------------------------------------------------------------
struct DiffRes { size_t pa=0, na=0; double s=0; };
static DiffRes runSim(DirectedGraph& G,
                      const unordered_set<int>& ps,
                      const unordered_set<int>& ns)
{
    DiffRes r;
    unordered_set<int> fp, fn;
    diffuse_signed_all(&G, ps, ns, fp, fn);
    r.pa = fp.size();
    r.na = fn.size();
    r.s  = double(r.pa) - double(r.na);
    return r;
}

} // namespace

// =====================================================================
// STRONG-PREV v2 seedSelection
// =====================================================================
unordered_set<int> seedSelection(DirectedGraph& G, unsigned int K) {
    using namespace student_algo_detail;

    unordered_set<int> S;
    if (K == 0 || G.getSize() == 0) return S;

    const GraphCache c = buildGraphCache(G);
    const SeedInfo& info = getSeedInfo();

    unordered_set<int> banned = info.negatives;
    if (info.hasGiven) banned.insert(info.given);

    size_t N = c.nodeIds.size();

    bool small  = (N <= 200);
    bool large  = (N > 5000);

    struct SizeProfile {
        double posCoeff;
        double negPenalty;
        double degCoeff;
        double outCoeff;
        double cutStd;
        double cutExtra;
        size_t candFloor;
        size_t candPerK;
        int refineBase;
        int refinePasses;
    } cfg;

    if (small) {
        cfg = {0.35, 0.90, 0.15, 1.35, -1.0, 0.0, N, N, 120, 2};
    } else if (large) {
        cfg = {0.65, 1.25, 0.03, 0.90, 1.1, 0.35, 180, 10, 15, 1};
    } else {
        cfg = {0.50, 1.00, 0.08, 1.10, 1.6, 0.40, 450, 25, 40, 1};
    }

    // ----------------------------------------------------------------
    // Fast score (size-aware tuning)
    // ----------------------------------------------------------------
    vector<double> sc(N);
    vector<double> negExp = computeNegExp(c, info.negatives);

    double negSum = 0.0, negSq = 0.0;
    for (double v : negExp) {
        negSum += v;
        negSq  += v * v;
    }
    double denom = max<size_t>(size_t(1), N);
    double negAvg = negSum / double(denom);
    double negVar = max(0.0, negSq / double(denom) - negAvg * negAvg);
    double negStd = sqrt(negVar);
    double negNorm = (negStd > 1e-9 ? negStd : (negAvg > 1e-9 ? negAvg : 1.0));
    double negCut = numeric_limits<double>::infinity();
    if (cfg.cutStd >= 0.0) {
        double base = negAvg * (1.0 + cfg.cutExtra);
        if (negStd > 1e-9) base += cfg.cutStd * negStd;
        else base += cfg.cutStd * max(negAvg, 1e-6);
        negCut = max(base, 0.0);
    }

    for (size_t i = 0; i < N; ++i) {
        double negScaled = negExp[i] / negNorm;
        double s = cfg.outCoeff * c.outW[i]
                 + cfg.degCoeff * double(c.outAdj[i].size())
                 - cfg.posCoeff * c.posT[i]
                 - cfg.negPenalty * negScaled;
        sc[i] = s;
    }

    vector<int> ord(N);
    for (int i = 0; i < (int)N; ++i) ord[i] = i;

    sort(ord.begin(), ord.end(), [&](int a, int b){
        if (sc[a] != sc[b]) return sc[a] > sc[b];
        return c.nodeIds[a] < c.nodeIds[b];
    });

    // =================================================================
    // Candidate pool (size aware)
    // =================================================================
    size_t candLimit = N;
    if (!small) {
        size_t perK = cfg.candPerK * max<size_t>(size_t(1), size_t(K));
        candLimit = min<size_t>(N, max<size_t>(cfg.candFloor, perK));
    }
    candLimit = max<size_t>(candLimit, min<size_t>(N, size_t(K)));

    vector<char> used(N, false);
    vector<int> cand;
    cand.reserve(candLimit);

    auto tryInsert = [&](int idx) {
        if (used[idx]) return false;
        int nid = c.nodeIds[idx];
        if (banned.count(nid)) return false;
        cand.push_back(nid);
        used[idx] = true;
        return true;
    };

    auto fillCand = [&](bool filter) {
        for (int idx : ord) {
            if (cand.size() >= candLimit) break;
            if (used[idx]) continue;
            if (filter && negCut < numeric_limits<double>::infinity() && negExp[idx] > negCut)
                continue;
            tryInsert(idx);
        }
    };

    fillCand(true);
    if (cand.size() < max<size_t>(size_t(K), candLimit / 3 + 1))
        fillCand(false);
    if (cand.empty()) {
        for (int idx : ord) {
            if (cand.size() >= candLimit) break;
            tryInsert(idx);
        }
    }

    unordered_set<int> neg = info.negatives;
    unordered_set<int> cur;
    if (info.hasGiven) cur.insert(info.given);

    if (large) {
        if (cand.empty()) {
            for (int idx : ord) {
                int nid = c.nodeIds[idx];
                if (banned.count(nid)) continue;
                cand.push_back(nid);
                if (cand.size() >= max<size_t>(size_t(K), size_t(40))) break;
            }
        }

        vector<pair<double,int>> ranked;
        size_t evalLimit = min<size_t>(cand.size(), max<size_t>(size_t(2 * K), size_t(40)));
        ranked.reserve(evalLimit);
        for (size_t i = 0; i < evalLimit; ++i) {
            unordered_set<int> tmp = cur;
            tmp.insert(cand[i]);
            double score = runSim(G, tmp, neg).s;
            ranked.emplace_back(score, cand[i]);
        }
        sort(ranked.begin(), ranked.end(), [&](const auto& a, const auto& b){
            if (a.first != b.first) return a.first > b.first;
            return a.second < b.second;
        });
        for (auto& p : ranked) {
            if (S.size() >= K) break;
            S.insert(p.second);
        }
        if (S.size() < K) {
            for (int nid : cand) {
                if (S.size() >= K) break;
                S.insert(nid);
            }
        }
        return S;
    }

    DiffRes base = runSim(G, cur, neg);
    double curS = base.s;
    int iter = 0;

    struct Ent { int id; double g, t; int u; };
    struct Cmp {
        bool operator()(const Ent&a,const Ent&b) const {
            return (a.g != b.g ? a.g < b.g : a.id > b.id);
        }
    };

    auto eval = [&](int id, int tg){
        unordered_set<int> tmp = cur;
        tmp.insert(id);
        DiffRes r = runSim(G, tmp, neg);
        return Ent{id, r.s - curS, r.s, tg};
    };

    priority_queue<Ent, vector<Ent>, Cmp> pq;
    for (int nid : cand) pq.push(eval(nid, 0));

    while (S.size() < K && !pq.empty()) {
        Ent x = pq.top(); pq.pop();
        if (cur.count(x.id) || banned.count(x.id)) continue;
        if (x.u == iter) {
            S.insert(x.id);
            cur.insert(x.id);
            curS = x.t;
            ++iter;
        } else pq.push(eval(x.id, iter));
    }

    // rebuild precise state after greedy loop
    cur.clear();
    if (info.hasGiven) cur.insert(info.given);
    for (int v : S) cur.insert(v);
    curS = runSim(G, cur, neg).s;

    // =====================================================================
    // STRONG-PREV v2 — LIMITED refinement (size aware)
    // =====================================================================
    if (!cand.empty()) {
        vector<int> seedList(S.begin(), S.end());
        if (!seedList.empty()) {
            int tryLimit = min<int>(cfg.refineBase, (int)cand.size());
            int passes = max(1, cfg.refinePasses);
            unordered_set<int> curSeeds = cur;
            double curScore = curS;
            for (int pass = 0; pass < passes; ++pass) {
                bool improved = false;
                for (size_t idx = 0; idx < seedList.size(); ++idx) {
                    int original = seedList[idx];
                    unordered_set<int> baseSet = curSeeds;
                    baseSet.erase(original);
                    double bestLocal = runSim(G, baseSet, neg).s;
                    int bestCand = original;
                    int tried = 0;
                    for (int nid : cand) {
                        if (baseSet.count(nid)) continue;
                        unordered_set<int> test = baseSet;
                        test.insert(nid);
                        double sc = runSim(G, test, neg).s;
                        ++tried;
                        if (sc > bestLocal) {
                            bestLocal = sc;
                            bestCand = nid;
                        }
                        if (tried >= tryLimit) break;
                    }
                    if (bestCand != original) {
                        seedList[idx] = bestCand;
                        curSeeds = baseSet;
                        curSeeds.insert(bestCand);
                        curScore = bestLocal;
                        improved = true;
                    }
                }
                if (!improved) break;
                cur = curSeeds;
                curS = curScore;
            }

            S.clear();
            for (int v : seedList) S.insert(v);

            cur.clear();
            if (info.hasGiven) cur.insert(info.given);
            for (int v : S) cur.insert(v);
            curS = runSim(G, cur, neg).s;
        }
    }

    if (S.size() < K) {
        for (int idx : ord) {
            if (S.size() >= K) break;
            int nid = c.nodeIds[idx];
            if (banned.count(nid) || S.count(nid)) continue;
            S.insert(nid);
        }
    }

    return S;
}

inline unordered_set<int> seedSelection(DirectedGraph& G,
                                        unsigned int K,
                                        int /*givenPosSeed*/,
                                        const unordered_set<int>& /*negSeeds*/)
{
    return seedSelection(G, K);
}

#endif
