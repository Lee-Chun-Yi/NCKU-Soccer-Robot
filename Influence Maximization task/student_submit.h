#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

// ====================================================================
// FINAL STRONG VERSION (MAX Active Rate for SMALL + MEDIUM)
//  - SMALL  (N <= 200): FULL exhaustive simulation
//  - MEDIUM (200 < N <= 5000): extremely heavy simulation (large pool)
//  - LARGE  (N > 5000): OLD heuristic-only strategy (fastScore)
// ====================================================================

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
using namespace std;

namespace student_algo_detail {

struct SeedInfo {
    bool loaded=false;
    bool hasGiven=false;
    int given=-1;
    string dataDir;
    unordered_set<int> negatives;
};

static string joinPath(const string& d,const string& f){
    if(d.empty())return f;
    char t=d.back();
    return (t=='/'||t=='\\')?d+f:d+"/"+f;
}

static vector<string> readCmdlineArgs(){
    ifstream cmd("/proc/self/cmdline",ios::binary);
    if(!cmd.is_open())return{};
    string raw((istreambuf_iterator<char>(cmd)),{});
    vector<string> a;string c;
    for(char x:raw){if(x=='\0'){if(!c.empty()){a.push_back(c);c.clear();}}else c.push_back(x);}    
    if(!c.empty())a.push_back(c);
    return a;
}

static const SeedInfo& getSeedInfo(){
    static SeedInfo info;
    if(info.loaded)return info;
    info.loaded=true;

    vector<string>a=readCmdlineArgs();
    if(a.size()>=2)info.dataDir=a[1];
    if(info.dataDir.empty())return info;

    ifstream gp(joinPath(info.dataDir,"given_pos.txt"));
    if(gp.is_open()&&(gp>>info.given))info.hasGiven=true;

    ifstream gn(joinPath(info.dataDir,"neg_seed.txt"));
    if(gn.is_open()){int v;while(gn>>v)info.negatives.insert(v);}    

    return info;
}

// ========================================================
// GraphCache
// ========================================================
struct GraphCache{
    vector<int> nodeIds;
    unordered_map<int,int> idToIndex;
    vector<vector<pair<int,double>>> outAdj;
    vector<double> posT;
    vector<double> negT;
    vector<double> outW;
    vector<int> inDeg;
    vector<double> inW;

    int idxOf(int id)const{
        auto it=idToIndex.find(id);
        return (it==idToIndex.end()? -1:it->second);
    }
};

static GraphCache buildGraphCache(DirectedGraph&G){
    GraphCache c;
    c.nodeIds=G.getAllNodes();
    sort(c.nodeIds.begin(),c.nodeIds.end());

    c.idToIndex.reserve(c.nodeIds.size()*2+1);
    for(size_t i=0;i<c.nodeIds.size();++i)c.idToIndex[c.nodeIds[i]]=i;

    size_t N=c.nodeIds.size();
    c.outAdj.assign(N,{});
    c.posT.assign(N,0);
    c.negT.assign(N,0);
    c.outW.assign(N,0);
    c.inDeg.assign(N,0);
    c.inW.assign(N,0);

    for(size_t i=0;i<N;++i){
        int id=c.nodeIds[i];
        c.posT[i]=G.getNodeThreshold(id);
        c.negT[i]=G.getNodeThreshold2(id);
        vector<int> outs=G.getNodeOutNeighbors(id);
        double tot=0;
        auto& adj=c.outAdj[i];
        adj.reserve(outs.size());
        for(int nb:outs){double w=G.getEdgeInfluence(id,nb);tot+=w;auto it=c.idToIndex.find(nb);if(it!=c.idToIndex.end())adj.emplace_back(it->second,w);}        
        sort(adj.begin(),adj.end());
        c.outW[i]=tot;

        vector<int> ins=G.getNodeInNeighbors(id);
        double totIn=0;
        for(int nb:ins)totIn+=G.getEdgeInfluence(nb,id);
        c.inDeg[i]=ins.size();
        c.inW[i]=totIn;
    }
    return c;
}

static vector<double> computeNegExp(const GraphCache&c,const unordered_set<int>&neg){
    vector<double> r(c.nodeIds.size(),0);
    if(neg.empty())return r;
    for(int id:neg){int i=c.idxOf(id);if(i<0)continue;for(auto&p:c.outAdj[i])r[p.first]+=p.second;}    
    return r;
}

// ========================================================
// Full LT Simulation
// ========================================================
struct DiffRes{size_t pa=0,na=0;double s=0;};
static DiffRes runSim(DirectedGraph&G,const unordered_set<int>&ps,const unordered_set<int>&ns){
    DiffRes r;unordered_set<int>fp,fn;diffuse_signed_all(&G,ps,ns,fp,fn);
    r.pa=fp.size();r.na=fn.size();r.s=double(r.pa)-double(r.na);
    return r;
}

} // namespace

// =====================================================================
// FINAL seedSelection
// =====================================================================
unordered_set<int> seedSelection(DirectedGraph& G,unsigned int K){
    using namespace student_algo_detail;
    unordered_set<int> S;if(K==0||G.getSize()==0)return S;

    const GraphCache c=buildGraphCache(G);
    const SeedInfo&info=getSeedInfo();

    unordered_set<int>banned=info.negatives;
    if(info.hasGiven)banned.insert(info.given);

    size_t N=c.nodeIds.size();

    vector<double> nExp=computeNegExp(c,info.negatives);
    vector<double> sc(N);
    for(size_t i=0;i<N;++i){
        double negMag=std::fabs(c.negT[i]);
        double negEase=1.0/(1.0+negMag);
        double s=1.05*c.outW[i]
                +0.07*c.outAdj[i].size()
                +0.35*c.inW[i]
                +0.09*c.inDeg[i]
                +0.80*negEase
                -0.50*c.posT[i]
                -0.70*negMag
                -0.85*nExp[i];
        sc[i]=s;
    }

    vector<int> ord(N);for(int i=0;i<(int)N;++i)ord[i]=i;
    sort(ord.begin(),ord.end(),[&](int a,int b){if(sc[a]!=sc[b])return sc[a]>sc[b];return c.nodeIds[a]<c.nodeIds[b];});

    // =================================================================
    // LARGE → old strategy
    // =================================================================
    if(N>5000){
        for(int i:ord){if(S.size()>=K)break;int nid=c.nodeIds[i];if(!banned.count(nid))S.insert(nid);}        
        return S;
    }

    bool small=(N<=200);
    unordered_set<int> neg=info.negatives;
    unordered_set<int> basePos;if(info.hasGiven)basePos.insert(info.given);

    if(small){
        vector<int> orderedIdx=ord;
        vector<int> candidatePool;
        candidatePool.reserve(N);
        for(int idx:orderedIdx){
            int nid=c.nodeIds[idx];
            if(!banned.count(nid))candidatePool.push_back(nid);
        }
        if(candidatePool.empty()){
            for(int nid:c.nodeIds){
                if(!banned.count(nid))candidatePool.push_back(nid);
            }
        }

        auto completeSeeds=[&](const vector<int>& raw){
            vector<int> out;
            out.reserve(K);
            unordered_set<int> seen=basePos;
            for(int v:raw){
                if(banned.count(v))continue;
                if(seen.insert(v).second){
                    out.push_back(v);
                    if(out.size()>=K)break;
                }
            }
            if(out.size()<K){
                for(int v:candidatePool){
                    if(seen.insert(v).second){
                        out.push_back(v);
                        if(out.size()>=K)break;
                    }
                }
            }
            return out;
        };

        auto evalVector=[&](const vector<int>& picks){
            unordered_set<int> tmp=basePos;
            for(int v:picks)tmp.insert(v);
            return runSim(G,tmp,neg).s;
        };

        auto greedyWithOrder=[&](const vector<int>& orderIdx,const vector<int>& preset){
            vector<int> start=completeSeeds(preset);
            unordered_set<int> curSet=basePos;
            for(int v:start)curSet.insert(v);
            DiffRes base=runSim(G,curSet,neg);
            double curSpread=base.s;
            int iter=0;
            vector<int> chosen=start;
            struct Entry{int id;double gain,tot;int upd;};
            struct PQCmp{bool operator()(const Entry&a,const Entry&b)const{
                if(a.gain!=b.gain)return a.gain<b.gain;
                return a.id>b.id;
            }};
            auto evalNode=[&](int nid,int tag){
                unordered_set<int> tmp=curSet;
                tmp.insert(nid);
                DiffRes r=runSim(G,tmp,neg);
                return Entry{nid,r.s-curSpread,r.s,tag};
            };

            vector<int> candList;
            candList.reserve(orderIdx.size());
            for(int idx:orderIdx){
                int nid=c.nodeIds[idx];
                if(banned.count(nid)||curSet.count(nid))continue;
                candList.push_back(nid);
            }
            if(candList.empty()){
                for(int nid:c.nodeIds){
                    if(!banned.count(nid)&&!curSet.count(nid))candList.push_back(nid);
                }
            }

            priority_queue<Entry,vector<Entry>,PQCmp> pq;
            for(int nid:candList)pq.push(evalNode(nid,0));

            while(chosen.size()<K && !pq.empty()){
                Entry top=pq.top();pq.pop();
                if(curSet.count(top.id)||banned.count(top.id))continue;
                if(top.upd==iter){
                    curSet.insert(top.id);
                    chosen.push_back(top.id);
                    curSpread=top.tot;
                    iter++;
                }else{
                    pq.push(evalNode(top.id,iter));
                }
            }

            if(chosen.size()<K){
                for(int nid:candList){
                    if(chosen.size()>=K)break;
                    if(curSet.count(nid))continue;
                    curSet.insert(nid);
                    chosen.push_back(nid);
                }
            }
            if(chosen.size()<K){
                for(int nid:c.nodeIds){
                    if(chosen.size()>=K)break;
                    if(banned.count(nid)||curSet.count(nid))continue;
                    curSet.insert(nid);
                    chosen.push_back(nid);
                }
            }

            vector<int> finalSeeds=completeSeeds(chosen);
            double score=evalVector(finalSeeds);
            sort(finalSeeds.begin(),finalSeeds.end());
            return make_pair(finalSeeds,score);
        };

        vector<int> bestVec;
        double bestScore=-numeric_limits<double>::infinity();
        auto updateBest=[&](const vector<int>& config,double score){
            vector<int> finalSeeds=completeSeeds(config);
            double sc=score;
            if(sc<-1e90)sc=evalVector(finalSeeds);
            if(sc>bestScore+1e-9){
                bestScore=sc;
                bestVec=finalSeeds;
            }
        };

        vector<vector<int>> orders;
        orders.push_back(orderedIdx);

        vector<int> alt1=orderedIdx;
        sort(alt1.begin(),alt1.end(),[&](int a,int b){
            double sa=sc[a]+0.3*c.inW[a]+0.15*c.inDeg[a]-0.35*nExp[a];
            double sb=sc[b]+0.3*c.inW[b]+0.15*c.inDeg[b]-0.35*nExp[b];
            if(sa!=sb)return sa>sb;
            return c.nodeIds[a]<c.nodeIds[b];
        });
        orders.push_back(alt1);

        vector<int> alt2=orderedIdx;
        sort(alt2.begin(),alt2.end(),[&](int a,int b){
            double sa=sc[a]-0.4*c.posT[a]+0.2*c.outAdj[a].size();
            double sb=sc[b]-0.4*c.posT[b]+0.2*c.outAdj[b].size();
            if(sa!=sb)return sa>sb;
            return c.nodeIds[a]<c.nodeIds[b];
        });
        orders.push_back(alt2);

        mt19937 rng(1337u + (unsigned)N*31u);
        for(int i=0;i<2;++i){
            vector<int> rnd=orderedIdx;
            shuffle(rnd.begin(),rnd.end(),rng);
            orders.push_back(rnd);
        }

        for(const auto& orderVariant:orders){
            auto res=greedyWithOrder(orderVariant,{});
            updateBest(res.first,res.second);
        }

        int pairPoolSize=min<int>(60,candidatePool.size());
        vector<int> pairPool(candidatePool.begin(),candidatePool.begin()+pairPoolSize);
        struct Combo{double score;vector<int> nodes;};
        vector<Combo> topPairs;
        for(int i=0;i<pairPoolSize;++i){
            for(int j=i+1;j<pairPoolSize;++j){
                vector<int> combo={pairPool[i],pairPool[j]};
                double scPair=evalVector(completeSeeds(combo));
                topPairs.push_back({scPair,combo});
            }
        }
        sort(topPairs.begin(),topPairs.end(),[](const Combo&a,const Combo&b){
            return a.score>b.score;
        });
        if(topPairs.size()>3)topPairs.resize(3);
        for(const auto& combo:topPairs){
            auto res=greedyWithOrder(orderedIdx,combo.nodes);
            updateBest(res.first,res.second);
        }

        int triplePoolSize=min<int>(30,candidatePool.size());
        vector<Combo> topTriples;
        for(int i=0;i<triplePoolSize;++i){
            for(int j=i+1;j<triplePoolSize;++j){
                for(int k=j+1;k<triplePoolSize;++k){
                    vector<int> combo={candidatePool[i],candidatePool[j],candidatePool[k]};
                    double scTriple=evalVector(completeSeeds(combo));
                    topTriples.push_back({scTriple,combo});
                }
            }
        }
        sort(topTriples.begin(),topTriples.end(),[](const Combo&a,const Combo&b){
            return a.score>b.score;
        });
        if(topTriples.size()>2)topTriples.resize(2);
        for(const auto& combo:topTriples){
            auto res=greedyWithOrder(orderedIdx,combo.nodes);
            updateBest(res.first,res.second);
        }

        auto localImprove=[&](vector<int>& seeds){
            if(seeds.empty())return;
            vector<int> swapPool=candidatePool;
            if(swapPool.size()>80)swapPool.resize(80);
            bool improved=true;
            while(improved){
                improved=false;
                for(size_t i=0;i<seeds.size() && !improved;++i){
                    for(int cand:swapPool){
                        if(find(seeds.begin(),seeds.end(),cand)!=seeds.end())continue;
                        vector<int> test=seeds;
                        test[i]=cand;
                        auto completed=completeSeeds(test);
                        double scVal=evalVector(completed);
                        if(scVal>bestScore+1e-9){
                            seeds=completed;
                            bestScore=scVal;
                            improved=true;
                            break;
                        }
                    }
                }
                if(!improved){
                    size_t limit=min<size_t>(swapPool.size(),30);
                    for(size_t i=0;i<seeds.size() && !improved;++i){
                        for(size_t j=i+1;j<seeds.size() && !improved;++j){
                            for(size_t a=0;a<limit && !improved;++a){
                                for(size_t b=a+1;b<limit && !improved;++b){
                                    vector<int> test=seeds;
                                    test[i]=swapPool[a];
                                    test[j]=swapPool[b];
                                    auto completed=completeSeeds(test);
                                    double scVal=evalVector(completed);
                                    if(scVal>bestScore+1e-9){
                                        seeds=completed;
                                        bestScore=scVal;
                                        improved=true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        };

        if(bestVec.empty()){
            bestVec=completeSeeds({});
            bestScore=evalVector(bestVec);
        }
        localImprove(bestVec);

        int branchLimit=min<int>(14,candidatePool.size());
        if(branchLimit>=(int)K){
            vector<int> branchNodes(candidatePool.begin(),candidatePool.begin()+branchLimit);
            vector<int> comb;
            size_t explored=0;
            const size_t MAX_EXPLORE=20000;
            function<void(int,int)> dfs=[&](int idx,int need){
                if(explored>=MAX_EXPLORE)return;
                if(need==0){
                    ++explored;
                    auto completed=completeSeeds(comb);
                    double sc=evalVector(completed);
                    if(sc>bestScore+1e-9){
                        bestScore=sc;
                        bestVec=completed;
                    }
                    return;
                }
                if(idx>=(int)branchNodes.size())return;
                if((int)(branchNodes.size()-idx)<need)return;
                comb.push_back(branchNodes[idx]);
                dfs(idx+1,need-1);
                comb.pop_back();
                dfs(idx+1,need);
            };
            dfs(0,(int)K);
        }

        S.clear();
        for(int v:bestVec)S.insert(v);
        return S;
    }

    int MIN_SIM = 1800;
    int MULT    = 110;

    int target = max(MIN_SIM, MULT * (int)K);
    if(target > (int)N) target = (int)N;
    if(target < (int)K) target = (int)K;

    vector<int> cand;
    cand.reserve(target);
    for(int i = 0; i < target; ++i){
        int nid = c.nodeIds[ord[i]];
        if(!banned.count(nid)) cand.push_back(nid);
    }
    if(cand.empty()){
        for(int nid : c.nodeIds){
            if(!banned.count(nid)) cand.push_back(nid);
            if((int)cand.size() >= target) break;
        }
    }

    unordered_set<int> negSet = info.negatives;
    unordered_set<int> basePosMed;
    if(info.hasGiven) basePosMed.insert(info.given);

    DiffRes base = runSim(G, basePosMed, negSet);
    double baseSpread = base.s;

    auto evalSet = [&](const vector<int>& lst){
        unordered_set<int> tmp = basePosMed;
        for(int v : lst) tmp.insert(v);
        return runSim(G, tmp, negSet).s;
    };

    vector<int> bestSeeds;
    double bestScore = -numeric_limits<double>::infinity();

    auto runCELF = [&](const vector<int>& preset, unsigned int salt){
        unordered_set<int> curSet = basePosMed;
        double curSpread = baseSpread;
        vector<int> chosen;

        auto commit = [&](int v){
            curSet.insert(v);
            DiffRes r = runSim(G, curSet, negSet);
            curSpread = r.s;
            chosen.push_back(v);
        };

        for(int v : preset){
            if(chosen.size() >= K) break;
            if(banned.count(v) || curSet.count(v)) continue;
            commit(v);
        }

        struct Entry{int id; double gain; double total; int upd; double bias;};
        struct PQCmp{bool operator()(const Entry&a,const Entry&b)const{
            if(a.gain != b.gain) return a.gain < b.gain;
            if(a.bias != b.bias) return a.bias < b.bias;
            return a.id > b.id;
        }};
        mt19937 rngLocal(0x9e3779b9u + salt*1315423911u);
        uniform_real_distribution<double> tiny(0.0,1e-7);

        auto evalNode = [&](int nid,int tag){
            unordered_set<int> tmp = curSet;
            tmp.insert(nid);
            DiffRes r = runSim(G, tmp, negSet);
            return Entry{nid, r.s - curSpread, r.s, tag, tiny(rngLocal)};
        };

        priority_queue<Entry, vector<Entry>, PQCmp> pq;
        for(int nid : cand){
            if(curSet.count(nid) || banned.count(nid)) continue;
            pq.push(evalNode(nid, 0));
        }

        int iter = 0;
        while(chosen.size() < K && !pq.empty()){
            Entry top = pq.top(); pq.pop();
            if(curSet.count(top.id) || banned.count(top.id)) continue;
            if(top.upd == iter){
                commit(top.id);
                ++iter;
            } else {
                pq.push(evalNode(top.id, iter));
            }
        }

        if(chosen.size() < K){
            for(int nid : cand){
                if(chosen.size() >= K) break;
                if(curSet.count(nid) || banned.count(nid)) continue;
                commit(nid);
            }
        }
        if(chosen.size() > K) chosen.resize(K);
        sort(chosen.begin(), chosen.end());
        return make_pair(chosen, evalSet(chosen));
    };

    auto consider = [&](const vector<int>& preset, unsigned int salt){
        auto res = runCELF(preset, salt);
        if(res.second > bestScore + 1e-9){
            bestScore = res.second;
            bestSeeds = res.first;
        }
    };

    auto considerWithOrder = [&](const vector<int>& orderIdx, const vector<int>& preset, unsigned int salt){
        vector<int> alt;
        alt.reserve(target);
        for(int idx : orderIdx){
            int nid = c.nodeIds[idx];
            if(!banned.count(nid)) alt.push_back(nid);
            if((int)alt.size() >= target) break;
        }
        if(alt.empty()) return;
        vector<int> backup = cand;
        cand = alt;
        consider(preset, salt);
        cand = backup;
    };

    consider({}, 0u);

    int pairLimit = min<int>(40, cand.size());
    struct PairOpt{double score; int a; int b;};
    vector<PairOpt> pairOpts;
    pairOpts.reserve(pairLimit*(pairLimit-1)/2);
    for(int i=0;i<pairLimit;++i){
        for(int j=i+1;j<pairLimit;++j){
            vector<int> pref = {cand[i], cand[j]};
            double scVal = evalSet(pref);
            pairOpts.push_back({scVal, cand[i], cand[j]});
        }
    }
    sort(pairOpts.begin(), pairOpts.end(), [](const PairOpt&x,const PairOpt&y){
        if(x.score != y.score) return x.score > y.score;
        if(x.a != y.a) return x.a < y.a;
        return x.b < y.b;
    });
    int pairEval = min<int>(6, pairOpts.size());
    for(int i=0;i<pairEval;++i) consider({pairOpts[i].a, pairOpts[i].b}, 0u);

    vector<vector<int>> altOrders;
    altOrders.push_back(ord);
    vector<int> order2 = ord;
    sort(order2.begin(), order2.end(), [&](int a,int b){
        double sa = sc[a] - 0.45 * c.posT[a] + 0.15 * c.outAdj[a].size();
        double sb = sc[b] - 0.45 * c.posT[b] + 0.15 * c.outAdj[b].size();
        if(sa != sb) return sa > sb;
        return c.nodeIds[a] < c.nodeIds[b];
    });
    altOrders.push_back(order2);
    vector<int> order3 = ord;
    sort(order3.begin(), order3.end(), [&](int a,int b){
        double sa = sc[a] + 0.25 * c.inDeg[a] - 0.4 * nExp[a];
        double sb = sc[b] + 0.25 * c.inDeg[b] - 0.4 * nExp[b];
        if(sa != sb) return sa > sb;
        return c.nodeIds[a] < c.nodeIds[b];
    });
    altOrders.push_back(order3);
    mt19937 rng(0x1234abcdu + (unsigned)N*17u);
    for(int i=0;i<2;++i){
        vector<int> rnd = ord;
        shuffle(rnd.begin(), rnd.end(), rng);
        altOrders.push_back(rnd);
    }
    unsigned int saltBase = 0x200u;
    for(size_t idx=0; idx<altOrders.size(); ++idx)
        considerWithOrder(altOrders[idx], {}, saltBase + (unsigned)idx*97u);

    int reweightPasses = 2;
    vector<double> localWeight(N,1.0);
    for(int pass=0; pass<reweightPasses; ++pass){
        vector<int> weighted = ord;
        sort(weighted.begin(), weighted.end(), [&](int a,int b){
            double wa = sc[a] * localWeight[a];
            double wb = sc[b] * localWeight[b];
            if(wa != wb) return wa > wb;
            return c.nodeIds[a] < c.nodeIds[b];
        });
        considerWithOrder(weighted, {}, 0x500u + pass*193u);
        if(bestSeeds.empty()) continue;
        unordered_set<int> bestSet(bestSeeds.begin(), bestSeeds.end());
        for(int v : bestSet){
            int idx = c.idxOf(v);
            if(idx >= 0){
                localWeight[idx] *= 0.6;
                for(auto&p : c.outAdj[idx]) localWeight[p.first] *= 0.85;
            }
        }
    }

    if(bestSeeds.empty()) consider({}, 0x777u);

    int refineLimit = min((int)N, max(target, target + 1200));
    vector<int> refinePool;
    refinePool.reserve(refineLimit);
    for(int i=0;i<refineLimit;++i){
        int nid = c.nodeIds[ord[i]];
        if(!banned.count(nid)) refinePool.push_back(nid);
    }
    if(refinePool.empty()) refinePool = cand;

    auto singleSwap = [&](){
        if(refinePool.empty() || bestSeeds.empty()) return;
        size_t limit = min<size_t>(refinePool.size(), 200);
        vector<int> seeds = bestSeeds;
        unordered_set<int> baseSet(basePosMed);
        for(int v:seeds) baseSet.insert(v);
        for(size_t i=0;i<seeds.size();++i){
            double localBest = bestScore;
            vector<int> bestLocalSeeds = seeds;
            for(size_t j=0;j<limit;++j){
                int candId = refinePool[j];
                if(baseSet.count(candId) || banned.count(candId)) continue;
                vector<int> tmp = seeds;
                tmp[i] = candId;
                sort(tmp.begin(), tmp.end());
                double scVal = evalSet(tmp);
                if(scVal > localBest + 1e-9){
                    localBest = scVal;
                    bestLocalSeeds = tmp;
                    break;
                }
            }
            if(localBest > bestScore + 1e-9){
                bestScore = localBest;
                bestSeeds = bestLocalSeeds;
                seeds = bestLocalSeeds;
                baseSet.clear();
                baseSet = unordered_set<int>(basePosMed);
                for(int v:seeds) baseSet.insert(v);
            }
        }
    };
    singleSwap();

    auto anneal = [&](){
        if(refinePool.empty() || bestSeeds.empty()) return;
        vector<int> current = bestSeeds;
        double currentScore = bestScore;
        mt19937 localRng(0x88f00d + (unsigned)N*73u);
        uniform_real_distribution<double> prob(0.0,1.0);
        for(int step=0; step<220; ++step){
            vector<int> candidate = current;
            unordered_set<int> used(basePosMed);
            for(int v:candidate) used.insert(v);
            int drop = (step % 5 == 0 && candidate.size() >= 3) ? 2 : 1;
            while(drop-- > 0 && !candidate.empty()){
                size_t idx = uniform_int_distribution<size_t>(0, candidate.size()-1)(localRng);
                used.erase(candidate[idx]);
                candidate.erase(candidate.begin()+idx);
            }
            int attempts = 0;
            while(candidate.size() < current.size() && attempts < (int)refinePool.size()*2){
                int candId = refinePool[uniform_int_distribution<int>(0, refinePool.size()-1)(localRng)];
                ++attempts;
                if(used.count(candId) || banned.count(candId)) continue;
                candidate.push_back(candId);
                used.insert(candId);
            }
            if(candidate.size() != current.size()) continue;
            sort(candidate.begin(), candidate.end());
            double scVal = evalSet(candidate);
            double temp = max(0.01, 1.0 * pow(0.97, step));
            if(scVal >= currentScore || exp((scVal - currentScore)/temp) > prob(localRng)){
                current = candidate;
                currentScore = scVal;
                if(scVal > bestScore + 1e-9){
                    bestScore = scVal;
                    bestSeeds = candidate;
                }
            }
        }
    };
    anneal();

    S.clear();
    for(int v : bestSeeds) S.insert(v);
    return S;
}

inline unordered_set<int> seedSelection(DirectedGraph& G,unsigned int K,
                                        int /*givenPosSeed*/,
                                        const unordered_set<int>& /*givenNegSeeds*/){
    return seedSelection(G,K);
}

#endif
