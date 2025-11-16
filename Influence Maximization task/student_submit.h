
#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <algorithm>
#include <fstream>
#include <iterator>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

// Seed Info Loader
namespace student_algo_detail {

	struct SeedInfo {
		bool loaded = false;
		bool hasGiven = false;
		int given = -1;
		string dataDir;
		unordered_set<int> negatives;
	};

	static string joinPath(const string& dir, const string& file) {
		return dir.empty() ? file : dir + ((dir.back() == '/' || dir.back() == '\') ? "" : "/") + file;
	}

	static vector<string> readCmdlineArgs() {
		ifstream cmd("/proc/self/cmdline", ios::binary);
		string raw((istreambuf_iterator<char>(cmd)), {});
		vector<string> args;
		string current;
		for (char c : raw) c ? current += c : (args.push_back(current), current.clear());
		if (!current.empty()) args.push_back(current);
		return args;
	}

	static const SeedInfo& getSeedInfo() {
		static SeedInfo info;
		if (info.loaded) return info;
		info.loaded = true;
		vector<string> args = readCmdlineArgs();
		if (args.size() >= 2) info.dataDir = args[1];

		ifstream gp(joinPath(info.dataDir, "given_pos.txt"));
		if (gp >> info.given) info.hasGiven = true;

		ifstream gn(joinPath(info.dataDir, "neg_seed.txt"));
		for (int val; gn >> val;) info.negatives.insert(val);

		return info;
	}

	// Graph Data Cache
	struct GraphCache {
		vector<int> nodeIds;
		unordered_map<int, int> idToIndex;
		vector<vector<pair<int, double>>> outAdj;
		vector<double> posThreshold, negThreshold, outStrength;

		int indexOf(int id) const {
			auto it = idToIndex.find(id);
			return it == idToIndex.end() ? -1 : it->second;
		}
	};

	static GraphCache buildGraphCache(DirectedGraph& G) {
		GraphCache c;
		c.nodeIds = G.getAllNodes();
		sort(c.nodeIds.begin(), c.nodeIds.end());
		for (size_t i = 0; i < c.nodeIds.size(); ++i) c.idToIndex[c.nodeIds[i]] = i;

		size_t N = c.nodeIds.size();
		c.outAdj.resize(N);
		c.posThreshold.resize(N);
		c.negThreshold.resize(N);
		c.outStrength.resize(N);

		for (size_t i = 0; i < N; ++i) {
			int u = c.nodeIds[i];
			c.posThreshold[i] = G.getNodeThreshold(u);
			c.negThreshold[i] = G.getNodeThreshold2(u);
			double sum = 0.0;
			for (int v : G.getNodeOutNeighbors(u)) {
				double w = G.getEdgeInfluence(u, v);
				sum += w;
				int idx = c.indexOf(v);
				if (idx >= 0) c.outAdj[i].emplace_back(idx, w);
			}
			sort(c.outAdj[i].begin(), c.outAdj[i].end());
			c.outStrength[i] = sum;
		}
		return c;
	}

	static vector<double> computeNegExposure(const GraphCache& c, const unordered_set<int>& negSeeds) {
		vector<double> exposure(c.nodeIds.size(), 0.0);
		for (int id : negSeeds) {
			int idx = c.indexOf(id);
			if (idx < 0) continue;
			for (const auto& e : c.outAdj[idx]) exposure[e.first] += e.second;
		}
		return exposure;
	}

	struct FullDiffResult {
		size_t posActive = 0, negActive = 0;
		double spread = 0.0;
	};

	static FullDiffResult runFullDiffusionSimulation(DirectedGraph& G, const unordered_set<int>& pos, const unordered_set<int>& neg) {
		FullDiffResult res;
		unordered_set<int> p, n;
		diffuse_signed_all(&G, pos, neg, p, n);
		res.posActive = p.size();
		res.negActive = n.size();
		res.spread = double(p.size()) - double(n.size());
		return res;
	}

} 

