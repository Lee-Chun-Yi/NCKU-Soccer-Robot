#ifndef YOUR_ALGORITHM_H
#define YOUR_ALGORITHM_H

#include "LT.h"
#include "graph.h"
#include <unordered_set>

using namespace std;

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

	return seeds;
}

#endif
