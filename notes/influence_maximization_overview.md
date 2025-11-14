# Influence Maximization Task Overview

This document summarizes the requirements gathered from the provided specification screenshots for the "Influence Maximization with simple Linear Threshold model" challenge. The latest batch of slides further clarifies the social-network context, the LT diffusion model, and how negative influence interferes with propagation.

## Problem Summary
- Objective: maximize the number of positive active users and minimize negative active users within a directed social network using the Linear Threshold (LT) model.
- Need to identify influential seed nodes to optimize advertising effect.
- Social graphs represent social media platforms where each node is a user and each directed edge (with weight) denotes potential influence from the source to the destination user.
- Negative influence (rumors, fake news) suppresses positive adoption by preventing users from accepting positive influence, so the algorithm must consider both positive spread and mitigation of harmful cascades.

## Input Format
- Graph described by `node.txt` and `edge.txt`.
  - `node.txt`: first line is number of nodes. Each subsequent line describes a node's ID plus its positive/negative threshold.
  - `edge.txt`: first line is number of edges. Each following line lists a directed edge with weight.
- Edge weights are the contribution scores used by the LT model; when a node becomes active it propagates influence through all of its outgoing edges proportionally to those weights.

## Output Requirements
- Implement the `seedSelection` function in `your_algorithm.h` returning an `unordered_set<int>` containing chosen seed nodes.

## Dataset Information
- Six datasets across three sizes: Small (100 nodes, ~1k edges), Medium (1k nodes, 9k–10k edges), Large (10k nodes, 500k edges). Provided dataset is Medium.

## Evaluation
- Score = Average(sigma(Active_Rate_of_each_dataset)).
- Diffusion proceeds in synchronous rounds. In each round, every active node attempts to activate its neighbors by contributing its edge weight to their accumulated influence. A non-active node activates when its accumulated influence meets or exceeds its individual threshold; propagation stops once no new nodes become active.
- Example (from slides): given a node with threshold 0.6, the propagation sequence may take three rounds before no further activations occur once the cumulative incoming weight crosses the threshold; intermediate star markers on the graph track which nodes activate at each round.

## Additional Context from Slides
- The course references popular social media platforms (Instagram, Twitter, Facebook, Line, etc.) to emphasize the practical relevance of the modeled social network.
- Nodes and edges form a directed weighted graph where weights such as 0.55, 0.3, 0.9 indicate influence probabilities or strengths.
- Negative influence represents content that should be contained (e.g., rumors or fake news) because it prevents the spread of positive influence.

## Graph Utility API (from latest slide deck)
The assignment materials also describe the helper methods provided by the graph container. Understanding these APIs clarifies how to traverse the network and retrieve node/edge metadata when implementing `seedSelection` or simulating LT diffusion:

- `getAllEdges()` → returns a vector of `(from_node, to_node)` pairs for every active edge; inactive or deleted edges are skipped so the list mirrors the current topology.
- `addEdge(int from, int to, double w)` → links two nodes with a directed edge of weight `w`, auto-creating any missing nodes and updating both nodes' `in_edges`/`out_edges` lists plus the global edge counter.
- `getNodeThreshold(int id)` / `getNodeThreshold2(int id)` → expose the positive and negative thresholds for a node (returning `0` if the node is missing). Both are `const` so they never mutate the graph.
- `getNodeInNeighbors(int id)` / `getNodeOutNeighbors(int id)` → yield the incoming or outgoing neighbor IDs filtered to only active edges. Empty vectors are returned when the node does not exist.
- `getEdgeInfluence(int u, int v)` → scans the active outgoing edges of `u` to report the influence weight toward `v`; returns `0` if the directed edge does not exist.
- `getAllNodes()` → gathers every node ID stored in `nodeMap`, enabling complete iterations over the network during simulation.
- `getSize()`, `getEdgeNumber()`, and `isNodeExist(int id)` → lightweight utilities that report node/edge counts or a node’s presence in the internal map.
- `isEdgeExist(int from, int to)` → checks whether a directed connection from `from` to `to` is active, helping algorithms avoid duplicate edge insertions or handle conditional logic before propagating influence.

## Graph Mutation Helpers
The directed graph implementation not only exposes read-only helpers, it also supports safe mutation of the topology:

- `deleteEdge(int u, int v)` removes an active edge if it exists. The function looks up the adjacency lists, toggles the edge's `active` flag to `false`, erases the iterator from the owning node's `out_edges` / `in_edges` containers, and decrements the global edge counter. When the edge is already missing the call simply returns `false`.
- `deleteNode(int id)` first confirms the node exists, then iterates through all incoming and outgoing edges to mark them inactive and clear their respective containers. After cleaning up `in_edges`, `out_edges`, and the node object itself, it erases the entry from `nodeMap` and decreases the node counter. Returning `false` signals that the requested node was not present.

These helpers are useful when building custom preprocessing pipelines (e.g., pruning low-weight edges or removing isolated nodes) before running the LT diffusion.

## Linear Threshold (LT) Model API
- Include the LT implementation via `#include "LT.h"` when writing the judge-facing solution.
- The header provides three diffusion routines tailored to different evaluation needs; the slides emphasize the *Signed LT* variants that account for both positive and negative influence.

### `influence_sum()`
- Computes the net influence on a candidate node by summing all positive neighbors' contributions and subtracting the negative neighbors' impact.
- Internally calls `getEdgeInfluence(u, target)` for each neighbor `u` to accumulate its outgoing edge weight toward the target node.
- Returns a `double` representing the final (positive minus negative) influence value that will be compared against the node's threshold.

### `diffuse_one_round_signed()`
- Executes a single synchronous round of the Signed LT diffusion.
- For each inactive node, gathers the sets of positive and negative active neighbors (`pos_in`, `neg_in`), computes the thresholds `inf` and `inf2` using `influence_sum`, and determines whether the node flips to positive or negative in this round.
- Returns the newly positive and negative activations so the caller can merge them into the global active sets.

### `diffuse_signed_all()`
- Wraps `diffuse_one_round_signed()` in a loop until no further activations occur.
- Maintains separate containers for positive and negative active nodes, repeatedly merges each round's new activations, and finally outputs the stabilized sets (positive first, then negative).

## Judge Information
- Online judge error codes: WA, QU, RTE (Segmentation Fault, Floating Point Exception), CE, TLE, MLE, OLE, RE, System error, IE.
- Compilation uses GCC with flags listed on judge FAQ; enabling ONLINE_JUDGE macro is recommended.

## `seedSelection` Requirements (Slide 37)
- Signature must exactly match `unordered_set<int> seedSelection(DirectedGraph &G, unsigned int numberOfSeeds);` — no additional parameters or default arguments.
- The function returns the chosen initial active users whose cascade is evaluated by the judge.

## Ranking Criteria (Slide 38)
- `Active Rate = (positive_active_users_at_end - negative_active_users_at_end) / total_users_in_network`.
- Example: if positive = 8,000, negative = 2,000, total = 10,000 ⇒ Active Rate = (8,000 − 2,000) / 10,000 = 60%.
- Final leaderboard rankings average this Active Rate across all datasets.

## Implementation Notices (Slide 39)
- Do **not** print (e.g., `cout`) inside `seedSelection`.
- Do **not** call `system("pause")` or use blocking input such as `cin`.
- The returned `unordered_set` must contain exactly `numberOfSeeds` elements.

## Contest Restrictions (Slide 40)
- **Time limit**: each dataset's `seedSelection` execution must finish within 12 minutes.
- **Memory limit**: peak usage per dataset may not exceed 2 GB.
- **Submission quota**: at most 5 submissions per day.
- **Blocking rule**: a new submission cannot be uploaded until the previous one has finished judging.

