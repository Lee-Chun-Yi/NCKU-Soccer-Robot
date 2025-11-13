#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
using namespace std;

/* ===============================================================
 * Node / Edge 結構定義
 * =============================================================== */
struct Edge {
    int from_node, to_node;
    double influence;
    bool active; 
    Edge(int f, int t, double inf)
        : from_node(f), to_node(t), influence(inf), active(true) {}
};

struct Node {
    int id;
    double pos_th, neg_th;
    vector<Edge*> in_edges;
    vector<Edge*> out_edges;
    Node(int _id = 0, double p = 0, double n = 0)
        : id(_id), pos_th(p), neg_th(n) {}
};

/* ===============================================================
 * DirectedGraph 類別
 * =============================================================== */
class DirectedGraph {
private:
    unordered_map<int, Node*> nodeMap;
    vector<Edge*> allEdges;
    int number_of_nodes = 0;
    int number_of_edges = 0;

public:
    DirectedGraph() {}
    ~DirectedGraph() {
        for (auto& kv : nodeMap)
            delete kv.second;
        for (auto e : allEdges)
            delete e;
    }

    /* ===================== 深拷貝與指定運算 ===================== */
    DirectedGraph(const DirectedGraph& other) {
        number_of_nodes = other.number_of_nodes;
        number_of_edges = other.number_of_edges;

        for (auto& kv : other.nodeMap) {
            Node* n = kv.second;
            nodeMap[kv.first] = new Node(n->id, n->pos_th, n->neg_th);
        }

        for (auto e : other.allEdges) {
            Edge* newE = new Edge(e->from_node, e->to_node, e->influence);
            newE->active = e->active;
            allEdges.push_back(newE);
            if (nodeMap.count(e->from_node))
                nodeMap[e->from_node]->out_edges.push_back(newE);
            if (nodeMap.count(e->to_node))
                nodeMap[e->to_node]->in_edges.push_back(newE);
        }
    }

    DirectedGraph& operator=(const DirectedGraph& other) {
        if (this == &other) return *this;

        for (auto& kv : nodeMap)
            delete kv.second;
        for (auto e : allEdges)
            delete e;
        nodeMap.clear();
        allEdges.clear();

        number_of_nodes = other.number_of_nodes;
        number_of_edges = other.number_of_edges;

        for (auto& kv : other.nodeMap) {
            Node* n = kv.second;
            nodeMap[kv.first] = new Node(n->id, n->pos_th, n->neg_th);
        }

        for (auto e : other.allEdges) {
            Edge* newE = new Edge(e->from_node, e->to_node, e->influence);
            newE->active = e->active;
            allEdges.push_back(newE);
            if (nodeMap.count(e->from_node))
                nodeMap[e->from_node]->out_edges.push_back(newE);
            if (nodeMap.count(e->to_node))
                nodeMap[e->to_node]->in_edges.push_back(newE);
        }
        return *this;
    }

    /* ========================= 基本操作 ========================= */
    void addNode(int id, double pos_th = 0.0, double neg_th = 0.0) {
        if (!nodeMap.count(id)) {
            nodeMap[id] = new Node(id, pos_th, -std::fabs(neg_th));
            number_of_nodes++;
        } else {
            nodeMap[id]->pos_th = pos_th;
            nodeMap[id]->neg_th = -std::fabs(neg_th);
        }
    }

    void addEdge(int u, int v, double inf) {
        if (!nodeMap.count(u)) addNode(u);
        if (!nodeMap.count(v)) addNode(v);
        Edge* e = new Edge(u, v, inf);
        nodeMap[u]->out_edges.push_back(e);
        nodeMap[v]->in_edges.push_back(e);
        allEdges.push_back(e);
        number_of_edges++;
    }

    /* ========================= getter 區 ========================= */
    double getNodeThreshold(int id) const {
        if (!nodeMap.count(id) || nodeMap.at(id) == nullptr) return 0;
        return nodeMap.at(id)->pos_th;
    }

    double getNodeThreshold2(int id) const {
        if (!nodeMap.count(id) || nodeMap.at(id) == nullptr) return 0;
        return nodeMap.at(id)->neg_th;
    }

    vector<int> getNodeInNeighbors(int id) const {
        vector<int> res;
        if (!nodeMap.count(id) || nodeMap.at(id) == nullptr) return res;
        for (auto e : nodeMap.at(id)->in_edges)
            if (e && e->active) res.push_back(e->from_node);
        return res;
    }

    vector<int> getNodeOutNeighbors(int id) const {
        vector<int> res;
        if (!nodeMap.count(id) || nodeMap.at(id) == nullptr) return res;
        for (auto e : nodeMap.at(id)->out_edges)
            if (e && e->active) res.push_back(e->to_node);
        return res;
    }

    double getEdgeInfluence(int u, int v) const {
        if (!nodeMap.count(u) || nodeMap.at(u) == nullptr) return 0;
        for (auto e : nodeMap.at(u)->out_edges)
            if (e && e->active && e->to_node == v)
                return e->influence;
        return 0;
    }

    vector<int> getAllNodes() const {
        vector<int> nodes;
        for (auto& kv : nodeMap)
            nodes.push_back(kv.first);
        return nodes;
    }

    int getSize() const { return number_of_nodes; }
    int getEdgeNumber() const { return number_of_edges; }

    bool isNodeExist(int id) const { return nodeMap.count(id); }

    bool isEdgeExist(int u, int v) const {
        if (!nodeMap.count(u)) return false;
        for (auto e : nodeMap.at(u)->out_edges)
            if (e && e->active && e->to_node == v)
                return true;
        return false;
    }

    vector<pair<int,int>> getAllEdges() const {
        vector<pair<int,int>> edges;
        for (auto e : allEdges)
            if (e && e->active)
                edges.emplace_back(e->from_node, e->to_node);
        return edges;
    }

    /* ==================== Safe Delete Functions ==================== */
    bool deleteEdge(int u, int v) {
        if (!isEdgeExist(u, v)) return false;

        auto &outs = nodeMap[u]->out_edges;
        outs.erase(remove_if(outs.begin(), outs.end(),
            [&](Edge* e) {
                if (e->to_node == v) { e->active = false; return true; }
                return false;
            }), outs.end());

        auto &ins = nodeMap[v]->in_edges;
        ins.erase(remove_if(ins.begin(), ins.end(),
            [&](Edge* e) {
                if (e->from_node == u) { e->active = false; return true; }
                return false;
            }), ins.end());

        for (auto e : allEdges)
            if (e && e->from_node == u && e->to_node == v)
                e->active = false;

        number_of_edges--;
        return true;
    }

    bool deleteNode(int id) {
        if (!isNodeExist(id)) return false;
        Node* n = nodeMap[id];
        if (!n) return false;

        for (auto e : n->in_edges)
            if (e) e->active = false;
        for (auto e : n->out_edges)
            if (e) e->active = false;

        n->in_edges.clear();
        n->out_edges.clear();
        delete n;
        nodeMap.erase(id);
        number_of_nodes--;
        return true;
    }
};

#endif
