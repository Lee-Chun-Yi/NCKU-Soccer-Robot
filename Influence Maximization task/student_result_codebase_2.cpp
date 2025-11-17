#include <iostream>
#include <fstream>
#include <iomanip>
#include <unordered_set>
#include "graph.h"
#include "LT.h"
#include "student_submit.h"
using namespace std;

#define TOTAL_POS_SEEDS 10  // given_pos(1) + student select(9)

DirectedGraph loadGraph(string dataFolder);
double activeRate(DirectedGraph&, string);

int main(int argc, char* argv[]) {
	DirectedGraph G = loadGraph(argv[1]);
	double ar = activeRate(G, argv[1]);
	cout << "Active_Rate=" << fixed << setprecision(5) << ar << endl;
	return 0;
}


/* Load network from given data directory */
DirectedGraph loadGraph(string dataFileDirectory) {
	DirectedGraph G;

	// ===== Read node.txt =====
	string node_file = dataFileDirectory + "/node.txt";
	ifstream f1(node_file);
	if (!f1.is_open()) {
		cerr << "Cannot open node file: " << node_file << endl;
		exit(EXIT_FAILURE);
	}

	int node_count;
	f1 >> node_count; // first line ignored
	int node;
	double pos_th, neg_th;
	while (f1 >> node >> pos_th >> neg_th) {
		G.addNode(node, pos_th, neg_th);
	}
	f1.close();

	// ===== Read edge.txt =====
	string edge_file = dataFileDirectory + "/edge.txt";
	ifstream f2(edge_file);
	if (!f2.is_open()) {
		cerr << "Cannot open edge file: " << edge_file << endl;
		exit(EXIT_FAILURE);
	}

	string tmp;
	f2 >> tmp; // ignore first token

	while (f2 >> tmp) {
		int u = stoi(tmp);
		f2 >> tmp;
		int v = stoi(tmp);
		f2 >> tmp;
		double w = stod(tmp);
		G.addEdge(u, v, w);
	}
	f2.close();

	return G;
}


/*
 * Compute active rate: (#pos - #neg) / N
 */
double activeRate(DirectedGraph& G, string folder) {

	// ===== Read given_pos.txt =====
	string given_pos_file = folder + "/given_pos.txt";
	ifstream gp(given_pos_file);
	if (!gp.is_open()) {
		cerr << "Cannot open given_pos file: " << given_pos_file << endl;
		exit(EXIT_FAILURE);
	}
	int givenPosSeed;
	gp >> givenPosSeed;
	gp.close();

	// ===== Read neg_seed.txt =====
	string neg_file = folder + "/neg_seed.txt";
	ifstream gn(neg_file);
	if (!gn.is_open()) {
		cerr << "Cannot open neg_seed file: " << neg_file << endl;
		exit(EXIT_FAILURE);
	}
	unordered_set<int> givenNegSeeds;
	int x;
	while (gn >> x) {
		givenNegSeeds.insert(x);
	}
	gn.close();

	// ===== Call student's seedSelection =====
	unsigned int need = TOTAL_POS_SEEDS - 1; // student selects 9
	unordered_set<int> studentSeeds =
		seedSelection(G, need, givenPosSeed, givenNegSeeds);

	if (studentSeeds.size() != need) {
		cerr << "[ERROR] Wrong #seeds. Need " << need
			<< ", got " << studentSeeds.size() << endl;
		exit(EXIT_FAILURE);
	}

	// final positive set = given + student
	unordered_set<int> pos_seeds;
	pos_seeds.insert(givenPosSeed);
	pos_seeds.insert(studentSeeds.begin(), studentSeeds.end());

	// ===== LT diffusion =====
	unordered_set<int> final_pos, final_neg;
	diffuse_signed_all(&G, pos_seeds, givenNegSeeds, final_pos, final_neg);

	int score = (int)final_pos.size() - (int)final_neg.size();

	int N = G.getSize();
	if (N == 0) return 0.0;
	return (double)score / (double)N;
}
