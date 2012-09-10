#include <vector>
#include <utility>
#include <map>
#include <iostream>


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
   boost::property< boost::vertex_index2_t, int>, boost::property<boost::edge_weight_t, float> > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_traits<WeightMap>::value_type EdgeWeight;

typedef boost::property_map<Graph, boost::vertex_index2_t>::type IndexMap;
typedef boost::property_traits<IndexMap>::value_type Index;

typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;

int main(){

	using namespace std;

	std::vector<int> source_layer = {1, 3, 5 , 9, 10, 11};

	std::vector<int> faux_neighbours = {1, 3, 5 , 9, 10, 11};

	std::map<Vertex, int> IVMap;

	boost::shared_ptr<Graph> graph;
	graph = boost::shared_ptr<Graph>(new Graph());

	IndexMap vertex_index_map = get(boost::vertex_index2, *graph);
    WeightMap edge_weight_map = get(boost::edge_weight, *graph);

	for(int i : source_layer){
		
		Vertex v = add_vertex(*graph);
		IVMap[i] = v;
        vertex_index_map[v] = i;
	}

	std::pair<vertex_iter, vertex_iter> vp;

	for (vp = vertices(*graph); vp.first != vp.second; ++vp.first) {

		Vertex v1 = *vp.first;
		int v1idx = vertex_index_map[v1];

		for(int n : faux_neighbours){
			// find vertex for neighbour
			Vertex v2 = IVMap[n];
			Edge e = (add_edge(v1, v2, *graph)).first;
			edge_weight_map[e] = 299.1f/n;
		}
	}


	BOOST_AUTO(parities, boost::make_one_bit_color_map(num_vertices(*graph), get(boost::vertex_index, *graph)));

	float w = boost::stoer_wagner_min_cut(*graph, get(boost::edge_weight, *graph), boost::parity_map(parities));

	cout << "The min-cut weight of G is " << w << ".\n" << endl;
	//assert(w == 7);

	cout << "One set of vertices consists of:" << endl;
	size_t i;
	for (i = 0; i < num_vertices(*graph); ++i) {
	if (get(parities, i))
	  cout << i << " " << vertex_index_map[i] << endl;
	}
	cout << endl;

	cout << "The other set of vertices consists of:" << endl;
	for (i = 0; i < num_vertices(*graph); ++i) {
	if (!get(parities, i))
	  cout << i << " " << vertex_index_map[i] << endl;
	}
	cout << endl;

	return EXIT_SUCCESS;

}