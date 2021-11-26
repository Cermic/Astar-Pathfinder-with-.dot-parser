/*
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================


Modified by Jack Smith (B00308927) and Greg Smith (B00308929) for use in 
a project that searches through an undirected weighted graph.

The information on the graph is read into the program by a .dot language 
parser.
*/
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h> 
#include <ctime>
#include <cstdio>
#include <filesystem>

using namespace boost;
using namespace std;
namespace fs = std::filesystem;

// auxiliary types
struct location
{
	float y, x; // lat, long
};
typedef float cost;

template <class Name, class LocMap>
class city_writer {
public:
	city_writer(Name n, LocMap l, float _minx, float _maxx,
		float _miny, float _maxy,
		unsigned int _ptx, unsigned int _pty)
		: name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
		maxy(_maxy), ptx(_ptx), pty(_pty) {}
	template <class Vertex>
	void operator()(ostream& out, const Vertex& v) const {
		float px = 1 - (loc[v].x - minx) / (maxx - minx);
		float py = (loc[v].y - miny) / (maxy - miny);
		out << "[label=\"" << name[v] << "\", pos=\""
			<< static_cast<unsigned int>(ptx * px) << ","
			<< static_cast<unsigned int>(pty * py)
			<< "\", fontsize=\"11\"]";
	}
private:
	Name name;
	LocMap loc;
	float minx, maxx, miny, maxy;
	unsigned int ptx, pty;
};

template <class WeightMap>
class time_writer {
public:
	time_writer(WeightMap w) : wm(w) {}
	template <class Edge>
	void operator()(ostream &out, const Edge& e) const {
		out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
	}
private:
	WeightMap wm;
};


// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class euclidean_distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	euclidean_distance_heuristic(LocMap l, Vertex goal)
		: m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return ::sqrt(dx * dx + dy * dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};

// manhattan distance heuristic
template <class Graph, class CostType, class LocMap>
class manhattan_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	manhattan_heuristic(LocMap l, Vertex goal)
		: m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return (dx + dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};

struct found_goal {}; // exception for termination
				  // visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};

int main(int argc, char **argv)
{
	// specify some types
	typedef adjacency_list<listS, vecS, undirectedS, no_property,
		property<edge_weight_t, cost> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef std::pair<int, int> edge;

	location* locations = NULL;   // Pointer to int, initialize to nothing.
	edge* edge_array = NULL;
	cost* weights = NULL;
	int* nodes = NULL;

	std::string line; // String for accepting line input from the file.
	int weightCounter = 0; // a counter for the weights to increment with
	int posCounter = 0; // a counter for the positions to increment with
	int nodeCounter = 0; // a counter to count all the nodes in the file
	int edgeCounter = 0; // a counter to count all the edges in the file
	string fileName = fs::current_path().parent_path().string() + "/files/random64_4_1517441833.dot";
	ifstream dotFile;

	dotFile.open(fileName.c_str());
	while (!dotFile.is_open())
	{
		cout << "File not opened, did you type the path correctly?" << "\n"
			"Please try again." << "\n" << endl;
		cin >> fileName;
		dotFile.open(fileName.c_str());
	}
	 /*
	 This loop counts all the lines that contain certain string portions and sets the size of the
	 of the arrays appropriately. This makes it dynamic and allows the file to be of varying size.

	 weightCount for weights
	 posCount for locations
	 nodeCount for nodes
	 edgeCount for edges

	 */
	if (dotFile.is_open())
	{
		int weightCount = 0; // holds the total number of weights
		int posCount = 0;   // holds the total number of positions
		int nodeCount = 0;  // holds the total number of nodes
		int edgeCount = 0;	// holds the total number of edges

		while (getline(dotFile, line))
		{
			// When a tag is found the relevant count is incremented
			if (line.find(",pos=") != string::npos)
			{
				posCount++;
				nodeCount++;
			}

			if (line.find("--") != string::npos)
			{
				weightCount++;
				edgeCount++;
			}
		}

		// The array's size is defined by the count found earlier by the number of entries in the file
		locations = new location[posCount]; // Allocate n ints and save ptr in a. // long / lat - ( X / Y )
		for (int i = 0; i < posCount; i++)
		{
			locations[i].x = 0;
			locations[i].y = 0;	// Initialize all elements to zero.
		}

		weights = new cost[weightCount]; // Allocate n ints and save ptr in a. // weights of edges
		for (int i = 0; i < weightCount; i++)
		{
			weights[i] = 0;	// Initialize all elements to zero.
		}

		edge_array = new edge[edgeCount];
		for (int i = 0; i < edgeCount; i++)
		{
			edge_array[i] = edge(0, 0);
		}

		nodes = new int[nodeCount];
		for (int i = 0; i < nodeCount; i++)
		{
			nodes[i] = 0;
		}
		dotFile.close(); // When no lines are left close the file
	}// File checking is concluded.

	/*
		The file is reopened to allow parsing through it to populate the various arrays that will be 
		traversed for use in generating a path through the graph.
	*/
	dotFile.open(fileName.c_str());
	if (dotFile.is_open())// If the file is open
	{
		weightCounter = 0; // a counter for the weights to increment with
		posCounter = 0; // a counter for the positions to increment with
		nodeCounter = 0; // a counter for the nodes to increment with
		edgeCounter = 0; // a counter for the edges to increment with

		while (getline(dotFile, line))// Get the line
		{
			if (line.find(",pos=") != string::npos)
			{
				std::size_t pos = line.find(','); // finds the first mention of , on the line
				std::string split = line.substr(pos); // splits the string into a substring at pos
				std::size_t findFirst = split.find_first_of('"'); // finds the position of the first " in the string
				std::size_t findComma = split.find_last_of(','); // finds position of the last ' on in the string
				std::size_t findLast = split.find_last_of('"'); // finds the position of the last " in the string

				std::string xCoord = split.substr(findFirst + 1, ((findComma - 1) - findFirst));
				// Assigns weight the characters starting at findFirst+1 which is the character after the " until the position 
				// of the , -1 . This should include all of the x coord

				std::string yCoord = split.substr(findComma + 1, ((findLast - 1) - findComma));
				// Assigns weight the characters starting at findComma +1 which is the character after the " until the position 
				// of the last " -1 . This should include all of the x coord

				locations[posCounter].x = std::strtof((xCoord).c_str(), 0); // Inputs the number found to the x cord of the position value
				locations[posCounter].y = std::strtof((yCoord).c_str(), 0); // Inputs the number found to the y cord of the position value
				posCounter++;

				std::size_t pos2 = line.find_first_of('['); // Finds the position of the first [ in the string
				std::string nodeFound = line.substr(0, (pos2)); // Sets the nodeFound string to the starting position on the string to pos2
				nodes[nodeCounter] = std::stoi(nodeFound);// inserts node found at the nodecounter position in the array.
				nodeCounter++;
			}

			if (line.find("--") != string::npos)
			{
				std::size_t foundFirstEdge = line.find_first_of('-');// Gets the position of the first -
				std::string fNSplit = line.substr(0, (foundFirstEdge));// Sets the string to the first position until the position of foundFirstEdge on the line
				std::size_t foundSecondEdge = line.find_last_of('-'); // Gets the position of the first - from the back of the string
				std::size_t foundFirstWhiteSpace = line.find_first_of(' '); // finds the first whitespace position
				std::string sNSplit = line.substr((foundSecondEdge + 1), (foundFirstWhiteSpace - 3)); // Sets the string to the foundSecondEdge position until the position of firstFoundEdge on the line

				edge_array[edgeCounter] = edge(std::stoi(fNSplit), std::stoi(sNSplit)); // inserts an edge into the array at edgeCounter position
				edgeCounter++;

				if (line.find(',') != string::npos)
				{
					std::size_t pos = line.find(','); // finds the first mention of , on the line
					std::string split = line.substr(pos); // splits the string into a substring at pos
					std::size_t findFirst = split.find_first_of('"'); // finds the position of the first " in the string
					std::size_t findLast = split.find_last_of('"'); // finds the position of the last " in the string

					std::string weight = split.substr(findFirst + 1, ((findLast - 1) - findFirst));
					// Assigns weight the characters starting at findFirst+1 (which is the character after the " until the position 
					// of ((findLast-1) - findFirst)) which is the difference between the two corrected to extract only the numbers
					// this makes sure the amount of numbers between them is irrelevant.

					weights[weightCounter] = std::strtof((weight).c_str(), 0); // Sets the weight of the node to the position of weight counter in the array
					weightCounter++;
				}
			}
		}

		dotFile.close(); // When no lines are left close the file
	}

	else cout << "Unable to open file" << "\n" << endl; // Print if file cannot be opened for any reason.

	// create graph
	mygraph_t g(nodeCounter);
	WeightMap weightmap = get(edge_weight, g);
	for (std::size_t j = 0; j < edgeCounter; ++j) {
		edge_descriptor e; bool inserted;
		boost::tie(e, inserted) = add_edge(edge_array[j].first,
			edge_array[j].second, g);
		weightmap[e] = weights[j];
	}
	// User defines start and end vertexs.
	int startVertex = 0, endVertex = 0;
	/*
		Ensures the input given won't be allowed to be lower than 0
		or higher than the number of nodes in the graph.
	*/
	do
	{
		if (startVertex < 0 || startVertex >(nodeCounter - 1))
		{
			cout << "Invalid input, please input start vertex again" << "\n" << endl;
		}
		cout << "Define start vertex between " << "0 " << "and " << (nodeCounter - 1) << endl;
		cin >> startVertex;
	} while (startVertex < 0 || startVertex > (nodeCounter-1));

	/*
		Ensures the input given won't be allowed to be lower than 0
		or higher than the number of nodes in the graph.
	*/
	do
	{
		if (endVertex < 0 || endVertex >(nodeCounter - 1))
		{
			cout << "Invalid input, please input start vertex again" << "\n" << endl;
		}
		cout << "Define end vertex between " << "0 " << "and " << (nodeCounter - 1) << endl;
		cin >> endVertex;
	} while (endVertex < 0 || endVertex >(nodeCounter - 1));

	vertex start = (g, startVertex);
	vertex goal = (g, endVertex);

	cout << "\n" << "Start vertex: " << nodes[start] << endl;
	cout << "\n" << "Goal vertex: " << nodes[goal] << "\n" <<endl;

	ofstream dotfile;
	dotfile.open("test-astar-cities.dot");
	write_graphviz(dotfile, g,
		city_writer<int*, location*>
		(nodes, locations, 73.46, 78.86, 40.67, 44.93,
			480, 400),
		time_writer<WeightMap>(weightmap));

	//start timer
	clock_t begin;
	double duration;
	begin = clock();

	vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
	vector<cost> d(num_vertices(g));

	//Euclidean distance heuristic//
	try {
		// call astar named parameter interface
		astar_search_tree
		(g, start,
			euclidean_distance_heuristic<mygraph_t, cost, location*>
			(locations, goal),
			predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
			distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
			visitor(astar_goal_visitor<vertex>(goal)));
	}
	catch (found_goal fg) { // found a path to the goal
		list<vertex> shortest_path;
		for (vertex v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}
		cout << "Using Euclidean Distance hueristic:" << "\n";
		cout << "Shortest path from " << nodes[start] << " to "
			<< nodes[goal] << ": ";
		list<vertex>::iterator spi = shortest_path.begin();
		cout << nodes[start];
		for (++spi; spi != shortest_path.end(); ++spi)
			cout << " -> " << nodes[*spi];
		cout << "\n" << "Total travel time: " << d[goal] << "\n" << endl;

		//timer
		duration = (clock() - begin) / (double)CLOCKS_PER_SEC;
		cout << "Benchmark: " << duration << " seconds" << '\n'<< endl;
		begin = clock();
	}

	//manhattan_distance//
	try {
		// call astar named parameter interface
		astar_search_tree
		(g, start,
			manhattan_heuristic<mygraph_t, cost, location*>
			(locations, goal),
			predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
			distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
			visitor(astar_goal_visitor<vertex>(goal)));
	}
	catch (found_goal fg) { // found a path to the goal
		list<vertex> shortest_path;
		for (vertex v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}
		cout << "Using Manhattan distance hueristic:" << "\n";
		cout << "Shortest path from " << nodes[start] << " to "
			<< nodes[goal] << ": ";
		list<vertex>::iterator spi = shortest_path.begin();
		cout << nodes[start];
		for (++spi; spi != shortest_path.end(); ++spi)
			cout << " -> " << nodes[*spi];
		cout << "\n" << "Total travel time: " << d[goal] << "\n" << endl;
		//timer
		duration = (clock() - begin) / (double)CLOCKS_PER_SEC;
		cout << "Benchmark: " << duration << " seconds"<<'\n' << endl;

		delete locations; // Freeing memory
		locations = NULL; // Clearing to prevent invalid referencing
		delete weights;
		weights = NULL;
		delete edge_array;
		edge_array = NULL;
		delete nodes;
		nodes = NULL;

		system("pause");

		return 0;
	}//manhattan end//

	cout << "Didn't find a path from " << nodes[start] << " to "
		<< nodes[goal] << " !" << "\n" << endl;
	system("pause");
	delete locations; // Freeing memory
	locations = NULL; // Clearing to prevent invalid referencing
	delete weights;
	weights = NULL;
	delete edge_array;
	edge_array = NULL;
	delete nodes;
	nodes = NULL;
	return 0;
}