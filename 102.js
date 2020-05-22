// CPCS 324 Algorithms & Data Structures 2
// Graph data structure starter - Transitive Closure Package
// 2018, Dr. Muhammad Al-Hashimi
// -----------------------------------------------------------------------
// simple graph object with linked-list edge implementation and minimal fields
// extra vertex and edge mem    ber fields and methods to be added later as needed
// note naming conventions in upload guide
var _v = [],
    _e = []; // exercise 9.2

// -----------------------------------------------------------------------
function main_graph()

{
    // create a graphs
    var g = new Graph();

    // set graphs properties
    g.label = 'Exercise 9.2: 1b (Levitin, 3rd edition)';

    // use input arrays _v and _e to initialize its internal data structures
    g.readGraph(_v, _e);

    // use print_graph() method to check graph report connectivity status if available
    g.printGraph();

    // perform depth-first search and output stored result
    g.connectedComp = g.topoSearch(g.connectedComp);
    document.write("<p>bfs_order: ", g.dfs_push, "</p>");


    //check graph connectivity status if available
    document.write("<p>", g.componentInfo(), "<p>");


    // perform breadth-first search and output stored result
    g.dfsORbfs = true;
    g.topoSearch(g.connectedComp);
    document.write("<p>bfs_order: ", g.bfs_order, "</p>");




    // output the warshall matrix
    document.write("<p>Transitive closure</br>");
    g.warshallFloyd();

    if (g.adjMatrix.tc != undefined) // that means if is digraph , if not digrap tc = undefined
    {
        for (var i = 0; i < g.adjMatrix.tc.length; i++)
        {
            document.write(g.adjMatrix.tc[i], "</br>");
        }
    }
    else
    {
        document.write("Graph is not digraph ,Transitive Closure is not applicable</p>");
    }



    //output the floyd matrix
    document.write("<p>Distance matrix</p>");

    if (g.adjMatrix.disc != undefined) // that means if is weighted , if not weighted disc = undefined
    {
        for (var i = 0; i < g.adjMatrix.disc.length; i++)
        {
            document.write(g.adjMatrix.disc[i], "</br>");
        }
    }
    else
    {
        document.write("Graph is not weighted , Distance matrix is not applicable </p>");
    }



    //perform MST by Prim2 (linear PQ)
    // check if graph weighted and undirgrap
    document.write("<p>MST by Prim2 (linear PQ)<br>");
    if (!g.digraph && g.weighted)
    {
        g.prim2();
    }
    else
    {
        document.write("Graph is not weighted or is digraph ,prim is not applicable</p>");
    }



    //output of Dijkstra
    if (g.weighted)
    {
        var distance = g.Dijkstra(0);
        document.write("<p>Distance matrix from Dijkstra<br>");

        for (var j = 1; j < g.nv + 1; j++)
        {
            for (var i = 0; i < distance.length; i++)
            {
                document.write(distance[i], ",");
            }

            document.write("</br>");
            distance = g.Dijkstra(j);
        }
    }
    else
    {
        document.write("Graph is not weighted, Dijkstra is not applicable</p>");
    }
}

// -----------------------------------------------------------------------

function Vertex(v)
{
    // pr operty fields
    this.label = v.label; // vertex can be labelled
    this.visit = false; // vertex can be marked visited or "seen"
    this.adjacent = new List(); // init an adjacency list

    // --------------------
    // base member methods
    this.adjacentByID = adjacentByIdImpl; //  Get id of adjacent vertices in an array.
    this.incidentEdges = incidentEdgesImpl; // Get information of incident edges in an array of custom objects
    this.vertexInfo = vertexInfoImpl; //Get printable vertex info strings, details depend on implementation.
    this.insertAdjacent = insertAdjacentImpl; //
}

// -----------------------------------------------------------------------
function Edge(vert_i, weight)
{
    // base property fields
    this.target_v = vert_i;

    // base member methods
    this.weight = weight;
}



// -----------------------------------------------------------------------
function Graph()
{
    // property fields
    this.vert = []; // vertex list (an array of Vertex objects)
    this.nv = 0; // number of vertices
    this.ne = 0; // number of edges
    this.weighted = false;
    this.dfs_push = []; // DFS order output
    this.bfs_order = []; // BFS order output
    this.label = ""; // identification string to label graph
    this.connectedComp = 0; // number of connected comps set by DFS; 0 (default) for no info
    this.adjMatrix = []; // graph adjacency matrix to be created on demand
    this.digraph = false; // true if digraph, false otherwise (default undirected)

    // base member methods
    this.listVerts = listVertsImpl;
    this.readGraph = better_input; // default input reader method
    this.addEdge = addEdgeImpl3;
    this.printGraph = better_output; //printGraphImpl;
    this.makeGraph = makeGraphImpl; //Not part of Project 1
    this.dfs = dfsImpl;
    this.bfs = bfsImpl;
    this.makeAdjMatrix = makeAdjMatrixImpl3;
    this.isConnected = isConnectedImpl;
    this.componentInfo = componentInfoImpl;
    this.topoSearch = topoSearchImpl;

    // --------------------
    // more student fields next
    this.dfsORbfs = false; // boolean,Use to determine which method to use in topoSearch , if true dfs
    this.dfsTCmat = [];
    this.Warshall = [];
    this.Floyd = []; // array with the minimum distances
    this.prim_e = [];
    this.VE = [];
    this.vt_h = [];
    this.VT = [];

    // transitive closure package (requirements in line comments, to be removed and replaced by JSDOCs)
    this.hasPath = hasPathImpl;
    this.shortestPath = shortestPathImpl;
    this.isDAG = isDAGImpl;
    this.warshallFloyd = WarshallFloydImpl;
    this.dfsTC = DfsTCImpl;
    this.Prim = primImpl;
    this.prim2 = primimpl2;
    this.Dijkstra = DijkstraImpl;

}



// -----------------------------------------------------------------------
// functions used by methods of Graph and ancillary objects
// -----------------------------------------------------------------------
// begin student code section
// -----------------------------------------------------------------------


// Transitive Closure package

/**

 * @methodOf Graph#
 * @method DijkstraImpl
 * @description This algorithm for single-source shortest-paths problem.
 *  this algortithm is applicable to undirect and direct grph with nonnegative weight only.
 * @param s - the inatial node.
 * @author Eiman Asghar.
 */
function DijkstraImpl(s)
{
    //initalize priority queue to empty.
    var PQ = new PQueue();

    //p its penultimate vertex for every vertex.
    var p = [];

    //the length d of a shortest path fom s(source) to v(current vertex).
    var d = [];

    //---------------------------------
    for (var j = 0; j < this.nv; j++)
    {
        //fill distance array (d)
        d[j] = Infinity;
        p[j] = null;

        //initalize vertex in the priority queue.
        PQ.insert(j, d[j]);
    }

    // update starter node (source) to 0.
    d[s] = 0;

    // update s in PQ.
    PQ.insert(s, d[s]);

    //---------------------------------
    // to store node from PQ
    var u;
    for (var i = 0; i < this.nv; i++)
    {
        //delete the minmum priority queue.
        u = PQ.deleteMin();

        // store the current node.
        this.VT[i] = u.item;

        // find adjacent of the current node
        var adj = this.vert[this.VT[i]].incidentEdges();

        for (var k = 0; k < adj.length; k++)
        {
            // find the shortest path and then update the value.
            if (u.prior + adj[k].edgeWeight < d[adj[k].adjvert_i])
            {

                d[adj[k].adjvert_i] = u.prior + adj[k].edgeWeight;
                p[adj[k].adjvert_i] = u.item;

                // update in PQ.
                PQ.insert(adj[k].adjvert_i, d[adj[k].adjvert_i]);
            }
        }
    }

    //---------------------------------
    // print the shortert path.
    if (s == 0)
    {

        document.write("</p>Shortest paths by Dijkstra from vertex ", s, "<br>");

        for (var l = 0; l < this.nv - 1; l++)
        {
            document.write(d[this.VT[l]], "(" + ((p[this.VT[l]] == undefined) ? "-" : p[this.VT[l]]) + "," + this.VT[l] + ") ,");
        }

        document.write(d[this.VT[l]], "(" + ((p[this.VT[l]] == undefined) ? "-" : p[this.VT[l]]) + "," + this.VT[l] + ").");



    }
    return d;
}



/**
 * @methodOf Graph#
 * @author Waad alshanbari
 * @description this function return the MST by the prim's second implementation
 * @input weigted connected graph
 * @returns MTS of graph
 */

function primimpl2()
{
    var p = 0; //parent varible
    //initialize a priority queue , line 1 in the algorthim
    var PQ = new PQueue();
    this.VE = this.vert;

    // mark all vertices unvisited
    for (var i = 0; i < this.nv; i++)
    {
        this.VE[i].visit = false;
    }

    //insert adajacent vertices of first vertex into the PQ ,line 2
    var edges = this.VE[0].incidentEdges();
    for (var i = 0; i < edges.length; i++)
    {
        var u = edges[i].adjvert_i; // u is the target
        var w = edges[i].edgeWeight; // target weight
        PQ.insert(u, w);
    }

    //VT contains visited vertices(target and weight)
    this.VT[this.VT.length] = {
        v_t: 0, // line 4
        vt_p: null //line  5
    }
    this.VE[0].visit = true;
    this.vt_h = []; //VT helper contains all the visited source and its weight

    for (var i = 0; i < this.nv - 1; i++)
    {
        var u = PQ.deleteMin();
        //delete node if its already visited
        while (this.VE[u.item].visit)
            u = PQ.deleteMin();
        var edges = this.VE[u.item].incidentEdges(); //to expand the min weight and traverse

        //update the parent if the weight is equal to min in the PQ
        for (var j = 0; j < edges.length; j++)

        {
            if (this.VE[edges[j].adjvert_i].visit)
            {
                if (edges[j].edgeWeight === u.prior)
                {
                    p = edges[j].adjvert_i; //change parent
                    break; //to stop updating if there is a match in weight
                }
            }
        }
        //if unvisited update the vt
        if (!this.VE[u.item].visit)
        {
            this.VT[this.VT.length] = { //line 10
                v_t: u.item,
                vt_p: p
            };

            this.VE[u.item].visit = true; //mark  visited

            //update the vt helper(source and  weight)
            this.vt_h[this.vt_h.length] = new Edge(u.item, u.prior); //line 11



            //update the PQ ,line 12
            edges = this.VE[u.item].incidentEdges();
            for (var k = 0; k < edges.length; k++)
            {
                if (!this.VE[edges[k].adjvert_i].visit)
                {
                    u = edges[k].adjvert_i;
                    w = edges[k].edgeWeight;
                    PQ.insert(u, w);
                }
            }
        }
    }

    for (var i = 0; i < this.VT.length - 1; i++)
    {
        document.write("(", (this.VT[i].vt_p == undefined) ? "-" : this.VT[i].vt_p, ",", this.VT[i].v_t, ") ,");
    }

    document.write("(", (this.VT[i].vt_p == undefined) ? "-" : this.VT[i].vt_p, ",", this.VT[i].v_t, ").");
}



/**
 * @methodOf Graph#
 * @description this method to constructing a minimum spanning tree
 * @method primImpl
 * @author lama althbiti
 * @returns {array} array of object for edge
 *
 */

function primImpl()
{
    var VT = []; // create set of vertices tree
    var min = Infinity; // any number less than infinty


    // make all vertex unvisited
    for (var i = 0; i < this.nv; i++)
    {
        this.vert[i].visit = false;
    }

    // inatiat first index with first value on vert array
    VT[0] = this.vert[0];

    // mark it as visitedd
    this.vert[0].visit = true;

    for (var i = 0; i < this.nv; i++)
    {
        for (var j = 0; j < VT.length; j++)
        {
            var inc_v = VT[j].incidentEdges();
            for (var k = 0; k < inc_v.length; k++)
            {
                if (!this.vert[inc_v[k].adjVert_i].visit && inc_v[k].edgeWeight < min)
                {
                    this.prim_e[i] = // update
                        {
                            v: VT[j],
                            u: this.vert[inc_v[k].adjVert_i],
                            w: inc_v[k].edgeWeight
                        };

                    min = inc_v[k].edgeWeight; // change the minimum value
                }
            }
        }

        VT[VT.length] = this.prim_e[this.prim_e.length - 1].u;
        this.prim_e[this.prim_e.length - 1].u.visit = true;
        min = Infinity;
    }
}

/**
 * @methodOf Graph#
 * @description this method for copumting thr tansitive closer of digraph
 * @method WarshallFloyd
 * @author Lama althbitit
 * @returns inserts .tc field in adjacency matrix if digraph, and .dist if weighted
 */

//--------------------------------------------------
function WarshallFloydImpl()
{
    this.makeAdjMatrix();
    // fill Floyd and warshall by adjacent matrix
    for (var i = 0; i < this.adjMatrix.length; i++)
    {
        this.Floyd[i] = this.adjMatrix[i];
        this.Warshall[i] = this.adjMatrix[i];

        // check if there is relation between vertices
        for (var j = 0; j < this.nv; j++)
        {
            if (this.adjMatrix[i][j] == 0 && (i != j))
            {
                this.Floyd[i][j] = Infinity;
            }
        }
    }

    for (var k = 0; k < this.Warshall.length; k++)
    {
        for (var i = 0; i < this.Warshall.length; i++)
        {
            for (var j = 0; j < this.Warshall.length; j++)
            {
                if (this.digraph) // if graph is digraph filed Warshall
                {
                    this.Warshall[i][j] =
                        (this.Warshall[i][j] || (this.Warshall[i][k] && this.Warshall[k][j])) ? 1 : 0;
                }

                if (this.weighted) // if graph is weighted filed Floyd
                {
                    this.Floyd[i][j] =
                        Math.min(this.shortestPath(i, j), (this.shortestPath(i, k) + this.shortestPath(k, j)));
                }
            }
        }
    }

    for (var i = 0; i < this.Floyd.length; i++)
    {
        for (var j = 0; j < this.Floyd.length; j++)
        {
            if (this.Floyd[i][j] == Infinity)
            {
                this.Floyd[i][j] = 0;
            }
        }
    }

    this.adjMatrix = {
        tc: (this.digraph) ? this.Warshall : undefined, // if graph is not digraph tc = undefined
        disc: (this.weighted) ? this.Floyd : undefined // if graph is not weighted dist = undefined
    };
}

/**
 * @methodOf Graph#
 * @description this method find the direct and undirect edge between vertices
 * @method DfsTC
 * @author Lama althbiti
 * @returns return TC matrix for digraph based on a dfs
 */

function DfsTCImpl()
{
    // (note pattern) for each vertex id
    for (var i = 0; i < this.nv; i++)
    {
        // get vertex
        var v = this.vert[i];

        // make all vertices unvisited
        for (var j = 0; j < this.nv; j++)
        {
            this.vert[j].visit = false;
        }

        // create the corresponding row
        this.dfsTCmat[i] = [];

        for (var j = 0; j < this.nv; j++)
        {
            this.dfsTCmat[i][j] = 0; // init the corresponding row
        }

        // process adjacent vertices
        var w = v.adjacentByID();
        for (var k = 0; k < w.length; k++)
        {
            this.dfs(w[k]);
        }

        // for each such node, use to set matrix
        for (k = 0; k < this.nv; k++)
        {
            if (this.vert[k].visit)
            {
                this.dfsTCmat[i][k] = 1;
            }
        }
    }
}


/**
 * @methodOf Graph#
 * @method hasPathImpl
 * @param {integer} v_i source vertex id
 * @param {integer} v_j target vertex is
 * @author Lama Althbiti
 * @returns {boolean} return true if there is path between v_i and v_j otherways return false
 */
//--------------------------------------------
function hasPathImpl(v_i, v_j)
{
    return (this.Warshall[i][j] == 1) ? true : false;
}


/** 
 * @methodOf Graph#
 * @method shoetestPath
 * @param {integer} v_i  source vertex id
 * @param {integer} v_j  target vertex is
 * @author Lama althbitit
 * @returns {edge} distance of shortest path between v_i, v_j in weighted graph
 */
//------------------------------
function shortestPathImpl(v_i, v_j)
{
    return (this.Floyd[v_i][v_j]);
}


/**
 * @methodOf Graph#
 * @method isDAG
 * @author Lama althbitit
 * @returns {boolean} return true if there is direct acycle otherways return false
 */
//-------------------------------------
function isDAGImpl()
{
    for (var i = 0, j = 0; i < this.Warshall.length && j < this.Warshall.length; i++, j++)
    {
        if (this.hasPath(i, j))
        {
            return false;
        }
    }
    return true;
}

// -----------------------------------------------------------------------
// published docs section (ref. assignment page)
// use starter6-based P1M1 code as-is (fixes/improvements OK)
// no JSDOC comments in this section (docs already published)
// -----------------------------------------------------------------------

//Add an edge from a vertex pair and an optional weight.
//A better implementation which relies on a vertex method to handle adjacency details.


function addEdgeImpl(u_i, v_i) // obsolete, Add an edge from a vertex pair
{

    // fetch edge vertices using their id, where u: source vertex, v: target vertex
    var u = this.vert[u_i];
    var v = this.vert[v_i];

    // insert (u,v), i.e., insert v (by id) in adjacency list of u
    u.adjacent.insert(v_i);

    // insert (v,u) if undirected graph (repeat above but reverse vertex order)
    if (!this.digraph)
    {
        v.adjacent.insert(u_i);
    }
}



//------------------------
function addEdgeImpl2(u_i, v_i, weight) //obsolete ,Add an edge from a vertex pair and optional weight
{
    // fetch vertices using their id, where u: edge source vertex, v: target vertex
    var u = this.vert[u_i];
    var v = this.vert[v_i];

    // insert (u,v), i.e., insert v in adjacency list of u
    // (first create edge object using v_i as target, then pass object)
    var edge = new Edge(v_i, weight);
    u.adjacent.insert(edge);

    // insert (v,u) if undirected graph (repeat above but reverse vertex order)
    if (!this.digraph)
    {
        edge = new Edge(u_i, weight);
        v.adjacent.insert(edge);
    }
}

//-----------------------------
//Add an edge from a vertex pair and an optional weight.
//A better implementation which relies on a vertex method to handle adjacency details.
function addEdgeImpl3(u_i, v_i, weight)
{
    // fetch vertices using their id, where u: edge source vertex, v: target vertex
    var u = this.vert[u_i];
    var v = this.vert[v_i];

    // insert (u,v)
    u.insertAdjacent(v_i, weight); // fix encapsulation issue in add addEdgeImpl2

    // insert (v,u) if undirected graph (repeat above but reverse vertex order)
    if (!this.digraph)
    {
        v.insertAdjacent(u_i, weight); // fix encapsulation issue in add addEdgeImpl2
    }
}


// --------------------
//Output graph properties and list its vertices
//This function depends on the vertex lister method of the graph.
function printGraphImpl()
{
    document.write("<p>GRAPH {", this.label, "} ", this.digraph ? "" : "UN", "DIRECTED - ", this.nv, " VERTICES, ",
        this.ne, " EDGES:</p>");

    document.write("<p>", this.componentInfo(), "<p>"); // implement encapsulation

    // list vertices
    this.listVerts();
}


// --------------------
function listVertsImpl()
{
    var i, v;
    for (i = 0; i < this.nv; i++)
    {
        v = this.vert[i];
        document.write("VERTEX: ", i, v.vertexInfo(), "<br>"); //
    }
}



// --------------------
function better_input(v, e)
{
    // set vertex and edge count fields
    this.nv = v.length;
    this.ne = e.length;

    // input vertices into internal vertex array
    for (var i = 0; i < this.nv; i++)
    {
        this.vert[i] = new Vertex(v[i]);
    }

    // input vertex pairs from edge list input array
    // remember to pass vertex ids to add_edge()

    for (i = 0; i < this.ne; i++)
    {
        this.addEdge(e[i].u, e[i].v, e[i].w);
    }

    // double edge count if graph undirected

    if (!this.digraph)

    {

        this.ne = e.length * 2;

    }



    if (!(e[0].w == undefined))

    {

        this.weighted = true;

    }



}



// --------------------

function better_output()
{
    document.write("<p>GRAPH {", this.label, "} ", this.weighted ? "" : "UN", "WEIGHTED, ", this.digraph ? "" : "UN", "DIRECTED - ", this.nv, " VERTICES, ",
        this.ne, " EDGES:</p>");

    // report connectivity status if available
    var output;
    switch (this.connectedComp)
    {
        case 0:
            output = "no connectivity info ";
            break;
        case 1:
            output = "CONNECTED";
            break;
        default:
            output = "DISCONNECTED " + this.connectedComp;
            break;
    }
    document.write(output + '</p>');

    // list vertices
    this.listVerts()
}


// --------------------
function topoSearchImpl(fun)
{
    var i;

    // mark all vertices unvisited
    for (i = 0; i < this.nv; i++)
    {
        this.vert[i].visit = false;
    }

    // traverse unvisited connected component
    for (i = 0; i < this.nv; i++)
    {
        if (!this.vert[i].visit)
        {
            (!this.dfsORbfs) ? (fun++, this.dfs(i)) : this.bfs(i);
        }
    }
    return fun;
}



// --------------------

function dfsImpl(v_i)
{
    // get landing vert by id then process
    var v = this.vert[v_i];
    v.visit = true;

    // array of push order
    this.dfs_push[this.dfs_push.length] = v_i;

    // recursively traverse unvisited adjacent vertices
    var w = v.adjacentByID(); //implement encapsulation
    var i;

    for (i = 0; i < w.length; i++)
    {
        if (!this.vert[w[i]].visit)
            this.dfs(w[i]);
    }

    this.connectedComp = i;
}

// --------------------
function bfsImpl(v_i)
{
    // get vertex v by its id
    var v = this.vert[v_i];

    // process v
    v.visit = true;

    // initialize queue with v
    var queue = new Queue();
    queue.enqueue(v);
    this.bfs_order[this.bfs_order.length] = v_i;

    // while queue not empty
    while (!queue.isEmpty())
    {
        // dequeue and process a vertex, u
        var u = queue.dequeue();

        // queue all unvisited vertices adjacent to u
        var w = u.adjacentByID(); //implement encapsulation

        for (var i = 0; i < w.length; i++)
        {
            if (!this.vert[w[i]].visit)
            {
                this.vert[w[i]].visit = true;
                queue.enqueue(this.vert[w[i]]);
                this.bfs_order[this.bfs_order.length] = w[i];
            }
        }
    }
}


//--------------------
function makeAdjMatrixImpl()
{
    for (var i = 0; i < this.nv; i++)
    {
        var v = this.vert[i];
        this.adjMatrix[i] = [];

        // initially create row elements and zero the adjacency matrix
        for (var j = 0; j < this.nv; j++)
        {
            this.adjMatrix[i][j] = 0;
        }

        // for each vertex, set 1 for each adjacency
        var w = v.adjacentByID();
        for (j = 0; j < w.length; j++)
        {
            this.adjMatrix[i][w[j]] = 1;
        }
    }
}

// --------------------
function makeAdjMatrixImpl2()
{
    for (var i = 0; i < this.nv; i++)
    {
        var v = this.vert[i];
        this.adjMatrix[i] = [];
        // initially create row elements and zero the adjacency matrix
        for (var j = 0; j < this.nv; j++)
        {
            this.adjMatrix[i][j] = 0;
        }

        // for each vertex, set 1 for each adjacency if graph unweighted
        // set weight if graph weighted
        var w = v.adjacentByID();
        var weight = v.adjacent.traverse(); //encapsulation issue

        for (j = 0; j < w.length; j++)
        {
            this.adjMatrix[i][w[j]] = (this.weighted) ? weight[j] : 1;
        }
    }
}



// --------------------
function makeAdjMatrixImpl3()
{
    for (var i = 0; i < this.nv; i++)
    {
        this.adjMatrix[i] = [];

        // initially create row elements and zero the adjacency matrix
        for (var j = 0; j < this.nv; j++)
        {
            this.adjMatrix[i][j] = 0;
        }

        var v = this.vert[i];
        var w = v.adjacentByID();
        var incidentEdges = v.incidentEdges(); // fix encapsulation issue in makeAdjMatrixImpl2

        for (j = 0; j < incidentEdges.length; j++)
        {
            this.adjMatrix[i][w[j]] = (this.weighted) ? incidentEdges[j].edgeWeight : 1;
        }
    }
}


// --------------------
function isConnectedImpl()
{
    return this.connectedComp == 1 ? true : false;
}


// --------------------
function componentInfoImpl() //Report connectivity information if available
{
    var output;
    switch (this.connectedComp)
    {
        case 0:
            output = "no connectivity info ";
            break;
        case 1:
            output = "CONNECTED";
            break;
        default:
            output = "DISCONNECTED " + this.connectedComp;
            break;
    }
    return output;
}


// --------------------
function adjacentByIdImpl() //Get id of adjacent vertices
{
    var out = [];
    var edge = this.adjacent.traverse();
    for (var i = 0; i < edge.length; i++)
    {
        out[i] = edge[i].target_v;
    }
    return out;
}

// --------------------
//this function return Array of custom objects containing edge information
function incidentEdgesImpl()
{
    var enode = []; // Array of object
    var w = this.adjacent.traverse(); //encappp
    for (var i = 0; i < w.length; i++)
    {
        enode[i] = {
            "adjvert_i": w[i].target_v,
            "edgeWeight": w[i].weight
        };
    }
    return enode;
}

//----------------------
function vertexInfoImpl() //Get vertex details in a printable string
{
    return (" {" + this.label + "} - VISIT: " + this.visit + " - ADJACENCY: " + this.adjacentByID());
}


// --------------------
//Insert a new edge node in the adjacency list of vertex.
// used by addEdgeImpl3 to implement encapsulation
function insertAdjacentImpl(v_i, weight)
{
    var edge = new Edge(v_i, weight);
    this.adjacent.insert(edge);
}

// --------------------
function makeGraphImpl(n, m, w) // it is not part of Project 1.
{}


//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//
//Priority queue section //
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

/** 
delete the item with highest priority
@methodOf PQueue#
@author SARA ALQAIDI
@returns vertex with minimum weight 
 */

function pqDeleteMinImpl()
{

    var min = this.pq.first;

    var ptr = this.pq.first;

    var prev = null;



    if (!this.isEmpty())
    {

        while (ptr != null)
        {

            if (ptr.next != null && ptr.next.item.prior < min.item.prior)
            {

                min = ptr.next;

                prev = ptr;

            }

            ptr = ptr.next;

        }

        if (min != this.pq.first)
        { //if first element is not min

            prev.next = min.next;

        }
        else
        {

            this.pq.delete_first(); //if first element is the min, delete it using delete_first() from Linklist

        }

        return min.item;

    }

}



//-----------------------------------------------------------------------
/**
insert in priority queue or update the node's key 
@methodOf PQueue#
@author SARA ALQAIDI
@param {PQNode} item pq node
@param {integer} key represents the priority key
 */

function pqInsertImpl(item, key)
{

    //create new pq node 

    var item = new PQNode(item, key);

    var ptr = this.pq.first;

    var insert = true; //to check if its insert or update 

    if (!this.isEmpty())
    {

        while (ptr.next != null)
        { //Traverse in pq   

            if (item.item === ptr.item.item)
            { // we found same node in pq

                if (item.prior < ptr.item.prior)
                {

                    ptr.item.prior = item.prior; //update key 

                    insert = false; // don't need to insert just update the key 

                    break; //exit while   

                }

            }

            ptr = ptr.next;

        }

    }

    //----------------------------------------------

    if (insert)
    { //if insert true

        this.pq.insert(item); //using insert() from linklist 

    }

}


//-----------------------------------------------------------------------
/**
check if the priority queue is empty or not
@methodOf PQueue#
@author SARA ALQAIDI
@returns {boolean} -  true if  priority queue is not empty otherwise false 
 */

function pqIsEmptyImpl()

{ // return true if the queue is empty. 

    return this.pq.isEmpty();

}