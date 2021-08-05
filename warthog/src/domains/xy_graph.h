#ifndef WARTHOG_XY_GRAPH_H
#define WARTHOG_XY_GRAPH_H

// xy_graph.h
// 
// A simple general purpose data structure for directed weighted xy graphs 
// (i.e. spatial networks embedded in two dimensions). 
// Supported input types are:
//  - warthog::gridmap objects
//  - road network graphs in the format of the 9th DIMACS competition
//
// This implemention stores all nodes and edges in two separate flat arrays
// and uses one to index the other. The graph can contain a maximum of
// 2^32 nodes and edges.
//
// @author: dharabor
// @created: 2016-01-07
//


#include "dimacs_parser.h"
#include "euclidean_heuristic.h"
#include "forward.h"
#include "graph.h"
#include "gridmap_expansion_policy.h"
#include "constants.h"

#include <ostream>
#include <unordered_map>
#include <vector>

namespace warthog
{

namespace graph
{

template<class T_NODE, class T_EDGE>
class xy_graph_base
{
    public:
        // create an empty graph
        xy_graph_base(uint32_t num_nodes=0) : verbose_(false)
        {
            grow(num_nodes);
        }

        ~xy_graph_base() 
        { }

        // create a graph from a grid map. 
        bool
        load_from_grid(warthog::gridmap* gm, bool store_incoming=true)
        {
            // enumerate the traversable nodes with degree > 0
            struct edge_tuple 
            {
                uint32_t from;
                uint32_t to;
                uint32_t wt;
            };

            warthog::gridmap_expansion_policy exp(gm);
            std::vector<uint32_t> nodes;
            std::vector<edge_tuple> edges;
            for(uint32_t y = 0; y < gm->header_height(); y++)
            {
                for(uint32_t x = 0; x < gm->header_width(); x++)
                {
                    uint32_t node_id = y * gm->header_width() + x;
                    uint32_t gm_id = gm->to_padded_id(node_id);

                    // skip obstacles
                    if(!gm->get_label(gm_id)) { continue; }

                    // each traversable node appears in the graph
                    add_node(x * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR, 
                             y * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR,
                             node_id);

                    warthog::search_node* n = exp.generate(gm_id);
                    warthog::search_node* nei = 0;
                    double edge_cost = 0;
                    exp.expand(n, 0);
                    for(exp.first(nei, edge_cost); nei != 0; exp.next(nei, edge_cost))
                    {
                        //// each traversable node with degree > 0 
                        //// appears in the graph
                        //if(id_map_.size() == 0 || id_map_.back() != node_id) 
                        //{ 
                        //    add_node(x * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR, 
                        //             y * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR,
                        //             node_id);
                        //}

                        // track the list of traversable edges
                        uint32_t nei_x, nei_y;
                        gm->to_unpadded_xy(nei->get_id(), nei_x, nei_y);
                        uint32_t nei_id = nei_y * gm->header_width() + nei_x;

                        assert(edge_cost > 0);

                        edge_tuple e;
                        e.from = node_id, 
                        e.to = nei_id, 
                        e.wt = edge_cost * warthog::graph::GRID_TO_GRAPH_SCALE_FACTOR;
                        edges.push_back(e);
                    }
                }
            }

            for(uint32_t i = 0; i < edges.size(); i++)
            {
                edge_tuple tuple = edges.at(i);    
                uint32_t from_id = to_graph_id(tuple.from);
                uint32_t to_id = to_graph_id(tuple.to);
                assert(from_id != warthog::INF && to_id != INF);

                warthog::graph::node* from = get_node(from_id);
                warthog::graph::node* to = get_node(to_id);
                assert(from && to);

                from->add_outgoing(warthog::graph::edge(to_id, tuple.wt));
                if(store_incoming)
                {
                    to->add_incoming(warthog::graph::edge(from_id, tuple.wt));
                }
            }
            return true;
        }

        // read in a map in the format of the 9th DIMACS
        // competition. In this format graphs are specified
        // using two files: (i) a gr file which defines edge
        // weights and endpoints and; (ii) a co file which defines 
        // node ids and xy coordinates
        //
        // @param reverse_arcs: reverses the direction of each edge
        // @param duplicate_edges: store edges with both head and tail node
        // @param enforce_euclidean: arc lengths must be >= euclidean distance
        bool
        load_from_dimacs(const char* gr_file, const char* co_file,
                bool reverse_arcs =false,
                bool store_incoming_edges = false,
                bool enforce_euclidean=true)
        {
            clear();

            //warthog::dimacs_parser dimacs(gr_file, co_file);
            warthog::dimacs_parser dimacs;
            if(!( dimacs.load_graph(gr_file) && dimacs.load_graph(co_file) ))
            {
                return false;
            }
            //std::cout << "loaded" << std::endl;
            
            filename_.assign(gr_file);

            // sort nodes by their ids
            bool (*node_comparator) 
                 (warthog::dimacs_parser::node, warthog::dimacs_parser::node) =
                    [](warthog::dimacs_parser::node n1,
                       warthog::dimacs_parser::node n2) -> bool
                        {
                            return n1.id_ < n2.id_;
                        };
            std::sort(dimacs.nodes_begin(), dimacs.nodes_end(), node_comparator);
            if(verbose_) { std::cerr << "nodes, sorted" << std::endl; }
            
            // allocate memory for nodes
            uint32_t num_nodes_dimacs = dimacs.get_num_nodes();
            capacity(num_nodes_dimacs);
            for(warthog::dimacs_parser::node_iterator it = dimacs.nodes_begin();
                    it != dimacs.nodes_end(); it++)
            {
               add_node((*it).x_, (*it).y_, (*it).id_);
            }
            if(verbose_) { std::cerr << "nodes, converted" << std::endl; }

            // scan the list of edges so as to know the in and out degree of each node
            std::vector<uint32_t> in_deg(dimacs.get_num_nodes(), 0);
            std::vector<uint32_t> out_deg(dimacs.get_num_nodes(), 0);
            for(warthog::dimacs_parser::edge_iterator it = dimacs.edges_begin();
                    it != dimacs.edges_end(); it++)
            {
                uint32_t h_graph_id = to_graph_id(it->head_id_);
                uint32_t t_graph_id = to_graph_id(it->tail_id_);
                in_deg[h_graph_id]++;
                out_deg[t_graph_id]++;
            }

            // allocate memory for edges
            for(warthog::dimacs_parser::node_iterator it = dimacs.nodes_begin();
                    it != dimacs.nodes_end(); it++)
            {
                uint32_t nid = it - dimacs.nodes_begin();
                nodes_[nid].capacity(
                        store_incoming_edges ? in_deg[nid] : 0, 
                        out_deg[nid]);
            }

            // convert edges to graph format
            for(warthog::dimacs_parser::edge_iterator it = dimacs.edges_begin();
                    it != dimacs.edges_end(); it++)
            {
                uint32_t hid = to_graph_id( (*it).head_id_ );
                uint32_t tid = to_graph_id( (*it).tail_id_ );

                // in a reverse graph the head and tail of every 
                // edge are swapped
                if(reverse_arcs)
                {
                    uint32_t tmp = hid;
                    hid = tid;
                    tid = tmp;
                }
                warthog::graph::edge e;
                e.node_id_ = hid;
                e.wt_ = (*it).weight_;

#ifndef NDEBUG
                uint32_t deg_before = nodes_[tid].out_degree();
#endif

                nodes_[tid].add_outgoing(e);

#ifndef NDEBUG
                uint32_t deg_after = nodes_[tid].out_degree();
#endif

                // edges can be stored twice: once as an incoming edge 
                // and once as an outgoing edge
                if(store_incoming_edges)
                {
                    e.node_id_ = tid;
                    nodes_[hid].add_incoming(e);

#ifndef NDEBUG
                    // sanity check: either the edge was a duplicate (and nothing
                    // was added) or the added was added in which case it should
                    // be the last element in the outgoing list of the tail node
                    warthog::graph::edge sanity = 
                        *(nodes_[tid].outgoing_end()-1);
                    assert(deg_before == deg_after || 
                            sanity.node_id_ == hid);
#endif
                }
                if(verbose_ && ((it - dimacs.edges_begin()) % 1000) == 0)
                {
                    std::cerr 
                        << "\rconverted K edges " 
                        << (it - dimacs.edges_begin()) / 1000;
                }
            }
            is_euclidean(enforce_euclidean);
            if(verbose_) { std::cout << "edges, converted" << std::endl; }

            std::cerr << "edge memory fragmentation: (1=none): " 
                << this->edge_mem_frag() << std::endl;
            return true;
        }

        // load graph from a binary blob description
        bool
        load_from_blob(  std::istream& oss,
                           bool store_incoming_edges = false,
                           bool enforce_euclidean=true)
        {
            // read number of nodes 
            uint32_t num_nodes;
            oss.read((char*)&num_nodes, sizeof(num_nodes));
            if(num_nodes > 0)
            {
                nodes_.resize(num_nodes);
                if(!oss.good()) { std::cerr << "err; reading #nodes data\n"; return false; }
            }

            // ready xy coordinates
            uint32_t xy_sz;
            oss.read((char*)&xy_sz, sizeof(xy_sz));
            if(!oss.good())
            { std::cerr << "err; reading #xy data\n"; return false;}
            if(xy_sz > 0)
            {
                xy_.resize(xy_sz);
                oss.read((char*)&xy_[0], sizeof(int32_t)*xy_sz);
                if(!oss.good()) { std::cerr << "err; reading xy data\n"; return false; }
            }

            // read external ids
            uint32_t id_map_sz;
            oss.read((char*)&id_map_sz, sizeof(id_map_sz));
            if(!oss.good()) { std::cerr << "err; reading #external-ids\n"; return false; }
            if(id_map_sz == get_num_nodes())
            {
                id_map_.resize(id_map_sz);
                oss.read((char*)&id_map_[0], sizeof(int32_t)*id_map_sz);
                if(!oss.good()) 
                { std::cerr << "err; reading external id data\n"; return false;}

                // build hashtable for reverse lookups
                ext_id_map_.reserve(get_num_nodes()*0.8);
                for(uint32_t i = 0; i < id_map_.size(); i++)
                {
                    ext_id_map_.insert(std::pair<uint32_t, uint32_t>(id_map_[i], i));
                }
            }
            else
            {
                std::cerr 
                    << "warn; ignoring external id values (read " << id_map_sz 
                    << " ids but graph has " << get_num_nodes() << " nodes)" 
                    << std::endl;
            }

            // read node and edge data
            uint32_t nid = 0;
            for( ; nid < num_nodes; nid++)
            {
                nodes_[nid].load(oss);
                if(!oss.good()) 
                { std::cerr 
                    << "err; reading outgoing adj list for node " << nid
                    << std::endl;
                    break;
                }
            }
            if(nid != num_nodes)
            {
                std::cerr 
                    << "err; read " << nid << " of " << get_num_nodes()
                    << " adjacency lists. missing data? "
                    << std::endl;
            }

            if(store_incoming_edges)
            {
                std::cerr << "#outgoing " << this->get_num_edges_out() << std::endl;

                // collect all edges and compute in degree of every node
                std::vector<uint32_t> in_deg_all(get_num_nodes(), 0);
                std::vector<uint32_t> from_id;
                std::vector<uint32_t> to_id;
                std::vector<double> weight;

                for( nid = 0; nid < num_nodes; nid++)
                {
                    for( warthog::graph::edge_iter it = nodes_[nid].outgoing_begin();
                            it != nodes_[nid].outgoing_end(); 
                            it++ ) 
                    { 
                        in_deg_all[it->node_id_]++;
                        from_id.push_back(nid);
                        to_id.push_back(it->node_id_);
                        weight.push_back(it->wt_);
                    }
                }

                // allocate memory
                for( nid = 0; nid < get_num_nodes(); nid++)
                { nodes_[nid].capacity(in_deg_all[nid], nodes_[nid].out_degree()); }

                // add incoming edges 
                for(uint32_t i = 0; i < from_id.size(); i++)
                {
                    nodes_[to_id[i]].add_incoming(
                            warthog::graph::edge(from_id[i], weight[i]));
                }
                std::cerr << "#incoming " << this->get_num_edges_in() << std::endl;
            }

            return is_euclidean(enforce_euclidean);
        }

        warthog::graph::xy_graph_base<T_NODE, T_EDGE>&
        operator=(const warthog::graph::xy_graph_base<T_NODE, T_EDGE>&& other)
        {
            // my memory, not want
            clear();

            // your memory, can has?
            verbose_ = other.verbose_;
            filename_ = other.filename_;
            nodes_ = std::move(other.nodes_);
            xy_ = std::move(other.xy_);
            id_map_ = std::move(other.id_map_);
            ext_id_map_ = std::move(other.ext_id_map_);

            return *this;
        }
        
        bool
        operator==(const warthog::graph::xy_graph_base<T_NODE, T_EDGE>& other)
        {
            if(xy_.size() == other.xy_.size())
            {
                for(uint32_t i = 0; i < xy_.size(); i++)
                {
                    if(xy_[i] != other.xy_[i]) { return false; }
                }
            } 
            else { return false; }


            if(id_map_.size() == other.id_map_.size())
            {
                for(uint32_t i = 0; i < id_map_.size(); i++)
                {
                    if(id_map_[i] != other.id_map_[i]) { return false; }
                }
            }
            else { return false; }
            
            if(get_num_nodes() == other.get_num_nodes())
            {
                for(uint32_t i = 0; i < get_num_nodes(); i++)
                {
                    if(!(nodes_[i] == other.nodes_[i])) { return false; }
                }
                return true;
            }
            return false;
        }



        // set the number of nodes and edges in the graph to zero.
        // any currently assigned nodes and edges have their destructors 
        // called
        void
        clear()
        {
            nodes_.clear();
            xy_.clear();
            id_map_.clear();
            ext_id_map_.clear();
        }

        // grow the graph so that the number of vertices is equal to 
        // @param num_nodes. if @param num_nodes is less thna the current
        // number of nodes, this function does nothing.
        void
        grow(uint32_t num_nodes)
        {
            nodes_.resize(num_nodes);
            xy_.resize(num_nodes*2);
            id_map_.resize(num_nodes);
            ext_id_map_.reserve(num_nodes*0.8);
        }

        // allocate capacity for at least @param num_nodes
        // when @param num_nodes <= the number of nodes in the graph, this
        // function does nothing
        void
        capacity(uint32_t num_nodes)
        {
            nodes_.reserve(num_nodes);
            xy_.reserve(num_nodes*2);
            id_map_.reserve(num_nodes);
            ext_id_map_.reserve(num_nodes*0.8);
        }


        inline uint32_t
        get_num_nodes() const
        {
            return nodes_.size();
        }

        inline uint32_t 
        get_num_edges_out() const
        {
            uint32_t num_edges = 0;
            for(auto& node : nodes_)
            {
                num_edges += node.out_degree();
            }
            return num_edges;
        }

        inline uint32_t
        get_num_edges_in() const
        {
            uint32_t num_edges = 0;
            for(auto& node : nodes_)
            {
                num_edges += node.in_degree();
            }
            return num_edges;
        }

        // Fetch the xy coordinates of a node
        //
        // @param id: an internal graph id
        // @return x: the x coordinate of node @param id
        // @return y: the y coordinate of node @param id
        inline void
        get_xy(uint32_t id, int32_t& x, int32_t& y) const
        {
            if(id < get_num_nodes())
            {
                x = xy_[id*2];
                y = xy_[id*2+1];
            }
            else
            {
                x = y = warthog::INF;
            }
        }

        // Set the xy coordinates of a node
        //
        // @param id: an internal graph id
        // @param x: the x coordinate of node @param id
        // @param y: the y coordinate of node @param id
        inline void
        set_xy(uint32_t id, int32_t x, int32_t y)
        {
            uint32_t nodes_sz = get_num_nodes();
            if(id >= nodes_sz) { return; }
            if(xy_.size() != nodes_sz) { xy_.resize(nodes_sz); }
            xy_[id*2] = x;
            xy_[id*2+1] = y;
        }

        // Fetch a node
        //
        // @param id: an internal graph id
        // @return: the node object associated with @param id
        inline T_NODE*
        get_node(uint32_t id) 
        {
            //if(id >= ID_OFFSET && id < nodes_sz_)
            if(id < get_num_nodes())
            {
                return &nodes_[id];
            }
            return 0;
        }

        // Add a new node into the graph. If a node already exists in the 
        // graph with the same external id as @param ext_id then then nothing 
        // is added.
        // NB: a new node is always added if @param ext_id is equal to the 
        // value warthog::INF
        //
        // @param x: the x-coordinate of the new node
        // @param y: the y-coordinate of the new node
        // @param ext_id: an (optional) external id for this node
        // @return: the internal graph id of the new node or the id of the
        // existing node whose graph id is equal to @param ext_id
        uint32_t
        add_node(int32_t x, int32_t y, uint32_t ext_id)
        {
            // check if a node with the same external id already exists; if so 
            // return the id of the existing node. otherwise, add a new node
            uint32_t graph_id = to_graph_id(ext_id);
            if(graph_id != warthog::INF) { return graph_id; } 

            graph_id = get_num_nodes();

            nodes_.push_back(warthog::graph::node());
            xy_.push_back(x);
            xy_.push_back(y);
            id_map_.push_back(ext_id);
            if(ext_id != warthog::INF)
            {
                ext_id_map_.insert(std::pair<uint32_t, uint32_t>(ext_id, graph_id));
            }
            return graph_id;
        }

        uint32_t
        add_node(int32_t x, int32_t y)
        {
            return add_node(x, y, warthog::INF);
        }

        uint32_t
        add_node()
        {
            return add_node(warthog::INF, warthog::INF, warthog::INF);
        }

        // print extra stuff to std::err 
        inline void 
        set_verbose(bool verbose) { verbose_ = verbose; } 

        inline bool
        get_verbose() const { return verbose_; }

        // @return the name of the file from which the 
        // current graph object was constructed
        inline const char* 
        get_filename() const { return filename_.c_str(); }

        inline size_t
        mem()
        {
            size_t mem = 0;
            for(uint32_t i = 0; i < get_num_nodes(); i++)
            {
                mem += nodes_[i].mem();
            }
            mem += sizeof(int32_t) * xy_.size() * 2;
            mem += sizeof(char)*filename_.length() +
                sizeof(*this);
            return mem;
        }
        
        // convert an external node id (e.g. as it appears in an input file)
        // to the equivalent internal id used by the current graph
        //
        // @param ex_id: the external id of the node
        // @return: the corresponding internal id for @param ex_id
        // if ex_id did not appear in the input file (or if the graph
        // was not created from an input file) the function returns
        // the value warthog::INF
        inline uint32_t 
        to_graph_id(uint32_t ext_id) 
        { 
            if(id_map_.size() == 0) { return warthog::INF; }

            std::unordered_map<uint32_t, uint32_t>::iterator it 
                    = ext_id_map_.find(ext_id);

            if(it == ext_id_map_.end()) { return warthog::INF; }
            return (*it).second;
        }

        // convert an internal node id (i.e. as used by the current graph
        // to the equivalent external id (e.g. as appears in an input file)
        //
        // @param ex_id: the external id of the node
        // @return: the corresponding internal id for @param ex_id
        // if ex_id did not appear in the input file (or if the graph
        // was not created from an input file) the function returns
        // the value warthog::INF
        inline uint32_t 
        to_external_id(uint32_t in_id)  const
        { 
            if(in_id > get_num_nodes()) { return warthog::INF; }
            return id_map_.at(in_id);
        }

        // compute the proportion of bytes allocated to edges with respect
        // to the size of the address space those bytes span.
        // a value of 1 using this metric indicates that the edges perfectly 
        // fit into the allocated address space
        inline double
        edge_mem_frag() 
        {
            T_EDGE *min_addr, *max_addr;
            min_addr = this->get_node(0)->outgoing_begin();
            max_addr = this->get_node(0)->outgoing_end();
            for(uint32_t i = 0; i < this->get_num_nodes(); i++)
            {
                T_NODE* n = this->get_node(i);
                T_EDGE* out_begin = n->outgoing_begin();
                T_EDGE* out_end = n->outgoing_end();
                T_EDGE* in_begin = n->incoming_begin();
                T_EDGE* in_end = n->incoming_end();

                min_addr = out_begin ? 
                            (out_begin < min_addr ? out_begin : min_addr) :
                            min_addr;
                max_addr = out_end ? 
                            (out_end > max_addr ? out_end : max_addr) :
                            max_addr;
                min_addr = in_begin ? 
                            (in_begin < min_addr ? in_begin : min_addr) :
                            min_addr;
                max_addr = in_end ? 
                            (in_end > max_addr ? in_end : max_addr) :
                            max_addr;
            }
            
            uint64_t mem_lb = sizeof(warthog::graph::edge) *
                (this->get_num_edges_out() + this->get_num_edges_in());
            uint64_t mem_actual =
                (max_addr-min_addr) * sizeof(warthog::graph::edge);
            return mem_actual / (double)mem_lb;
        }

        // write a binary blob description of the graph
        void
        save(std::ostream& oss)
        {
            // write number of nodes
            uint32_t nodes_sz = get_num_nodes();
            oss.write((char*)(&nodes_sz), sizeof(nodes_sz));
            if(!oss.good()) 
            { std::cerr << "err writing #nodes\n"; return; }

            // write xy values
            uint32_t xy_sz = xy_.size();
            oss.write((char*)&xy_sz, sizeof(xy_sz));
            if(!oss.good()) 
            { std::cerr << "err writing #xy\n"; return; }
            oss.write((char*)&xy_[0], sizeof(int32_t)*xy_sz);
            if(!oss.good()) 
            { std::cerr << "err writing xy coordinates data\n"; return; }

            // write external ids
            uint32_t id_map_sz = id_map_.size();
            oss.write((char*)&id_map_sz, sizeof(int32_t));
            if(!oss.good()) 
            { std::cerr << "err writing #external ids\n"; return; }
            oss.write((char*)&id_map_[0], sizeof(int32_t)*id_map_sz);
            if(!oss.good()) 
            { std::cerr << "err writing external ids\n"; return; }
            
            // write edge data
            uint32_t nid = 0;
            for( ; nid < get_num_nodes(); nid++)
            {
                nodes_[nid].save(oss);
                if(!oss.good()) 
                { std::cerr 
                    << "err; writing outgoing adj list for node " << nid
                    << std::endl;
                    break;
                }
            }
        }

        // print text descriptions of the set of nodes 
        // in the range [firstid, lastid)
        // NB: if the range is invalid, nothing is printed
        void
        print_dimacs_co(
                std::ostream& oss, uint32_t first_id, uint32_t last_id)
        {
            if(first_id > last_id || last_id > get_num_nodes())
            { return; }

            if(get_num_nodes() > 0)
            {
                oss << "p aux sp co " << (last_id - first_id) << std::endl;
                for(uint32_t i = first_id; i < last_id; i++)
                {
                    oss << "v " << to_external_id(i) 
                        << " " << xy_[i*2] << " " << xy_[i*2+1] << std::endl;
                }
            }
        }

        // print text descriptions of the set of arcs associated with 
        // all nodes in the range [firstid, lastid)
        // NB: if the range is invalid, nothing is printed
        void
        print_dimacs_gr(
                std::ostream& oss,
                uint32_t first_id, uint32_t last_id)
        {
            if(first_id > last_id || last_id > get_num_nodes())
            { return; }

            if(get_num_nodes() > 0)
            {
                oss << "p sp " << (last_id - first_id) << " " << 
                    this->get_num_edges_out() << std::endl;
                for(uint32_t i = first_id; i < last_id; i++)
                {
                    warthog::graph::node* n = get_node(i);
                    for(warthog::graph::edge_iter it = n->outgoing_begin(); 
                            it != n->outgoing_end(); it++)
                    {
                        oss << "a " << to_external_id(i) << " " 
                            << to_external_id((*it).node_id_)
                            << " " << (uint32_t)((*it).wt_) << std::endl;
                    }
                }
            }
        }

        // check if arc weights are Euclidean and (optionally) fix if not.
        // "fixing" means arc weights must be at least as large as the
        // Euclidean distance between the arc's head and tail vertex.
        bool
        is_euclidean(bool fix_if_not=true)
        {
            warthog::euclidean_heuristic h_euc(this);
            for(uint32_t t_id= 0; t_id < get_num_nodes(); t_id++)
            {
                int32_t tx, ty, hx, hy;
                get_xy(t_id, tx, ty);
                if(tx == warthog::INF || ty == warthog::INF) { continue; }

                for(warthog::graph::edge_iter it = nodes_[t_id].outgoing_begin();
                    it != nodes_[t_id].outgoing_end();
                    it++)
                {
                    uint32_t h_id = (*it).node_id_;
                    get_xy(h_id, hx, hy);

                    double hdist = h_euc.h(tx, ty, hx, hy);
                    if((*it).wt_ < hdist)
                    {
                        if(!fix_if_not) { return false; }
                        (*it).wt_ = ceil(hdist);
                    } 
                }
            }
            return true;
        }



        //inline void
        //shrink_to_fit()
        //{
        //    edge* tmp = new edge[get_num_edges()];
        //    uint32_t e_index = 0;
        //    for(uint32_t i = 0; i < this->get_num_nodes(); i++)
        //    { 
        //        uint32_t in_deg = nodes_[i].in_degree();
        //        uint32_t out_deg = nodes_[i].out_degree();
        //        nodes_[i].relocate(&tmp[e_index], &tmp[e_index + in_deg]);
        //        e_index += (in_deg + out_deg);
        //    }
        //}

    private:
        // the set of nodes that comprise the graph
        std::vector<T_NODE> nodes_;
        
        // xy coordinates stored as adjacent pairs (x, then y)
        std::vector<int32_t> xy_;

        // these containers serve to map from external graph ids to 
        // internal graph ids
        std::vector<uint32_t> id_map_; 
        std::unordered_map<uint32_t, uint32_t> ext_id_map_; 

        std::string filename_;
        bool verbose_;
};
typedef xy_graph_base<warthog::graph::node, warthog::graph::edge> xy_graph;







}
}

#endif

