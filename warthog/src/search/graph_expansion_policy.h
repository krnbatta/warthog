#ifndef WARTHOG_GRAPH_EXPANSION_POLICY_H
#define WARTHOG_GRAPH_EXPANSION_POLICY_H

#include "constants.h"

// search/graph_expansion_policy.h
//
// an expansion policy for xy graphs. includes support for a node
// filtering mechanism (i.e. it can be configured to prune successor
// nodes that do not match some specified criteria)
//
// @author: dharabor
// @created: 2018-05-04
//


#include "dummy_filter.h"
#include "arraylist.h"
#include "xy_graph.h"
#include "problem_instance.h"
#include "search_node.h"

namespace warthog
{

template <class FILTER = warthog::dummy_filter>
class graph_expansion_policy
{
    public:
        graph_expansion_policy(
                warthog::graph::xy_graph* g, FILTER* filter = 0)
            :  filter_(filter), g_(g)
        {
            assert(g);
            if(filter == 0)
            {
                fn_generate_successor = &warthog::graph_expansion_policy
                                        <FILTER>::fn_generate_no_filter;
            }
            else
            {
                fn_generate_successor = &warthog::graph_expansion_policy
                                        <FILTER>::fn_generate_with_filter;
            }

            nodes_pool_size_ = g_->get_num_nodes();
            nodepool_ = new warthog::search_node[nodes_pool_size_];
            for(uint32_t i = 0; i < nodes_pool_size_; i++)
            {
                nodepool_[i].set_id(i);
            }
        }

        ~graph_expansion_policy()
        {
            delete [] nodepool_;
        }

		void
		expand(warthog::search_node* current, warthog::problem_instance* pi)
        {
            edge_index_ = 0;
            current_graph_node_ = g_->get_node(current->get_id()) ;
        }

		inline void
		first(warthog::search_node*& ret, double& cost)
		{
            edge_index_ = UINT32_MAX;
            next(ret, cost);
		}

		inline void
		n(warthog::search_node*& ret, double& cost)
		{
            if(edge_index_ < current_graph_node_->out_degree())
            {
                warthog::graph::edge *e =
                    current_graph_node_->outgoing_begin() + edge_index_;
                ret = (this->*fn_generate_successor)
                         (e->node_id_, edge_index_, e);
                cost = e->wt_;
            }
            else
            {
                ret = 0;
                cost = 0;
            }
		}

		inline void
		next(warthog::search_node*& ret, double& cost)
		{
            assert(current_graph_node_);
            ret = 0;
            cost = warthog::INF;

            warthog::graph::edge_iter begin =
                current_graph_node_->outgoing_begin();

            for( ++edge_index_;
                   edge_index_ < current_graph_node_->out_degree();
                   edge_index_++)
            {
                warthog::graph::edge& e = *(begin+edge_index_);
                assert(e.node_id_ < g_->get_num_nodes());
                ret = (this->*fn_generate_successor)
                        (e.node_id_, edge_index_, e);
                if(ret)
                {
                    cost = e.wt_;
                    break;
                }
            }
		}

        warthog::search_node*
        generate_start_node(warthog::problem_instance* pi)
        {
            uint32_t s_graph_id = g_->to_graph_id(pi->start_id_);
            if(s_graph_id == warthog::INF) { return 0; }
            return &nodepool_[s_graph_id];
        }

        warthog::search_node*
        generate_target_node(warthog::problem_instance* pi)
        {
            // convert from external id to internal id
            uint32_t t_graph_id = g_->to_graph_id(pi->target_id_);
            if(t_graph_id == warthog::INF) { return 0; }

            // also update the filter with the new target location
            filter_->set_target(pi->target_id_);

            // generate the search node
            return &nodepool_[t_graph_id];
        }

        warthog::search_node*
        generate(uint32_t nid)
        {
            return &nodepool_[nid];
        }

        bool
        is_target(warthog::search_node* n, warthog::problem_instance* pi)
        {
            return n->get_id() == pi->target_id_;
        }

        inline uint32_t
        get_num_neighbours()
        {
            return
                current_graph_node_ ?  current_graph_node_->out_degree() : 0;
        }

        void
        get_xy(uint32_t node_id, int32_t& x, int32_t& y)
        {
            g_->get_xy(node_id, x, y);
        }

        uint32_t
        get_nodes_pool_size() { return nodes_pool_size_; }

        size_t
		mem()
        {
            return
                sizeof(warthog::search_node)*nodes_pool_size_ +
                sizeof(this);
        }

	private:
        FILTER* filter_;
        warthog::graph::xy_graph* g_;

        uint32_t edge_index_;
        warthog::graph::node* current_graph_node_;

        warthog::search_node* nodepool_;
        uint32_t nodes_pool_size_;

        typedef
            warthog::search_node*
            (warthog::graph_expansion_policy<FILTER>::*generate_fn)
            (uint32_t current_id, uint32_t edge_idx, warthog::graph::edge& e);

        generate_fn fn_generate_successor;

        inline warthog::search_node*
        fn_generate_with_filter(
                uint32_t current_id,
                uint32_t edge_idx,
                warthog::graph::edge& e)
        {
            if(!filter_->filter(current_id, edge_idx))
            {
                return &nodepool_[e.node_id_];
            }
            return 0;
        }

        inline warthog::search_node*
        fn_generate_no_filter(
                uint32_t current_id,
                uint32_t edge_idx,
                warthog::graph::edge& e)
        {
            return &nodepool_[e.node_id_];
        }

};

typedef
warthog::graph_expansion_policy<> simple_graph_expansion_policy;

}

#endif
