#include "apriori_filter.h"
#include "constants.h"
#include "zero_heuristic.h"
#include "fixed_graph_contraction.h"
#include "flexible_astar.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"

warthog::ch::fixed_graph_contraction::fixed_graph_contraction(
                warthog::graph::xy_graph* g)
    : g_(g)
{
    order_ = new std::vector<uint32_t>(get_graph()->get_num_nodes());
    warthog::ch::make_input_order(*g, *order_);
    init();
}

warthog::ch::fixed_graph_contraction::fixed_graph_contraction(
                warthog::graph::xy_graph* g,
                std::vector<uint32_t>* order)
    : g_(g)
{
    assert(get_graph()->get_num_nodes() == order->size());
    order_ = order;
    init();
}

warthog::ch::fixed_graph_contraction::~fixed_graph_contraction()
{
    delete alg_;
    delete filter_;
    delete heuristic_;
    delete expander_;
    delete open_;
}


void
warthog::ch::fixed_graph_contraction::init()
{
    done_ = false;
    verbose_ = false;
    c_pct_ = 100;
    order_index_ = 0;

    filter_ = new warthog::apriori_filter(get_graph()->get_num_nodes());
    expander_ = new warthog::graph_expansion_policy< warthog::apriori_filter >
        (get_graph(), filter_);
    open_ = new warthog::pqueue_min();

    heuristic_ = new warthog::zero_heuristic();
    alg_ = new flexible_astar<
                    warthog::zero_heuristic,
                    warthog::graph_expansion_policy<warthog::apriori_filter>,
                    warthog::pqueue_min>
                        (heuristic_, expander_, open_);
}

void
warthog::ch::fixed_graph_contraction::contract()
{
    if(done_) { return; }
    done_ = true;

    if(c_pct_ < 100)
    {
        std::cerr << "partially "
                  << "("<<c_pct_<<"% of nodes) ";
    }

    warthog::timer mytimer;
    double t_begin = mytimer.get_time_micro();

    std::cerr << "contracting graph " << g_->get_filename() << std::endl;
    total_searches_ = 0;
    total_expansions_ = 0;

    uint32_t edges_before = g_->get_num_edges_out();
    uint32_t total_nodes = g_->get_num_nodes();
    uint32_t num_contractions = 0;
    double t_last = mytimer.get_time_micro();
    for(uint32_t cid = next(); cid != warthog::INF; cid = next())
    {
        
        uint32_t pct = (num_contractions / (double)g_->get_num_nodes()) * 100;
        if(pct >= c_pct_)
        { 
            std::cerr << "\npartial contraction finished " 
                      << "(processed "<< pct << "% of all nodes)";
            break; 
        }

        warthog::graph::node* n = g_->get_node(cid);


        uc_neis_.clear();
        for(uint32_t i = 0; i < n->out_degree(); i++)
        {
            warthog::graph::edge& e_out = *(n->outgoing_begin() + i);
            if(filter_->get_flag(e_out.node_id_)) { continue; }
            uc_neis_.push_back(e_out);
        }

        uint32_t uc_neis_incoming_begin_ = uc_neis_.size();
        for(uint32_t i = 0; i < n->in_degree(); i++)
        {
            warthog::graph::edge& e_in = *(n->incoming_begin() + i);
            if(filter_->get_flag(e_in.node_id_)) { continue; }
            uc_neis_.push_back(e_in);
        }
        
        uint32_t max_expand = warthog::INF;
        double  max_outgoing_wt = 0;
        for(uint32_t j = 0; j < uc_neis_incoming_begin_; j++)
        {
            warthog::graph::edge& e_out = uc_neis_.at(j);
            if(e_out.wt_ > max_outgoing_wt) { max_outgoing_wt = e_out.wt_; }
        }

        // contract
        filter_->set_flag_true(cid);
        uint32_t eadd = 0;
        for(uint32_t i = uc_neis_incoming_begin_; i < uc_neis_.size(); i++)
        {
            warthog::graph::edge& e_in = uc_neis_.at(i);
            double max_cost = e_in.wt_ + max_outgoing_wt;
            witness_search(e_in.node_id_, warthog::INF, max_cost, max_expand);

            for(uint32_t j = 0; j < uc_neis_incoming_begin_; j++)
            {
                warthog::graph::edge& e_out = uc_neis_.at(j);
                if(e_in.node_id_ == e_out.node_id_) { continue; }
                
                warthog::search_node* nei = 
                    alg_->get_generated_node(e_out.node_id_);
                double witness_len = nei ? nei->get_g() : warthog::INF;
                double via_len = e_in.wt_ + e_out.wt_;

                if(witness_len > via_len)
                {
                    eadd++;
                    warthog::graph::node* tail = g_->get_node(e_in.node_id_);
                    tail->add_outgoing(
                            warthog::graph::edge(e_out.node_id_, via_len));
                    warthog::graph::node* head = g_->get_node(e_out.node_id_);
                    head->add_incoming(
                            warthog::graph::edge(e_in.node_id_, via_len));
                }
            }
        }

        if((mytimer.get_time_micro() - t_last) > 1000000)
        {
            std::cerr 
                << pct << "%; " << ++num_contractions 
                << " /  " << total_nodes
                << "; current: " << cid 
                << " in: " << n->in_degree() << " out: " << n->out_degree() 
                << " eadd " << eadd << std::endl;
                std::cerr << std::flush;
                t_last = mytimer.get_time_micro();
        }
    }

    std::cerr 
        << "\ngraph, contracted. "
        << " time (s): " << 
        (mytimer.get_time_micro() - t_begin) / 1000000.0
        << ". edges before " << edges_before 
        << "; edges after " << g_->get_num_edges_out() << std::endl;
}

uint32_t 
warthog::ch::fixed_graph_contraction::next()
{
    if(order_index_ < order_->size())
    {
        return order_->at(order_index_++);
    }
    return warthog::INF;
}

// NB: assumes the via-node is already contracted
double
warthog::ch::fixed_graph_contraction::witness_search(
        uint32_t from_id, uint32_t to_id, double via_len,
        uint32_t max_expand)
{
    // pathfinding queries must specify an external start and target id
    // (i.e. as they appear in the input file)
    warthog::graph::xy_graph* g = this->get_graph();
    uint32_t ext_from_id = g->to_external_id(from_id);
    uint32_t ext_to_id = g->to_external_id(to_id);

    // run the search
    alg_->set_cost_cutoff(via_len);
    alg_->set_max_expansions_cutoff(max_expand);
    warthog::problem_instance pi(ext_from_id, ext_to_id);
    warthog::solution sol;
    alg_->get_distance(pi, sol);

    // metrics
    total_expansions_ += sol.nodes_expanded_;
    total_searches_++;
    return sol.sum_of_edge_costs_;
}

size_t
warthog::ch::fixed_graph_contraction::mem()
{
    return 
        alg_->mem() + 
        sizeof(this);
}
