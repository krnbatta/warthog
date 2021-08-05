#ifndef WARTHOG_BIDIRECTIONAL_SEARCH_H
#define WARTHOG_BIDIRECTIONAL_SEARCH_H

// bidirectional_search.h
//
// A customisable variant of bidirectional best-first search.
// Users can pass in any heuristic and any (domain-specific) expansion policy.
//
// @author: dharabor
// @created: 2016-02-14
//

#include "constants.h"
#include "graph_expansion_policy.h"
#include "xy_graph.h"
#include "pqueue.h"
#include "search.h"
#include "search_node.h"
#include "solution.h"
#include "timer.h"
#include "zero_heuristic.h"
#include "bi_debug.h"

#include "constants.h"

#include <cstdlib>
#include <stack>
#include <cstdint>
#include <typeinfo>

namespace warthog
{

typedef double (* heuristicFn)
(uint32_t nodeid, uint32_t targetid);

template<class H, class E>
class bidirectional_search : public warthog::search
{
    public:
        bidirectional_search(E* fexp, E* bexp, H* heuristic)
            : fexpander_(fexp), bexpander_(bexp), heuristic_(heuristic)
        {
            fopen_ = new pqueue_min(512);
            bopen_ = new pqueue_min(512);

            dijkstra_ = false;
            if(typeid(*heuristic_) == typeid(warthog::zero_heuristic))
            {
                dijkstra_ = true;
            }

            exp_cutoff_ = warthog::INF;
            cost_cutoff_ = warthog::INF;
        }

        ~bidirectional_search()
        {
            delete fopen_;
            delete bopen_;
        }

        virtual void
        get_path(warthog::problem_instance& pi, warthog::solution& sol)
        {
            pi_ = pi;
            this->search(sol);
            if(best_cost_ != warthog::INF)
            {
                sol.sum_of_edge_costs_ = best_cost_;
                reconstruct_path(sol);
            }

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "path: \n";
                for(uint32_t i = 0; i < sol.path_.size(); i++)
                {
                    std::cerr << sol.path_.at(i) << std::endl;
                }
            }
            #endif
        }

        virtual void
        get_distance(warthog::problem_instance& pi, warthog::solution& sol)
        {
            pi_ = pi;
            this->search(sol);
            if(sol.nodes_expanded_ > exp_cutoff_ ) { std::cerr << "wtf!"; exit(1); }
            if(best_cost_ != warthog::INF)
            { sol.sum_of_edge_costs_ = best_cost_; }
        }

        // set a cost-cutoff to run a bounded-cost A* search.
        // the search terminates when the target is found or the f-cost
        // limit is reached.
        inline void
        set_cost_cutoff(double cutoff) { cost_cutoff_ = cutoff; }

        inline double
        get_cost_cutoff() { return cost_cutoff_; }

        // set a cutoff on the maximum number of node expansions.
        // the search terminates when the target is found or when
        // the limit is reached
        inline void
        set_max_expansions_cutoff(uint32_t cutoff) { exp_cutoff_ = cutoff; }

        inline uint32_t
        get_max_expansions_cutoff() { return exp_cutoff_; }

        warthog::search_node*
        get_search_node(uint32_t id, int direction=0)
        {
            if(direction == 0)
                return fexpander_->get_ptr(id, pi_.instance_id_);
            return bexpander_->get_ptr(id, pi_.instance_id_);
        }

        size_t
        mem()
        {
            return sizeof(*this) +
                fopen_->mem() +
                bopen_->mem() +
                fexpander_->mem();
                bexpander_->mem();
        }

    private:
        Bidebugger debugger;
        warthog::pqueue_min* fopen_;
        warthog::pqueue_min* bopen_;
        E* fexpander_;
        E* bexpander_;
        H* heuristic_;
        bool dijkstra_;

        // early termination limits
        double cost_cutoff_;
        uint32_t exp_cutoff_;

        // v is the section of the path in the forward
        // direction and w is the section of the path
        // in the backward direction. need parent pointers
        // of both to extract the actual path
        warthog::search_node* v_;
        warthog::search_node* w_;
        double best_cost_;
        warthog::problem_instance pi_;

        void
        reconstruct_path(warthog::solution& sol)
        {
            if(v_ && (&*v_ == &*bexpander_->generate(v_->get_id())))
            {
                warthog::search_node* tmp = v_;
                v_ = w_;
                w_ = tmp;
            }

            warthog::search_node* current = v_;
            while(true)
            {
               sol.path_.push_back(current->get_id());
               if(current->get_parent() == warthog::NODE_NONE) break;
               current = fexpander_->generate(current->get_parent());

            }
            std::reverse(sol.path_.begin(), sol.path_.end());

            current = w_;
            while(current->get_parent() != warthog::NODE_NONE)
            {
               sol.path_.push_back(current->get_parent());
               current = bexpander_->generate(current->get_parent());
            }
        }

        // modify this function to balance the search
        // by default the search expands one node in
        // each direction then switches to the other direction
        bool
        forward_next()
        {
            double fwd_min, bwd_min;
            bwd_min = bopen_->size() ? bopen_->peek()->get_f() : warthog::INF;
            fwd_min = fopen_->size() ? fopen_->peek()->get_f() : warthog::INF;
            return fwd_min < bwd_min;
        }

        void
        search(warthog::solution& sol)
        {
            warthog::timer mytimer;
            mytimer.start();

            // init
            best_cost_ = warthog::INF;
            v_ = w_ = 0;
            fopen_->clear();
            bopen_->clear();

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "bidirectional_search. ";
                pi_.print(std::cerr);
                std::cerr << std::endl;
            }
            #endif

            // generate the start and target nodes.
            warthog::search_node *start, *target;
            start = fexpander_->generate_start_node(&pi_);
            target = bexpander_->generate_target_node(&pi_);
            start->init(pi_.instance_id_, warthog::NODE_NONE,
                    0, heuristic_->h(start->get_id(), target->get_id()));
            target->init(pi_.instance_id_, warthog::NODE_NONE,
                    0, heuristic_->h(start->get_id(), target->get_id()));
            fopen_->push(start);
            bopen_->push(target);

            // also update problem instance with internal ids (for debugging)
            pi_.start_id_ = start->get_id();
            pi_.target_id_ = target->get_id();

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                debugger = Bidebugger();

                int32_t x, y;
                fexpander_->get_xy(start->get_id(), x, y);
                std::cerr
                    << "0"
                    << ". Start Node "
                    << "(f)"
                    << " ("<<x<<", "<<y<<")...";
                start->print(std::cerr);
                debugger.addSource(start, "f-" + std::to_string(start->get_id()+1), x, y, "f-"+std::to_string(start->get_parent()+1));

                int32_t start_x = x;
                int32_t start_y = y;
                bexpander_->get_xy(target->get_id(), x, y);
                std::cerr
                    << "0"
                    << ". End Node "
                    << "(b)"
                    << " ("<<x<<", "<<y<<")...";
                target->print(std::cerr);
                debugger.addDestination(target, "b-" + std::to_string(target->get_id()+1), x, y, "b-"+std::to_string(target->get_parent()+1));

                debugger.AddEvent("generating", start, "f-" + std::to_string(start->get_id()+1), start_x, start_y, "f-"+std::to_string(start->get_parent()+1));

                std::cerr << std::endl;
            }
            #endif
            E* cur_expander;
            std::string sdir;
            // expand
            while(fopen_->size() || bopen_->size())
            {
                double fwd_bound = fopen_->size() ?
                    fopen_->peek()->get_f() : warthog::INF;
                double bwd_bound = bopen_->size() ?
                    bopen_->peek()->get_f() : warthog::INF;
                uint32_t best_bound = dijkstra_ ?
                    (fwd_bound + bwd_bound) : std::min(fwd_bound, bwd_bound);

                if(best_bound > best_cost_) { break; }
                if(best_bound > cost_cutoff_) { break; }
                if(sol.nodes_expanded_ >= exp_cutoff_) { break; }

                // always expand the most promising node in either direction
                if(forward_next())
                {
                    warthog::search_node* current = fopen_->pop();
                    expand(current, fopen_, fexpander_, bexpander_,
                            pi_.target_id_, sol);
                    cur_expander = fexpander_;
                    sdir = "f";
                    #ifndef NDEBUG
                    if(pi_.verbose_){
                      if(!(fopen_->size() || bopen_->size())){
                        int32_t x, y;
                        fexpander_->get_xy(start->get_id(), x, y);
                        debugger.endSearch(current,  "f-" + std::to_string(current->get_id()+1), x, y, "f-"+std::to_string(current->get_parent()+1));
                        debugger.printToDebugFile();
                      }
                    }
                    #endif
                }
                else
                {
                    warthog::search_node* current = bopen_->pop();
                    expand(current, bopen_, bexpander_, fexpander_,
                            pi_.start_id_, sol);
                    cur_expander = bexpander_;
                    sdir = "b";
                    #ifndef NDEBUG
                    if(pi_.verbose_){
                      if(!(fopen_->size() || bopen_->size())){
                        int32_t x, y;
                        bexpander_->get_xy(start->get_id(), x, y);
                        debugger.endSearch(current,  "b-" + std::to_string(current->get_id()+1), x, y, "b-"+std::to_string(current->get_parent()+1));
                        debugger.printToDebugFile();
                      }
                    }
                    #endif
                }
            }
            #ifndef NDEBUG
            if(pi_.verbose_){
              int32_t x, y;
              cur_expander->get_xy(target->get_id(), x, y);
              debugger.endSearch(target,  sdir + "-" + std::to_string(target->get_id()+1), x, y, sdir + "-" + std::to_string(target->get_parent()+1));
              debugger.printToDebugFile();
            }
            #endif

			mytimer.stop();
			sol.time_elapsed_nano_ = mytimer.elapsed_time_nano();

            assert(best_cost_ == warthog::INF || (v_ && w_));
        }
//jj
        void
        expand( warthog::search_node* current,
                warthog::pqueue_min* open, E* expander, E* reverse_expander,
                uint32_t tmp_targetid, warthog::solution& sol)
        {
            if(current == 0) { return; }
            current->set_expanded(true);
            expander->expand(current, &pi_);
            sol.nodes_expanded_++;

            std::string sdir = (pi_.target_id_ == tmp_targetid ? "f" : "b");

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr
                    << sol.nodes_expanded_
                    << ". expanding "
                    << (pi_.target_id_ == tmp_targetid ? "(f)" : "(b)")
                    << " ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
                debugger.AddEvent("expanding", current, sdir + "-" + std::to_string(current->get_id()+1), x, y, sdir + "-" + std::to_string(current->get_parent()+1));
            }
            #endif

            // update the best solution if possible
            warthog::search_node* rev_current =
                reverse_expander->generate(current->get_id());
            if(rev_current->get_search_id() == current->get_search_id())
            {
                if((current->get_g() + rev_current->get_g()) < best_cost_)
                {
                    v_ = current;
                    w_ = rev_current;
                    best_cost_ = current->get_g() + rev_current->get_g();

                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        int32_t x, y;
                        expander->get_xy(current->get_id(), x, y);
                        std::cerr <<"new best solution!  cost=" << best_cost_<<std::endl;
                    }
                    #endif
                }
            }

            // generate all neighbours
            warthog::search_node* n = 0;
            double cost_to_n = warthog::INF;
            for(expander->first(n, cost_to_n);
                    n != 0;
                    expander->next(n, cost_to_n))
            {
                sol.nodes_touched_++;

                // add new nodes to the fringe
                if(n->get_search_id() != current->get_search_id())
                {
                    double gval = current->get_g() + cost_to_n;
                    n->init(current->get_search_id(), current->get_id(),
                            gval,
                            gval + heuristic_->h(n->get_id(), tmp_targetid));
                    open->push(n);
                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        int32_t x, y;
                        expander->get_xy(n->get_id(), x, y);
                        std::cerr << "  generating "
                            << "(edgecost=" << cost_to_n<<") "
                            << "("<<x<<", "<<y<<")...";
                        n->print(std::cerr);
                        std::cerr << std::endl;
                        debugger.AddEvent("generating", n, sdir + "-" + std::to_string(n->get_id()+1), x, y, sdir + "-" + std::to_string(n->get_parent()+1));
                    }
                    #endif
                    sol.nodes_inserted_++;
                }

                // process neighbour nodes that the search has seen before
                else
                {
                    // skip neighbours already expanded
                    if(n->get_expanded())
                    {
                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            int32_t x, y;
                            expander->get_xy(n->get_id(), x, y);
                            std::cerr << "  closed; (edgecost=" << cost_to_n << ") "
                                << "("<<x<<", "<<y<<")...";
                            n->print(std::cerr);
                            std::cerr << std::endl;

                        }
                        #endif
                        continue;
                    }

                    // relax nodes on the fringe
                    if(open->contains(n))
                    {
                        double gval = current->get_g() + cost_to_n;
                        if(gval < n->get_g())
                        {
                            n->relax(gval, current->get_parent());
                            open->decrease_key(n);
                            sol.nodes_updated_++;
                            #ifndef NDEBUG
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << "  open; updating "
                                    << "(edgecost="<< cost_to_n<<") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                                debugger.AddEvent("updating", n, sdir + "-" + std::to_string(n->get_id()+1), x, y, sdir + "-" + std::to_string(n->get_parent()+1));
                            }
                            #endif
                        }
                        #ifndef NDEBUG
                        else
                        {
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << "  open; not updating "
                                    << "(edgecost=" << cost_to_n<< ") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                            }
                        }
                        #endif
                    }
                }

            }


            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr <<"closing ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
                debugger.AddEvent("closing", current, sdir + "-" + std::to_string(current->get_id()+1), x, y, sdir + "-" + std::to_string(current->get_parent()+1));
            }
            #endif
        }

        // clear the open lists and return all memory allocated for nodes
        // to the node pool
        void
        reclaim()
        {
            fopen_->clear();
            bopen_->clear();
            fexpander_->reclaim();
            bexpander_->reclaim();
        }

        inline double
        h(uint32_t id1, uint32_t id2)
        {
        }


};

}

#endif
