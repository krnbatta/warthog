#ifndef WARTHOG_CHASE_SEARCH_H
#define WARTHOG_CHASE_SEARCH_H

// search/chase_search.h
//
// An implementation of the CHASE algorithm. This method is a two-stage
// bi-directional search with contraction hierarchies. The CH graph
// is divided into a "core" set of nodes, defined as the top x% of the
// hierarchy. 
//
// In the first stage of the search, CHASE employs vanilla BCH in the 
// non-core part of the graph. Any successors which reside in the core 
// have their g-values relaxed but their expansion is deferred unless
// it can be proven the optimal path must involve some nodes from the core.
//
// In the second phase the method applies BCH plus arcflags in order to
// speed up search inside the core.
//
// For more theoretical details see:
//
// [Bauer, Delling, Sanders, Schieferdecker, Schultes and Wagner, 
// Combining Hierarchical and Goal-directed Speed-up Techniques 
// for Dijkstra's Algorithm, Journal of Experimental Algorithms,
// vol 15, 2010]
//
// @author: dharabor
// @created: 2016-09-10
//

#include "constants.h"
#include "xy_graph.h"
#include "pqueue.h"
#include "search.h"
#include "search_node.h"
#include "timer.h"
#include "zero_heuristic.h"

#include "constants.h"
#include <cstdlib>
#include <stack>
#include <stdint.h>

namespace warthog
{

//template<class E, class H>
template<class H>
class chase_search : public warthog::search
{
    public:
        chase_search(
                warthog::chase_expansion_policy* fexp, 
                warthog::chase_expansion_policy* bexp, 
                H* heuristic,
                std::vector<uint32_t>* rank, 
                double core_pct)
            : fexpander_(fexp), bexpander_(bexp), heuristic_(heuristic)
        {
            fopen_ = new pqueue_min(512);
            bopen_ = new pqueue_min(512);
            
            dijkstra_ = false;
            if(typeid(*heuristic_) == typeid(warthog::zero_heuristic))
            {
                dijkstra_ = true;
            }
            
            rank_ = rank;
            max_phase1_rank_ = fexpander_->get_num_nodes()*(1-core_pct);
        }

        ~chase_search()
        {
            fexpander_->reclaim();
            bexpander_->reclaim();
            fopen_->clear();
            bopen_->clear();
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
            if(best_cost_ != warthog::INF) 
            { 
                sol.sum_of_edge_costs_ = best_cost_;
            }
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
        warthog::pqueue_min* fopen_;
        warthog::pqueue_min* bopen_;
        warthog::chase_expansion_policy* fexpander_;
        warthog::chase_expansion_policy* bexpander_;
        H* heuristic_;
        bool dijkstra_;
        bool forward_next_;

        // CHASE-specific stuff
        uint32_t phase_;
        uint32_t max_phase1_rank_;
        std::vector<warthog::search_node*> fwd_norelax_;
        std::vector<warthog::search_node*> bwd_norelax_;
        std::vector<uint32_t>* rank_;

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
            while(current)
            {
               sol.path_.push_back(current->get_id());
               current = bexpander_->generate(current->get_parent());
            }
            std::reverse(sol.path_.begin(), sol.path_.end());

            current = fexpander_->generate(w_->get_parent());
            while(current)
            {  
               sol.path_.push_back(current->get_id());
               current = fexpander_->generate(current->get_parent());
            }
        }

        void 
        search(warthog::solution& sol)
        {
            std::cerr << "CHASE_SEARCH IS BROKEN\n" << std::endl;
            return;
            warthog::timer mytimer;
            mytimer.start();

            // init data structures used during search
            fopen_->clear();
            bopen_->clear();
            fwd_norelax_.clear();
            bwd_norelax_.clear();
            fexpander_->reset();
            bexpander_->reset();

            // init search parameters
            forward_next_ = true;
            best_cost_ = warthog::INF;
            v_ = w_ = 0;

            warthog::search_node *start, *target;
            start = fexpander_->generate_start_node(&pi_);
            target = bexpander_->generate_target_node(&pi_);
            pi_.start_id_ = start->get_id();
            pi_.target_id_ = target->get_id();
            
            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                std::cerr << "chase_search. ";
                pi_.print(std::cerr);
                std::cerr << std::endl;
            }
            #endif

            // initialise the start and target
            start->init(pi_.instance_id_, warthog::NODE_NONE, 
                    0, heuristic_->h(pi_.start_id_, pi_.target_id_));
            target->init(pi_.instance_id_, warthog::NODE_NONE, 
                    0, heuristic_->h(pi_.start_id_, pi_.target_id_));

            // these variables help interleave the search, decide when to
            // switch phases and when to terminate
            phase_ = 1;
            bool cannot_improve = false;
            double fwd_core_lb = DBL_MAX;
            double bwd_core_lb = DBL_MAX;
            uint32_t search_direction = 1;

            // search begin
            fopen_->push(start);
            bopen_->push(target);
            while(true)
            {
                // expand something
                switch(search_direction)
                {
                    case 1:
                    {
                        // expand in the forward direction
                        warthog::search_node* current = fopen_->pop();
                        expand(current, fopen_, fexpander_, bexpander_, 
                                pi_.target_id_, fwd_norelax_, 
                                fwd_core_lb, sol);

                        // switch directions
                        if( bopen_->size() > 0 && 
                           bopen_->peek()->get_f() < best_cost_ )
                        { search_direction = 2; }
                        else
                        {
                            cannot_improve = 
                               fopen_->size() == 0 ||
                               fopen_->peek()->get_f() > best_cost_;
                        }
                        break;
                    }
                    case 2:
                    {
                        // expand in the backwards direction
                        warthog::search_node* current = bopen_->pop();
                        expand(current, bopen_, bexpander_, fexpander_, 
                                pi_.target_id_, bwd_norelax_, 
                                bwd_core_lb, sol);

                        // switch directions
                        if( fopen_->size() > 0 && 
                            fopen_->peek()->get_f() < best_cost_)
                        { search_direction = 1; }
                        else 
                        {
                            cannot_improve = 
                                bopen_->size() == 0 ||
                                bopen_->peek()->get_f() > best_cost_;
                        }
                        break; 
                    }
                }
                if(pi_.verbose_)
                {
                    std::cerr 
                        << "best_cost " << best_cost_ 
                        << " fwd_ub: " << fwd_core_lb 
                        << " bwd_ub: " << bwd_core_lb
                        << std::endl;
                }


                if(cannot_improve)
                {
                    if(phase_ == 1)
                    {
                        uint32_t fwd_lower_bound = 
                            std::min(fwd_core_lb, fopen_->size() > 0 ?  
                                        fopen_->peek()->get_f() : DBL_MAX);

                        uint32_t bwd_lower_bound = 
                            std::min(bwd_core_lb, bopen_->size() > 0 ?  
                                        bopen_->peek()->get_f() : DBL_MAX);

                        uint32_t best_bound = 
                            std::min(fwd_lower_bound, bwd_lower_bound);
                        
                        // early terminate; optimal path does not involve
                        // any nodes from the core 
                        if(best_bound >= best_cost_)
                        {
                            #ifndef NDEBUG
                            if(pi_.verbose_)
                            {
                                std::cerr 
                                    << "provably-best solution found; "
                                    << "cost=" << best_cost_ << std::endl;
                            }
                            #endif
                            break; 
                        }

                        // early terminate if we can't reach the core
                        // in both directions
                        if(fwd_core_lb == DBL_MAX || bwd_core_lb == DBL_MAX)
                        { break; }
                        

                        // both directions can reach the core; time for phase2
                        fopen_->clear();
                        bopen_->clear();
                        for(uint32_t i = 0; i < fwd_norelax_.size(); i++)
                        {
                            fopen_->push(fwd_norelax_.at(i));
                        }
                        for(uint32_t i = 0; i < bwd_norelax_.size(); i++)
                        {
                            bopen_->push(bwd_norelax_.at(i));
                        }
                         
                        // reset variables that control the search
                        phase_ = 2;
                        fexpander_->begin_phase2();
                        bexpander_->begin_phase2();
                        cannot_improve = false;
                        fwd_core_lb = bwd_core_lb = DBL_MAX;
                        search_direction = 1;

                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            std::cerr << "=== PHASE2 ===" << std::endl;
                        }
                        #endif
                    }
                    else 
                    {
                        // phase 2 complete
                        #ifndef NDEBUG
                        if(pi_.verbose_)
                        {
                            if(best_cost_ != warthog::INF)
                            {
                                std::cerr 
                                    << "provably-best solution found; "
                                    << "cost=" << best_cost_ << std::endl;
                            }
                            else
                            {
                                std::cerr 
                                    << "no solution exists" << std::endl;
                            }
                        }
                        #endif
                        break; 
                    } 
                }
            }

			mytimer.stop();
			sol.time_elapsed_nano_= mytimer.elapsed_time_nano();
            assert(best_cost_ != warthog::INF || (v_ == 0 && w_ == 0));
        }

        void
        expand( warthog::search_node* current,
                warthog::pqueue_min* open,
                warthog::chase_expansion_policy* expander,
                warthog::chase_expansion_policy* reverse_expander, 
                uint32_t tmp_targetid,
                std::vector<warthog::search_node*>& norelax, 
                double&  norelax_distance_min,
                warthog::solution& sol)
        {
            // target not found yet; expand as normal
            current->set_expanded(true);
            expander->expand(current, &pi_);
            sol.nodes_expanded_++;

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr 
                    << sol.nodes_expanded_ 
                    << ". expanding " 
                    << (&*open == &*fopen_ ? "(f)" : "(b)")
                    << " ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
            }
            #endif
            
            // generate all neighbours
            warthog::search_node* n = 0;
            double cost_to_n = warthog::INF;
            for(expander->first(n, cost_to_n); n != 0; expander->next(n, cost_to_n))
            {
                sol.nodes_touched_++;

                // add new nodes to the fringe
                if(n->get_search_id() != current->get_search_id())
                {
                    double gval = current->get_g() + cost_to_n;
                    n->init(current->get_search_id(), current->get_id(), gval,
                            gval + 
                            heuristic_->h(n->get_id(), tmp_targetid));

                    // during phase1, only new nodes whose rank is below the 
                    // limit are queued on the open list
                    // during phase2, all new nodes are queued
                    if( phase_ == 2 || rank_->at(n->get_id()) < max_phase1_rank_ )
                    {
                        sol.nodes_inserted_++;
                        open->push(n);
                    }
                    else
                    {
                        norelax.push_back(n);
                        if(gval < norelax_distance_min)
                        {
                            norelax_distance_min = gval; 
                        }
                    }

                    #ifndef NDEBUG
                    if(pi_.verbose_)
                    {
                        if( phase_ == 1 &&
                            rank_->at(n->get_id()) >= max_phase1_rank_ )
                        {
                            std::cerr << "phase2-list ";
                        }
                        else
                        {
                            std::cerr << "generating ";
                        }

                        int32_t x, y;
                        expander->get_xy(n->get_id(), x, y);
                        std::cerr 
                            << "(edgecost=" << cost_to_n<<") " 
                            << "("<<x<<", "<<y<<")...";
                        n->print(std::cerr);
                        std::cerr << std::endl;
                    }
                    #endif
                }

                // update neighbours the search has seen before
                else
                {
                    if(n->get_expanded())
                    {
                        // skip neighbours already expanded
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

                    // relax (or generate) each neighbour
                    double gval = current->get_g() + cost_to_n;
                    if(open->contains(n))
                    {
                        // update a node from the fringe
                        if(gval < n->get_g())
                        {
                            sol.nodes_updated_++;
                            n->relax(gval, current->get_parent());
                            open->decrease_key(n);
                            #ifndef NDEBUG
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << " updating "
                                    << "(edgecost="<< cost_to_n<<") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                            }
                            #endif
                        }
                        else
                        {
                            #ifndef NDEBUG
                            if(pi_.verbose_)
                            {
                                int32_t x, y;
                                expander->get_xy(n->get_id(), x, y);
                                std::cerr << " not updating "
                                    << "(edgecost=" << cost_to_n<< ") "
                                    << "("<<x<<", "<<y<<")...";
                                n->print(std::cerr);
                                std::cerr << std::endl;
                            }
                            #endif
                        }
                    }
                    else
                    {
                        // relax the g-value of the nodes not being
                        // expanded in phase1
                        // (these are not added to open yet)
                        assert(phase_ == 1);
                        if(gval < n->get_g())
                        {
                            n->relax(gval, current->get_parent());
                            if(gval < norelax_distance_min)
                            {
                                norelax_distance_min = gval; 
                            }
                        }
                    }
                }

                // update the best solution if possible
                warthog::search_node* reverse_n = 
                    reverse_expander->generate(n->get_id());
                if(reverse_n->get_search_id() == n->get_search_id())
//                        && reverse_n->get_expanded())
                {
                    if((current->get_g() + cost_to_n + reverse_n->get_g()) < best_cost_)
                    {
                        v_ = n;
                        w_ = reverse_n;
                        best_cost_ = current->get_g() + cost_to_n + reverse_n->get_g();

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
            }

            #ifndef NDEBUG
            if(pi_.verbose_)
            {
                int32_t x, y;
                expander->get_xy(current->get_id(), x, y);
                std::cerr <<"closing ("<<x<<", "<<y<<")...";
                current->print(std::cerr);
                std::cerr << std::endl;
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

};

}

#endif

