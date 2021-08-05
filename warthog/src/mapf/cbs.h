#ifndef WARTHOG_CBS_H
#define WARTHOG_CBS_H

// mapf/cbs.h
//
// Utilities for working with Conflict-based Search.
//
// @author: dharabor
// @created: 2018-11-06
//

#include "gridmap.h"
#include "pqueue.h"
#include "reservation_table.h"
#include "search_node.h"

namespace warthog
{

namespace cbs
{

// this data structure describes the constraints imposed on a single 
// grid cell. a brief description of the members:
//
// timestep_ is the time at which the constraints apply
// v_ is true if the cell is constrained / blocked
// e_ indicates which of the 8 outgoing directions are constrained / blocked
struct cell_constraints
{
    cell_constraints() 
        : timestep_(0), v_(0), e_(0) { }

    cell_constraints(uint16_t timestep, uint8_t v, uint8_t e) 
        : timestep_(timestep), v_(v), e_(e) { }

    uint16_t timestep_; 
    uint8_t v_;
    uint8_t e_;
};

// this data structure describes the all constraints that currently apply
class time_constraints
{
    public:

       time_constraints(uint32_t map_xy_sz) : map_xy_sz_(map_xy_sz)
       {
           cons_ = new std::vector< std::vector<cell_constraints> >(map_xy_sz_);
       } 

       ~time_constraints()
       {
           delete cons_;
       }

       inline void
       add_constraint(uint32_t padded_id, cell_constraints con)
       {
           assert(padded_id < cons_->size());
           cons_->at(padded_id).push_back(con);
       }

       // return all constraints associated with the xy location
       // @param padded_id
       inline std::vector<cell_constraints>& 
       get_constraints(uint32_t padded_id)
       {
           return cons_->at(padded_id);
       }

       // return any constraints associated with the xy location
       // @param padded_id at time @param timestep
       inline cell_constraints*
       get_constraints(uint32_t padded_id, uint32_t timestep)
       {
            cell_constraints* retval = &dummy_;
            std::vector<cell_constraints>::iterator con_iter = 
                std::find_if(
                        cons_->at(padded_id).begin(), 
                        cons_->at(padded_id).end(),
                    [timestep](warthog::cbs::cell_constraints& tmp)
                    -> bool
                    {
                        return tmp.timestep_  == timestep;
                    });
            if(con_iter != cons_->at(padded_id).end())
            {
                retval = &*con_iter;
            }
            return retval;
       }

       void
       clear_constraints(uint32_t padded_id)
       {
           cons_->at(padded_id).clear();
       }

       void
       clear_constraints()
       {
           for(uint32_t i = 0; i < cons_->size(); i++)
           {
               cons_->at(i).clear();
           }   
       }

    private:
        std::vector< std::vector<cell_constraints> >* cons_;
        warthog::cbs::cell_constraints dummy_;
        uint32_t map_xy_sz_;
};


// this data structure defines the lessthan comparator for 
// CBS low-level search. Tie-breaks as follows:
// 1. smaller f-value, then
// 2. larger g-value and !is_reserved, then
// 3. !is_reserved, then
// 4. larger g-value
class cmp_cbs_ll_lessthan
{
    public:
        cmp_cbs_ll_lessthan(warthog::reservation_table* restab)
            : restab_(restab)
        { 
            is_reserved_fn_ = &cmp_cbs_ll_lessthan::__is_reserved;
        }

        bool
        operator()(const warthog::search_node& first, const warthog::search_node& second)
        {
			if(first.get_f() < second.get_f())
			{
				return true;
			}

			if(first.get_f() == second.get_f())
			{
                // break ties in favour of larger g
                if(first.get_g() >= second.get_g())
                {
                    // but only if tile is not reserved
                    if((this->*(is_reserved_fn_))(first.get_id()))
                    {
                        if((this->*(is_reserved_fn_))(second.get_id()))
                        {
                            // both nodes are reserved; so we pick
                            // @param first which has a larger g-value
                            return true;
                        }
                        // @param first has a larger g-value but is_reserved
                        // @param second has a smaller g-value & !is_reserved 
                        // tie-break in favour of @param second
                        return false;
                    }
                    // @param first !is_reserved and it has a 
                    // smaller g-value. hooray.
                    return true; 
                }

                if((this->*(is_reserved_fn_))(second.get_id()))
                {
                    if((this->*(is_reserved_fn_))(first.get_id()))
                    {
                        // both @param first and @param second are reserved
                        // we choose @param second since it has a larger g-value
                        return false;
                    }
                    // @param second has a larger g-value and is_reserved 
                    // @param first has a smaller g-value & !is_reserved
                    // we choose @param first
                    return true;
                    
                }

                // @param second has larger g-value and !is_reserved and so
                // @param first is strictly dominated
                // we choose @param second
                return false;
			}
            return false;
        }
    
    private:
        typedef bool(cmp_cbs_ll_lessthan::*fn_is_reserved)(uint32_t time_indexed_id);

        warthog::reservation_table* restab_;
        fn_is_reserved  is_reserved_fn_;

        bool
        __is_reserved_dummy(uint32_t) { return false; }

        bool
        __is_reserved(uint32_t map_id)
        { 
            return restab_->is_reserved(map_id);
        }
};

typedef pqueue<cmp_cbs_ll_lessthan> pqueue_cbs_ll;

}

}

#endif
