#include "cbs_ll_heuristic.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "labelled_gridmap.h"
#include "evl_gridmap_expansion_policy.h"
#include "vl_gridmap_expansion_policy.h"
#include "zero_heuristic.h"

#include <functional>
#include <stdint.h>

void
warthog::cbs_ll_heuristic::compute_h_values(
        std::vector<uint32_t>& targets,
        warthog::gridmap* map)
{
    t_map_.clear();
    h_.clear();
    h_.resize(targets.size());
    t_index_ = 0;

    uint32_t bitwidth_map = 
        32 - __builtin_clz(map->width() * map->height());
    id_mask_ = (1 << bitwidth_map)-1;

    warthog::pqueue_min open;
    warthog::zero_heuristic h;
    warthog::gridmap_expansion_policy expander(map, true);

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::gridmap_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &expander, &open);

    std::function<void(warthog::search_node*)> on_expand_fn = 
        [&] (warthog::search_node* current) -> void
        {
            uint32_t t_index = t_map_.size() - 1;
            //uint32_t h_sz = h_.size();
            //uint32_t h_sz_2 = h_.at(t_index).size();
            //h_sz = h_sz + 1 - 1;
            //h_sz_2 = h_sz_2 + 1 - 1;
            h_[t_index][current->get_id()] = current->get_g();
            //uint32_t tmp = h_[t_index][current->get_id()];
            //assert(tmp == current->get_g());
        };
    alg.apply_on_expand(on_expand_fn);

    for(uint32_t target_id : targets)
    {
        uint32_t t_index = t_map_.size();
        t_map_[target_id] = t_index;
        h_[t_index_].resize(map->width() * map->height(), warthog::INF);
        warthog::problem_instance problem(target_id, warthog::INF);
        warthog::solution sol;
        alg.get_distance(problem, sol);
    }
}

void
warthog::cbs_ll_heuristic::compute_h_values(
        std::vector<uint32_t>& targets,
        warthog::vl_gridmap* map)
{
    t_map_.clear();
    h_.clear();
    h_.resize(targets.size());
    t_index_ = 0;

    uint32_t bitwidth_map = 
        32 - __builtin_clz(map->width() * map->height());
    id_mask_ = (1 << bitwidth_map)-1;

    warthog::pqueue_min open;
    warthog::zero_heuristic h;
    warthog::vl_gridmap_expansion_policy expander(map);

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::vl_gridmap_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &expander, &open);

    std::function<void(warthog::search_node*)> on_expand_fn = 
        [&] (warthog::search_node* current) -> void
        {
            uint32_t t_index = t_map_.size() - 1;
            //uint32_t h_sz = h_.size();
            //uint32_t h_sz_2 = h_.at(t_index).size();
            //h_sz = h_sz + 1 - 1;
            //h_sz_2 = h_sz_2 + 1 - 1;
            h_[t_index][current->get_id()] = current->get_g();
            //uint32_t tmp = h_[t_index][current->get_id()];
            //assert(tmp == current->get_g());
        };
    alg.apply_on_expand(on_expand_fn);

    for(uint32_t target_id : targets)
    {
        uint32_t t_index = t_map_.size();
        t_map_[target_id] = t_index;
        h_[t_index_].resize(map->width() * map->height(), warthog::INF);
        warthog::problem_instance problem(target_id, warthog::INF);
        warthog::solution sol;
        alg.get_distance(problem, sol);
    }
}

void
warthog::cbs_ll_heuristic::compute_h_values(
        std::vector<uint32_t>& targets,
        warthog::evl_gridmap* map)
{
    t_map_.clear();
    h_.clear();
    h_.resize(targets.size());
    t_index_ = 0;

    uint32_t bitwidth_map = 
        32 - __builtin_clz(map->width() * map->height());
    id_mask_ = (1 << bitwidth_map)-1;

    warthog::pqueue_min open;
    warthog::zero_heuristic h;
    warthog::evl_gridmap_expansion_policy expander(map);

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::evl_gridmap_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &expander, &open);

    std::function<void(warthog::search_node*)> on_expand_fn = 
        [&] (warthog::search_node* current) -> void
        {
            uint32_t t_index = t_map_.size() - 1;
            //uint32_t h_sz = h_.size();
            //uint32_t h_sz_2 = h_.at(t_index).size();
            //h_sz = h_sz + 1 - 1;
            //h_sz_2 = h_sz_2 + 1 - 1;
            h_[t_index][current->get_id()] = current->get_g();
            //uint32_t tmp = h_[t_index][current->get_id()];
            //assert(tmp == current->get_g());
        };
    alg.apply_on_expand(on_expand_fn);

    for(uint32_t target_id : targets)
    {
        uint32_t t_index = t_map_.size();
        t_map_[target_id] = t_index;
        h_[t_index_].resize(map->width() * map->height(), warthog::INF);
        warthog::problem_instance problem(target_id, warthog::INF);
        warthog::solution sol;
        alg.get_distance(problem, sol);
    }
}
