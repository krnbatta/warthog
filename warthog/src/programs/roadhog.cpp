// programs/roadhog.cpp 
//
// Pulls together a variety of different algorithms for 
// routing on road graphs.
//
// @author: dharabor
// @created: 2016-11-24
//

#include "af_filter.h"
#include "apex_filter.h"
#include "bbaf_filter.h"
#include "bb_filter.h"
#include "bch_search.h"
#include "bidirectional_search.h"
#include "cfg.h"
#include "bch_expansion_policy.h"
#include "bch_af_expansion_policy.h"
#include "bch_bbaf_expansion_policy.h"
#include "bch_bb_expansion_policy.h"
#include "chase_expansion_policy.h"
#include "chase_search.h"
#include "constants.h"
#include "contraction.h"
#include "corner_point_graph.h"
#include "dimacs_parser.h"
#include "euclidean_heuristic.h"
#include "fch_af_expansion_policy.h"
#include "fch_bb_expansion_policy.h"
#include "fch_bbaf_expansion_policy.h"
#include "fch_dfs_expansion_policy.h"
#include "fch_fm_expansion_policy.h"
#include "fch_expansion_policy.h"
#include "fch_x_expansion_policy.h"
#include "firstmove_labelling.h"
#include "fixed_graph_contraction.h"
#include "flexible_astar.h"
#include "graph_expansion_policy.h"
#include "lazy_graph_contraction.h"
#include "xy_graph.h"
#include "solution.h"
#include "timer.h"
#include "zero_heuristic.h"

// stuff for searching corner point graphs
#include "ch_cpg_expansion_policy.h"
#include "fch_af_cpg_expansion_policy.h"
#include "fch_bb_cpg_expansion_policy.h"
#include "fch_bbaf_cpg_expansion_policy.h"
#include "fch_cpg_expansion_policy.h"

#include "fch_jpg_expansion_policy.h"
#include "fch_bb_jpg_expansion_policy.h"
#include "fch_af_jpg_expansion_policy.h"

#include "getopt.h"

#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unordered_map>

// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;

// suppress the header row when printing results? (default: no)
int suppress_header = 0;

uint32_t nruns = 1;

void
help()
{
	std::cerr << "Valid parameters:\n"
    << "\t--alg [ algorithm name (required) ]\n"
    << "\t--input [ algorithm-specific input files (omit to show options) ] \n"
    << "\t--problem [ ss or p2p problem file (required) ]\n"
	<< "\t--verbose (print debug info; omitting this param means no)\n"
	<< "\t--nruns [int (repeats per instance; default=" << nruns << ")]\n"
    << "\nRecognised values for --alg:\n"
    << "\tastar, dijkstra, bi-astar, bi-dijkstra\n"
    << "\tbch, bch-astar, bch-af, bch-bb, bch-bbaf, chase, ch-cpg\n"
    << "\tfch, fchx, fch-af, fch-bb, fch-bbaf, fch-dfs\n"
    << "\tfch-cpg, fch-af-cpg, fch-bb-cpg, fch-bbaf-cpg\n"
    << "\tfch-jpg, fch-bb-jpg, fch-af-jpg\n"
    << "\nRecognised values for --input:\n "
    << "\ttoo many to list. missing input files will be listed at runtime\n";
}

////////////////////////////////////////////////////////////////////////////
// ----- These two closures were useful once upon a time. Maybe again? --- 
////////////////////////////////////////////////////////////////////////////
//    std::function<uint32_t(warthog::search_node*)> fn_get_apex = 
//        [&order] (warthog::search_node* n) -> uint32_t
//        {
//            while(true)
//            {
//                warthog::search_node* p = n->get_parent();
//                if(!p || order.at(p->get_id()) < order.at(n->get_id()))
//                { break; }
//                n = p;
//            }
//            return order.at(n->get_id());
//        };
//
//    std::function<uint32_t(uint32_t)> fn_redundant_expansions = 
//        [&fexp, &order, &alg] (uint32_t apex) -> uint32_t
//        {
//            std::set<uint32_t> tmp;
//            for(uint32_t i = 0; i < order.size(); i++)
//            {
//                warthog::search_node* n = fexp.get_ptr(i, alg.get_search_id());
//                if(n && order.at(n->get_id()) > apex)
//                { 
//                    tmp.insert(n->get_id());
//                }
//            }
//
//            for(uint32_t j = 0; j < order.size(); j++)
//            {
//                warthog::search_node* n = fexp.get_ptr(j, alg.get_search_id());
//                warthog::search_node* m = n;
//                while(m)
//                {
//                    if(tmp.find(m->get_id()) != tmp.end())
//                    {
//                        while(tmp.find(n->get_id()) == tmp.end())
//                        {
//                            tmp.insert(n->get_id());
//                            n = n->get_parent();
//                        }
//                        break;
//                    }
//                    m = m->get_parent();
//                }
//            }
//            return tmp.size();
//        };

void
run_experiments( warthog::search* algo, std::string alg_name, 
        warthog::dimacs_parser& parser, std::ostream& out)
{
    std::cerr << "running experiments\n";
    std::cerr << "(averaging over " << nruns << " runs per instance)\n";

    if(!suppress_header)
    {
        std::cout 
            << "id\talg\texpanded\tinserted\tupdated\ttouched"
            << "\tnanos\tpcost\tplen\tmap\n";
    }
    uint32_t exp_id = 0;
    for(auto it = parser.experiments_begin(); 
            it != parser.experiments_end(); 
            it++)
    {
        warthog::dimacs_parser::experiment exp = (*it);
        warthog::solution sol;
        warthog::problem_instance pi(
                exp.source, (exp.p2p ? exp.target : warthog::INF), verbose);
        uint32_t expanded=0, inserted=0, updated=0, touched=0;
        double nano_time = DBL_MAX;
        for(uint32_t i = 0; i < nruns; i++)
        {
            sol.reset();
            algo->get_distance(pi, sol);

            expanded += sol.nodes_expanded_;
            inserted += sol.nodes_inserted_;
            touched += sol.nodes_touched_;
            updated += sol.nodes_updated_;
            nano_time = nano_time < sol.time_elapsed_nano_ 
                            ?  nano_time : sol.time_elapsed_nano_;
        }

        out
            << exp_id++ <<"\t" 
            << alg_name << "\t" 
            << expanded / nruns << "\t" 
            << inserted / nruns << "\t"
            << updated / nruns << "\t"
            << touched / nruns << "\t"
            << nano_time << "\t" /// (double)nruns << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << (sol.path_.size()-1) << "\t" 
            << parser.get_problemfile() 
            << std::endl;
    }
}

void
run_astar(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str()))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }

    warthog::simple_graph_expansion_policy expander(&g);
    warthog::euclidean_heuristic h(&g);
    warthog::pqueue_min open;

    warthog::flexible_astar<
        warthog::euclidean_heuristic, 
        warthog::simple_graph_expansion_policy, 
        warthog::pqueue_min> 
            alg(&h, &expander, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bi_astar(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }

    warthog::simple_graph_expansion_policy fexp(&g);

    warthog::graph::xy_graph backward_g;
    if(!backward_g.load_from_dimacs(gr.c_str(), co.c_str(), true, true))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }
    warthog::simple_graph_expansion_policy bexp(&g);

    warthog::euclidean_heuristic h(&g);
    warthog::bidirectional_search<
        warthog::euclidean_heuristic,
        warthog::simple_graph_expansion_policy> 
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bi_dijkstra(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str()))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }
    warthog::simple_graph_expansion_policy fexp(&g);
    warthog::simple_graph_expansion_policy bexp(&g);

    warthog::zero_heuristic h;
    warthog::bidirectional_search<
        warthog::zero_heuristic, warthog::simple_graph_expansion_policy>
        alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_dijkstra(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    warthog::graph::xy_graph g;

    if(!g.load_from_dimacs(gr.c_str(), co.c_str()))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }

    warthog::simple_graph_expansion_policy expander(&g);
    warthog::zero_heuristic h;
    warthog::pqueue<warthog::cmp_less_search_node_f_only, warthog::min_q> open;

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::simple_graph_expansion_policy,
        warthog::pqueue<warthog::cmp_less_search_node_f_only, warthog::min_q>> 
            alg(&h, &expander, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bch(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; missing contraction order input file\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // load up the graph
    std::shared_ptr<warthog::graph::xy_graph> g(
            new warthog::graph::xy_graph());
    if(!g->load_from_dimacs( gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files "
            << "(one or both)\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::cerr << "preparing to search\n";
    warthog::bch_expansion_policy fexp(g.get(), &order);
    warthog::bch_expansion_policy bexp (g.get(), &order, true);
    warthog::zero_heuristic h;
    warthog::bch_search<
        warthog::zero_heuristic, 
        warthog::bch_expansion_policy> 
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bch_backwards_only(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; missing contraction order input file\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // load up the graph
    std::shared_ptr<warthog::graph::xy_graph> g(
            new warthog::graph::xy_graph());
    if(!g->load_from_dimacs( gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files "
            << "(one or both)\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::cerr << "preparing to search\n";
    warthog::bch_expansion_policy bexp (g.get(), &order, true);
    warthog::zero_heuristic h;
    warthog::pqueue_min open;

    warthog::flexible_astar<
        warthog::zero_heuristic, 
        warthog::bch_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &bexp, &open);

    std::cerr << "running experiments\n";
    std::cerr << "(averaging over " << nruns << " runs per instance)\n";

    if(!suppress_header)
    {
        std::cout 
            << "id\talg\texpanded\tinserted\tupdated\ttouched"
            << "\tnanos\tpcost\tplen\tmap\n";
    }
    uint32_t exp_id = 0;
    for(auto it = parser.experiments_begin(); 
            it != parser.experiments_end(); 
            it++)
    {
        warthog::dimacs_parser::experiment exp = (*it);
        warthog::solution sol;
        warthog::problem_instance pi(exp.source, warthog::INF, verbose);
        uint32_t expanded=0, inserted=0, updated=0, touched=0;
        double nano_time = DBL_MAX;
        for(uint32_t i = 0; i < nruns; i++)
        {
            sol.reset();
            alg.get_path(pi, sol);

            expanded += sol.nodes_expanded_;
            inserted += sol.nodes_inserted_;
            touched += sol.nodes_touched_;
            updated += sol.nodes_updated_;
            nano_time = nano_time < sol.time_elapsed_nano_ 
                            ?  nano_time : sol.time_elapsed_nano_;
        }

        std::cout
            << exp_id++ <<"\t" 
            << alg_name << "\t" 
            << expanded / nruns << "\t" 
            << inserted / nruns << "\t"
            << updated / nruns << "\t"
            << touched / nruns << "\t"
            << nano_time << "\t" /// (double)nruns << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << sol.path_.size() << "\t" 
            << parser.get_problemfile() 
            << std::endl;
    }
}

void
run_bch_astar(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; missing contraction order input file\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the graph 
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    std::cerr << "preparing to search\n";
    warthog::euclidean_heuristic h(&g);
    warthog::bch_expansion_policy fexp(&g, &order);
    warthog::bch_expansion_policy bexp (&g, &order, true);
    warthog::bch_search<
        warthog::euclidean_heuristic,
        warthog::bch_expansion_policy>
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_ch_cpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if(!orderfile.compare("") || !gridmapfile.compare(""))
    {
        std::cerr << "err; insufficient input params for alg " << alg_name 
                  << ". required (in, order) "
                  << "--input [ gr file ] [co file]"
                  << " [contraction order file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    if(!pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }

    // wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // expander
	warthog::zero_heuristic h;
	warthog::ch::ch_cpg_expansion_policy fexp(&cpg, &order);
	warthog::ch::ch_cpg_expansion_policy bexp(&cpg, &order, true);
    warthog::bch_search<
        warthog::zero_heuristic,
        warthog::ch::ch_cpg_expansion_policy>
        alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_chase(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return;
    }

    // read core percentage parameter
    double core_pct_value = 0.9;
    std::string str_core_pct = cfg.get_param_value("alg");
    if(str_core_pct != "")
    {
        alg_name.append(str_core_pct);
        int32_t tmp = atoi(str_core_pct.c_str());
        core_pct_value = (double)tmp / 100.0;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool sort_by_id = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, sort_by_id))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            new warthog::graph::xy_graph());
    if(!g->load_from_dimacs( gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files "
            << "(one or both)\n";
        return;
    }

    // load up the arc-flags; we divide the labels into two sets, one for the 
    // up graph and one for the down graph
    warthog::label::af_labelling *fwd_lab=0, *bwd_lab=0;
    bool result = warthog::label::af_labelling::load_bch_labels(
            arclabels_file.c_str(), g.get(), &part, &order, fwd_lab, bwd_lab);
    if(!result)
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::shared_ptr<warthog::label::af_labelling> fwd_afl(fwd_lab);
    std::shared_ptr<warthog::label::af_labelling> bwd_afl(bwd_lab);
    
    warthog::af_filter fwd_filter(fwd_afl.get());
    warthog::af_filter bwd_filter(bwd_afl.get());

    std::cerr << "preparing to search\n";
    warthog::zero_heuristic h;
    warthog::chase_expansion_policy fexp(g.get(), &fwd_filter);
    warthog::chase_expansion_policy bexp (g.get(), &bwd_filter, true);
    warthog::chase_search<warthog::zero_heuristic> 
        alg(&fexp, &bexp, &h, &order, core_pct_value);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bch_bb(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool lex_order = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, lex_order))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the edge labels
    warthog::label::bb_labelling *fwd_lab_ptr=0, *bwd_lab_ptr=0;
    bool result = warthog::label::bb_labelling::load_bch_labels(
            arclabels_file.c_str(), g.get(), &order, 
            fwd_lab_ptr, bwd_lab_ptr);
    if(!result)
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::shared_ptr<warthog::label::bb_labelling> fwd_lab(fwd_lab_ptr);
    std::shared_ptr<warthog::label::bb_labelling> bwd_lab(bwd_lab_ptr);

    warthog::bb_filter fwd_filter(fwd_lab.get());
    warthog::bb_filter bwd_filter(bwd_lab.get());

    std::cerr << "preparing to search\n";
    warthog::bch_bb_expansion_policy fexp(g.get(), &fwd_filter);
    warthog::bch_bb_expansion_policy bexp (g.get(), &bwd_filter, true);
    warthog::zero_heuristic h;
    warthog::bch_search<
        warthog::zero_heuristic,
        warthog::bch_bb_expansion_policy> 
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bch_af(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool lex_order = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, lex_order))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the arc-flags; we divide the labels into two sets, one for the 
    // up graph and one for the down graph
    warthog::label::af_labelling *fwd_lab=0, *bwd_lab=0;
    bool result = warthog::label::af_labelling::load_bch_labels(
            arclabels_file.c_str(), g.get(), &part, &order, fwd_lab, bwd_lab);
    if(!result)
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::shared_ptr<warthog::label::af_labelling> fwd_afl(fwd_lab);
    std::shared_ptr<warthog::label::af_labelling> bwd_afl(bwd_lab);
    
    warthog::af_filter fwd_filter(fwd_afl.get());
    warthog::af_filter bwd_filter(bwd_afl.get());

    std::cerr << "preparing to search\n";
    warthog::bch_af_expansion_policy fexp(g.get(), &fwd_filter);
    warthog::bch_af_expansion_policy bexp (g.get(), &bwd_filter, true);
    warthog::zero_heuristic h;
    warthog::bch_search<
        warthog::zero_heuristic,
        warthog::bch_af_expansion_policy>
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_bch_bbaf(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool lex_order = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, lex_order))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the arc-flags; we divide the labels into two sets, one for the 
    // up graph and one for the down graph
    warthog::label::bbaf_labelling *fwd_lab_ptr=0, *bwd_lab_ptr=0;
    bool result = warthog::label::bbaf_labelling::load_bch_labels(
            arclabels_file.c_str(), g.get(), &part, &order, 
            fwd_lab_ptr, bwd_lab_ptr);
    if(!result)
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::ch::optimise_graph_for_bch_v2(g.get(), &order);

    std::shared_ptr<warthog::label::bbaf_labelling> fwd_lab(fwd_lab_ptr);
    std::shared_ptr<warthog::label::bbaf_labelling> bwd_lab(bwd_lab_ptr);
    
    warthog::bbaf_filter fwd_filter(fwd_lab.get());
    warthog::bbaf_filter bwd_filter(bwd_lab.get());

    std::cerr << "preparing to search\n";
    warthog::zero_heuristic h;
    warthog::bch_bbaf_expansion_policy fexp(g.get(), &fwd_filter);
    warthog::bch_bbaf_expansion_policy bexp (g.get(), &bwd_filter, true);
    warthog::bch_search<
        warthog::zero_heuristic,
        warthog::bch_bbaf_expansion_policy> 
            alg(&fexp, &bexp, &h);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] \n";
        return;
    }

    // load up the graph 
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    std::cerr << "preparing to search\n";
    warthog::fch_expansion_policy fexp(&g, &order); 
    warthog::euclidean_heuristic h(&g);
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &fexp, &open);
    
    // extra metric; how many nodes do we expand above the apex?
    std::function<uint32_t(warthog::search_node*)> fn_get_apex = 
    [&order, &fexp] (warthog::search_node* n) -> uint32_t
    {
        while(true)
        {
            warthog::search_node* p = fexp.generate(n->get_parent());
            if(!p || order.at(p->get_id()) < order.at(n->get_id()))
            { break; }
            n = p;
        }
        return order.at(n->get_id());
    };

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_dfs(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string alg_params = cfg.get_param_value("alg");
    std::string orderfile = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] "
                  << " [graph partition file]\n";
        return;
    }

    // load up the graph 
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    std::cerr << "preparing to search\n";

    // sort the graph edges
    warthog::ch::fch_sort_successors(&g, &order);

    // define the workload
    double cutoff = 1;
    if(alg_params != "")
    {
        uint32_t pct_dijkstra = std::stoi(alg_params.c_str());
        if(!(pct_dijkstra >= 0 && pct_dijkstra <= 100))
        {
            std::cerr << "dijkstra percentage must be in range 0-100\n";
            return;
        }
        cutoff = pct_dijkstra > 0 ? (1 - ((double)pct_dijkstra)/100) : 0;
        alg_name += "-dijk-";
        alg_name += std::to_string(pct_dijkstra);
    }

    warthog::util::workload_manager workload(g.get_num_nodes());
    for(uint32_t i = 0; i < g.get_num_nodes(); i++)
    {
        if(order.at(i) >= (uint32_t)(order.size()*cutoff))
        { workload.set_flag(i, true); }
    }

    // load up the labelling
    std::string arclab_file =  gr + "." + alg_name + "." + "label";

    warthog::label::dfs_labelling* lab = warthog::label::dfs_labelling::load(
            arclab_file.c_str(), &g, &order, &part);

    if(lab == 0)
    {
        lab = warthog::label::dfs_labelling::compute(
                &g, &part, &order, &workload);
        std::cerr << "precompute finished. saving result to " 
            << arclab_file << "...";
        warthog::label::dfs_labelling::save(arclab_file.c_str(), *lab);
        std::cerr << "done.\n";
    }

    warthog::fch_dfs_expansion_policy fexp(&g, &order, lab, false);
    warthog::euclidean_heuristic h(&g);
    warthog::pqueue_min open;

    warthog::flexible_astar<
        warthog::euclidean_heuristic, 
        warthog::fch_dfs_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &fexp, &open);
    
    // extra metric; how many nodes do we expand above the apex?
    std::function<uint32_t(warthog::search_node*)> fn_get_apex = 
    [&order, &fexp] (warthog::search_node* n) -> uint32_t
    {
        while(true)
        {
            warthog::search_node* p = fexp.generate(n->get_parent());
            if(!p || order.at(p->get_id()) < order.at(n->get_id()))
            { break; }
            n = p;
        }
        return order.at(n->get_id());
    };

    run_experiments(&alg, alg_name, parser, std::cout);

    delete lab;
}

void
run_fch_fm(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string alg_params = cfg.get_param_value("alg");
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] "
                  << "\n";
        return;
    }

    // load up the graph 
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(gr.c_str(), co.c_str(), false, true))
    {
        std::cerr 
            << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    std::cerr << "preparing to search\n";

    // sort the graph edges
    warthog::ch::fch_sort_successors(&g, &order);

    // define the workload
    double cutoff = 1;
    if(alg_params != "")
    {
        uint32_t pct_dijkstra = std::stoi(alg_params.c_str());
        if(!(pct_dijkstra >= 0 && pct_dijkstra <= 100))
        {
            std::cerr << "dijkstra percentage must be in range 0-100\n";
            return;
        }
        cutoff = pct_dijkstra > 0 ? (1 - ((double)pct_dijkstra)/100) : 0;
        alg_name += "-dijk-";
        alg_name += std::to_string(pct_dijkstra);
    }

    warthog::util::workload_manager workload(g.get_num_nodes());
    for(uint32_t i = 0; i < g.get_num_nodes(); i++)
    {
        if(order.at(i) >= (uint32_t)(order.size()*cutoff))
        { workload.set_flag(i, true); }
    }

    // load up the labelling
    std::string arclab_file =  gr + "." + alg_name + "." + "label";

    warthog::label::firstmove_labelling* lab =
        warthog::label::firstmove_labelling::load(
            arclab_file.c_str(), &g, &order);

    if(lab == 0)
    {
        //std::cerr << "computing fch-firstmove labelling... \n";
        std::function<warthog::fch_expansion_policy*(void)> fn_new_expander = 
            [&g, &order]() -> warthog::fch_expansion_policy*
            {
                return new warthog::fch_expansion_policy(&g, &order);
            };
        lab = warthog::label::firstmove_labelling::compute
            <warthog::fch_expansion_policy>
                (&g, &order, fn_new_expander, &workload);
        std::cerr << "precompute finished. saving result to " 
            << arclab_file << "...";
        warthog::label::firstmove_labelling::save(arclab_file.c_str(), *lab);
        std::cerr << "done.\n";
    }

    warthog::fch_fm_expansion_policy fexp(&g, &order, lab, false);
    warthog::euclidean_heuristic h(&g);
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_fm_expansion_policy,
        warthog::pqueue_min>
            alg(&h, &fexp, &open);
    
    // extra metric; how many nodes do we expand above the apex?
    std::function<uint32_t(warthog::search_node*)> fn_get_apex = 
    [&order, &fexp] (warthog::search_node* n) -> uint32_t
    {
        while(true)
        {
            warthog::search_node* p = fexp.generate(n->get_parent());
            if(!p || order.at(p->get_id()) < order.at(n->get_id()))
            { break; }
            n = p;
        }
        return order.at(n->get_id());
    };

    run_experiments(&alg, alg_name, parser, std::cout);

    delete lab;
}

void
run_fch_apex_experiment(warthog::util::cfg& cfg, 
        warthog::dimacs_parser& parser, std::string alg_name, 
        std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    if(orderfile == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] \n";
        return;
    }

    // load up the graph 
    warthog::graph::xy_graph g;
    g.load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // load up the node order
    std::vector<uint32_t> order;
    warthog::ch::load_node_order(orderfile.c_str(), order);
    warthog::ch::value_index_swap_dimacs(order);

    std::cerr << "preparing to search\n";
    warthog::euclidean_heuristic h(&g);
    warthog::apex_filter filter(&order, &g);
    warthog::fch_x_expansion_policy fexp(&g, &order, &filter);

    std::string apex_filter_type = cfg.get_param_value("alg");
    if(apex_filter_type == "no_paa")
    {
        // don't prune up nodes after the apex is reached
        // (weaker filter)
        filter.prune_after_apex_ = false;
        alg_name = alg_name + "_" + apex_filter_type;
    }
    if(apex_filter_type == "no_pba")
    {
        // don't prune down nodes before the apex is reached
        // (weaker filter)
        filter.prune_before_apex_ = false;
        alg_name = alg_name + "_" + apex_filter_type;
    }

    // reference algo
    warthog::fch_expansion_policy fch_exp(&g, &order);
    warthog::pqueue_min open;

    warthog::flexible_astar<
        warthog::euclidean_heuristic, 
        warthog::fch_expansion_policy,
        warthog::pqueue_min> 
            fch(&h, &fch_exp, &open);

    // fchx
    warthog::pqueue_min open_fchx;
    warthog::flexible_astar<
        warthog::euclidean_heuristic, 
        warthog::fch_x_expansion_policy> 
            fchx(&h, &fexp, &open_fchx);

    std::cerr << "running fch apex experiment\n";
    if(!suppress_header)
    {
        std::cout 
            << "id\talg\texpanded\tinserted\tupdated\ttouched"
            << "\tnanos\tpcost\tplen\tmap\n";
    }
    uint32_t exp_id = 0;
    for(auto it = parser.experiments_begin(); 
            it != parser.experiments_end(); 
            it++)
    {
        warthog::dimacs_parser::experiment exp = (*it);
		warthog::solution sol;
        warthog::problem_instance pi(
                exp.source, (exp.p2p ? exp.target : warthog::INF), verbose);
        // run once
        fch.get_path(pi, sol);

        // identify the apex of the path
        if(sol.path_.size() > 0)
        {
            filter.set_apex(sol.path_.at(0));
            for(uint32_t i = 1; i < sol.path_.size(); i++)
            {
                if(order.at(filter.get_apex()) < order.at(sol.path_.at(i)))
                {
                    filter.set_apex(sol.path_.at(i));
                }
            }
        }

        // run the experiment again, using the apex to prune
        sol.reset();
        fchx.get_path(pi, sol);

        std::cout
            << exp_id++ <<"\t" 
            << alg_name << "\t" 
            << sol.nodes_expanded_ << "\t" 
            << sol.nodes_inserted_ << "\t"
            << sol.nodes_updated_ << "\t"
            << sol.nodes_touched_ << "\t"
            << sol.time_elapsed_nano_ << "\t"
            << sol.sum_of_edge_costs_ << "\t" 
            << sol.path_.size() << "\t" 
            << parser.get_problemfile() 
            << std::endl;
    }

}

void
run_fch_af(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool sort_by_id = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, sort_by_id))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the arc-flags
    std::shared_ptr<warthog::label::af_labelling> afl
        (warthog::label::af_labelling::load
        (arclabels_file.c_str(), g.get(), &part));
    if(!afl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    warthog::af_filter filter(afl.get());
    warthog::euclidean_heuristic h(g.get());
    warthog::fch_af_expansion_policy fexp(g.get(), &order, &filter);
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_af_expansion_policy,
        warthog::pqueue_min>
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_bb(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool lex_order = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, lex_order))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }
    // load up the arc labels
    std::shared_ptr<warthog::label::bb_labelling> bbl
        (warthog::label::bb_labelling::load(arclabels_file.c_str(), g.get()));
    if(!bbl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::bb_filter filter(bbl.get());

    warthog::euclidean_heuristic h(g.get());
    warthog::fch_bb_expansion_policy fexp(g.get(), &order, &filter);
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_bb_expansion_policy,
        warthog::pqueue_min>
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);

}

void
run_fch_bbaf(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || partition_file == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool sort_by_id = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, sort_by_id))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> g(
            warthog::ch::load_contraction_hierarchy_and_optimise_for_fch(
                gr.c_str(), co.c_str(), &order, false, true));
    if(!g.get())
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the arc-flags
    std::shared_ptr<warthog::label::bbaf_labelling> lab(
            warthog::label::bbaf_labelling::load(
                arclabels_file.c_str(), g.get(), &part));

    if(!lab.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    std::cerr << "preparing to search\n";
    warthog::euclidean_heuristic h(g.get());
    warthog::fch_bbaf_expansion_policy fexp(g.get(), &order, lab.get());
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_bbaf_expansion_policy,
        warthog::pqueue_min>
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_bbaf_cpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if( orderfile == "" || arclabels_file == "" || 
        partition_file == "" || gridmapfile == "" )
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < order.size(); i++) order.at(i)+=2;
    order.push_back(0);
    order.push_back(1);

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // graph wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the arc-flags
    std::shared_ptr<warthog::label::bbaf_labelling> lab(
            warthog::label::bbaf_labelling::load(
                arclabels_file.c_str(), pg.get(), &part));
    if(!lab.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    // search algo
    warthog::euclidean_heuristic h(pg.get());
    warthog::fch_bbaf_cpg_expansion_policy fexp(&cpg, &order, lab.get());
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_bbaf_cpg_expansion_policy, 
        warthog::pqueue_min>
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_cpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if(co == "" || gr == "" || orderfile == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input params for alg " << alg_name 
                  << ". required (in, order) "
                  << "--input [ gr file ] [co file]"
                  << " [contraction order file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // expander
    warthog::fch_cpg_expansion_policy fexp(&cpg, &order); 

    // heuristic 
    warthog::euclidean_heuristic h(pg.get());

    // open list
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_cpg_expansion_policy, 
        warthog::pqueue_min> 
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_jpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if(co == "" || gr == "" || orderfile == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input params for alg " << alg_name 
                  << ". required (in, order) "
                  << "--input [ gr file ] [co file]"
                  << " [contraction order file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // expander
    warthog::fch_jpg_expansion_policy fexp(&cpg, &order); 

    // heuristic 
    warthog::euclidean_heuristic h(pg.get());

    // open list
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_jpg_expansion_policy, 
        warthog::pqueue_min> 
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_af_cpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if( orderfile == "" || arclabels_file == "" || 
        partition_file == "" || gridmapfile == "" )
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < order.size(); i++) order.at(i)+=2;
    order.push_back(0);
    order.push_back(1);

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // graph wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the arc-flags
    std::shared_ptr<warthog::label::af_labelling> afl(
        warthog::label::af_labelling::load(
            arclabels_file.c_str(), pg.get(), &part));
    if(!afl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }
    warthog::af_filter filter(afl.get());

    // search algo
    warthog::euclidean_heuristic h(pg.get());
    warthog::fch_af_cpg_expansion_policy fexp(&cpg, &order, afl.get());
    warthog::pqueue_min open; 

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_af_cpg_expansion_policy,
        warthog::pqueue_min> alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_af_jpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string partition_file = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if( orderfile == "" || arclabels_file == "" || 
        partition_file == "" || gridmapfile == "" )
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file] "
                  << " [graph partition file] [gridmap file]\n";
        return;
    }
    
    // load up the node order
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load contraction order file\n";
        return;
    }

    // we insert two extra elements in the event that we
    // need to insert the start or target. both have the lowest
    // possible rank in the hierarchy (0 and 1)
    // NB: along the way we need to increase all ranks by 2 
    for(uint32_t i = 0; i < order.size(); i++) order.at(i)+=2;
    order.push_back(0);
    order.push_back(1);

    // load up the node partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(
            partition_file.c_str(), part))
    {
        std::cerr << "err; could not load graph partition input file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));
            
    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // graph wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the arc-flags
    std::shared_ptr<warthog::label::af_labelling> afl(
            warthog::label::af_labelling::load(
                arclabels_file.c_str(), pg.get(), &part));
    if(!afl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    // search algo
    warthog::euclidean_heuristic h(pg.get());
    warthog::fch_af_jpg_expansion_policy fexp(&cpg, &order, afl.get());
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_af_jpg_expansion_policy,
        warthog::pqueue_min> 
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_bb_cpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file]"
                  << " [gridmap file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool sort_by_id = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, sort_by_id))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // wrapper
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the arc-flags
    std::shared_ptr<warthog::label::bb_labelling> bbl(
            warthog::label::bb_labelling::load(
                arclabels_file.c_str(), pg.get()));
    if(!bbl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    // create a search algo
    warthog::euclidean_heuristic h(pg.get());
    warthog::fch_bb_cpg_expansion_policy fexp(&cpg, &order, bbl.get());
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_bb_cpg_expansion_policy,
        warthog::pqueue_min>
            alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_fch_bb_jpg(warthog::util::cfg& cfg, warthog::dimacs_parser& parser, 
        std::string alg_name, std::string gr, std::string co)
{
    std::string orderfile = cfg.get_param_value("input");
    std::string arclabels_file = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");
    if(orderfile == "" || arclabels_file == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input parameters for --alg "
                  << alg_name << ". required, in order:\n"
                  << " --input [gr file] [co file] "
                  << " [contraction order file] [arclabels file]"
                  << " [gridmap file]\n";
        return;
    }

    // load up the node order
    std::vector<uint32_t> order;
    bool sort_by_id = true;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, sort_by_id))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    pg->load_from_dimacs(gr.c_str(), co.c_str(), false, true);

    // wrapper
    warthog::graph::corner_point_graph jpg(map, pg);

    // load up the arc-flags
    std::shared_ptr<warthog::label::bb_labelling> bbl
        (warthog::label::bb_labelling::load(
            arclabels_file.c_str(), pg.get()));
    if(!bbl.get())
    {
        std::cerr << "err; could not load arcflags file\n";
        return;
    }

    // create a search algo
    warthog::euclidean_heuristic h(pg.get());
    warthog::fch_bb_jpg_expansion_policy fexp(&jpg, &order, bbl.get());
    warthog::pqueue_min open;

    warthog::flexible_astar< 
        warthog::euclidean_heuristic, 
        warthog::fch_bb_jpg_expansion_policy,
        warthog::pqueue_min> 
           alg(&h, &fexp, &open);

    run_experiments(&alg, alg_name, parser, std::cout);
}

void
run_dimacs(warthog::util::cfg& cfg)
{
    std::string gr = cfg.get_param_value("input");
    std::string co = cfg.get_param_value("input");
    std::string problemfile = cfg.get_param_value("problem");
    std::string alg_name = cfg.get_param_value("alg");
    std::string par_nruns = cfg.get_param_value("nruns");

    if(par_nruns != "")
    {
       char* end;
       nruns = strtol(par_nruns.c_str(), &end, 10);
    }


    if((problemfile == ""))
    {
        std::cerr << "parameter is missing: --problem\n";
        return;
    }
    if((gr == "") || co == "")
    {
        std::cerr << "parameter is missing: --input [gr file] [co file]\n";
        return;
    }
    if((alg_name == ""))
    {
        std::cerr << "parameter is missing: --alg\n";
        return;
    }

    warthog::dimacs_parser parser;
    parser.load_instance(problemfile.c_str());
    if(parser.num_experiments() == 0)
    {
        std::cerr << "err; specified problem file contains no instances\n";
        return;
    }

    if(alg_name == "dijkstra")
    {
        run_dijkstra(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "astar")
    {
        run_astar(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bi-dijkstra")
    {
        run_bi_dijkstra(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bi-astar")
    {
        run_bi_astar(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bch")
    {
        run_bch(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bchb")
    {
        run_bch_backwards_only(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "chase")
    {
        run_chase(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bch-astar")
    {
        run_bch_astar(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bch-bb")
    {
        run_bch_bb(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bch-af")
    {
        run_bch_af(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "bch-bbaf")
    {
        run_bch_bbaf(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "ch-cpg")
    {
        run_ch_cpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch")
    {
        run_fch(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fchx")
    {
        run_fch_apex_experiment(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-af")
    {
        run_fch_af(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-bb")
    {
        run_fch_bb(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-bbaf")
    {
        run_fch_bbaf(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-dfs")
    {
        run_fch_dfs(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-fm")
    {
        run_fch_fm(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-cpg")
    {
        run_fch_cpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-af-cpg")
    {
        run_fch_af_cpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-bb-cpg")
    {
        run_fch_bb_cpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-bbaf-cpg")
    {
        run_fch_bbaf_cpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-jpg")
    {
        run_fch_jpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-bb-jpg")
    {
        run_fch_bb_jpg(cfg, parser, alg_name, gr, co);
    }
    else if(alg_name == "fch-af-jpg")
    {
        run_fch_af_jpg(cfg, parser, alg_name, gr, co);
    }
    else
    {
        std::cerr << "invalid search algorithm\n";
    }
}


int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"alg",  required_argument, 0, 1},
		{"nruns",  required_argument, 0, 1},
		{"help", no_argument, &print_help, 1},
		{"checkopt",  no_argument, &checkopt, 1},
		{"verbose",  no_argument, &verbose, 1},
		{"noheader",  no_argument, &suppress_header, 1},
		{"input",  required_argument, 0, 1},
		{"problem",  required_argument, 0, 1},
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "-f", valid_args);

    if(argc == 1 || print_help)
    {
		help();
        exit(0);
    }

    run_dimacs(cfg);
}



