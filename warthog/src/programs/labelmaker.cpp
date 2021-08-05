#include "af_labelling.h"
#include "bb_labelling.h"
#include "bbaf_labelling.h"
#include "dfs_labelling.h"
#include "firstmove_labelling.h"
#include "cfg.h"
#include "contraction.h"
#include "corner_point_graph.h"
#include "dimacs_parser.h"
#include "dummy_filter.h"
#include "fch_expansion_policy.h"
#include "fch_jpg_expansion_policy.h"
#include "graph_expansion_policy.h"
#include "graph.h"
#include "gridmap.h"
#include "helpers.h"
#include "xy_graph.h"

#include <iostream>
#include <memory>
#include <string>

int verbose=false;
warthog::util::cfg cfg;

void
help()
{
    std::cerr << 
        "create arc labels for " <<
        "a given (currently, DIMACS-format only) input graph\n";
	std::cerr << "valid parameters:\n"
    //<< "\t--dimacs [gr file] [co file] (IN THIS ORDER!!)\n"
	//<< "\t--order [order-of-contraction file]\n"
    << "\t--type [ af | bb | bbaf | fm | fch-af | fch-bb "
    << "| fch-bbaf | fch-bb-jpg | fch-dfs | fch-fm ] \n"
    << "\t--input [ algorithm-specific input files (omit to show options) ]\n" 
	<< "\t--verbose (optional)\n";
}

void
compute_fch_bb_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || orderfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node ordering (with ids listed in order of contraction)
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr 
            << "warn; partial contraction order. computing labels only"
            << "for contracted nodes\n";
    }

    // define the preprocessing workload 
    warthog::util::workload_manager workload(g.get_num_nodes());

    // users can limit the preprocessing to a specified id range
    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the range which also appear in the order
    for(uint32_t i = 0; i < order.size(); i++)
    {
        if(order.at(i) < firstid || order.at(i) >= lastid) { continue; }
        workload.set_flag(order.at(i), true);
    }

    // node order becomes a lex order for the purposes of search
    warthog::ch::value_index_swap_dimacs(order);

    // FCH assumes the successors list is sorted 
    warthog::ch::fch_sort_successors(&g, &order);

    // gogogo
    std::cerr << "computing fch-bb labelling... \n";
    std::function<warthog::fch_expansion_policy*(void)> fn_new_expander = 
        [&g, &order]() -> warthog::fch_expansion_policy*
        {
            return new warthog::fch_expansion_policy(&g, &order);
        };
    warthog::label::bb_labelling* labelling = 
        warthog::label::bb_labelling::compute<warthog::fch_expansion_policy>(
                &g, fn_new_expander, &workload);

    // save the result
    std::string outfile(grfile);
    outfile.append(".fch-bb.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "\ndone; \nsaving to " << outfile << "\n";
    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }
    fs_out.close();

    delete labelling;
    std::cerr << "done!\n";
}

void
compute_fch_bb_jpg_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || orderfile == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file] [gridmap]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));

    // load up the graph 
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    if(!pg->load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != cpg.get_num_nodes())
    {
        std::cerr 
            << "warn; partial contraction order. computing labels only"
            << "for contracted nodes\n";
    }

    // define the preprocessing workload 
    warthog::util::workload_manager workload(cpg.get_num_nodes());

    // users can limit the preprocessing to a specified id range
    uint32_t firstid = 0;
    uint32_t lastid = cpg.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the range which also appear in the order
    for(uint32_t i = 0; i < order.size(); i++)
    {
        if(order.at(i) < firstid || order.at(i) >= lastid) { continue; }
        workload.set_flag(order.at(i), true);
    }

    // node order becomes a lex order for the purposes of search
    warthog::ch::value_index_swap_dimacs(order);
    
    // gogogo
    std::cerr << "creating fch-jpg-bb labelling\n";
    std::function<warthog::fch_jpg_expansion_policy*(void)> 
        fn_new_expander = [&cpg, &order] () 
            -> warthog::fch_jpg_expansion_policy*
        {
            return new warthog::fch_jpg_expansion_policy(&cpg, &order);
        };

    warthog::label::bb_labelling* labelling = 
    warthog::label::bb_labelling::compute<warthog::fch_jpg_expansion_policy>(
            pg.get(), fn_new_expander, &workload);

    std::cerr << "labelling done\n";

    // save the result
    std::string outfile(grfile);
    outfile.append(".fch-bb-jpg.arclabel");
    if(firstid != 0 || lastid != cpg.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);

    std::cerr << "saving to" << outfile << "\n";
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }

    fs_out.close();
    delete labelling;
    std::cerr << "all done!\n";
}

void
compute_bb_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    if(grfile == "" || cofile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]\n";
        return;
    }

    // load up (or create) the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // define the preprocessing workload size
    warthog::util::workload_manager workload(g.get_num_nodes());

    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }
        
    // we only target nodes in the specified id range
    for(uint32_t i = firstid; i < lastid; i++)
    {
        workload.set_flag(i, true);
    }

    warthog::dummy_filter filter;

    // gogogo
    std::cerr << "computing bounding box labelling... \n";
    std::function<
        warthog::graph_expansion_policy<warthog::dummy_filter>* (void) >
        fn_new_expander = 
        [&g, &filter]() -> 
        warthog::graph_expansion_policy<warthog::dummy_filter>*
        {
            return new warthog::graph_expansion_policy<warthog::dummy_filter>
                (&g, &filter);
        };
    warthog::label::bb_labelling* labelling = 
        warthog::label::bb_labelling::compute
            <warthog::graph_expansion_policy<warthog::dummy_filter>>
                (&g, fn_new_expander, &workload);

    // save the result
    std::string outfile(grfile);
    outfile.append(".bb.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "\ndone; \nsaving to " << outfile << "\n";
    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }
    fs_out.close();

    delete labelling;
    std::cerr << "done!\n";
}

void
compute_fch_af_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "" || orderfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]"
                  << " [graph partition file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true);

    // load up the partition info
    std::vector<uint32_t> part;
    warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part);

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr 
            << "warn; partial contraction order. computing labels only"
            << "for contracted nodes\n";
    }

    // define the preprocessing workload size
    warthog::util::workload_manager workload(g.get_num_nodes());

    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the range which also appear in the order
    for(uint32_t i = 0; i < order.size(); i++)
    {
        if(order.at(i) < firstid || order.at(i) >= lastid) { continue; }
        workload.set_flag(order.at(i), true);
    }

    // node order becomes a lex order for the purposes of search
    warthog::ch::value_index_swap_dimacs(order);

    // FCH assumes the successors list is sorted 
    warthog::ch::fch_sort_successors(&g, &order);

    // gogogo
    std::cerr << "computing fch-af labelling... \n";
    std::function<warthog::fch_expansion_policy*(void)> fn_new_expander = 
        [&g, &order]() -> warthog::fch_expansion_policy*
        {
            return new warthog::fch_expansion_policy(&g, &order);
        };
    warthog::label::af_labelling* labelling = 
        warthog::label::af_labelling::compute
            <warthog::fch_expansion_policy>
                (&g, &part, fn_new_expander, &workload);

    // save the result
    std::string outfile(grfile);
    outfile.append(".fch-af.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "\ndone; \nsaving to " << outfile << "\n";
    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }
    fs_out.close();

    delete labelling;
    std::cerr << "done!\n";
}

void
compute_fch_af_jpg_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");
    std::string gridmapfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "" ||
            orderfile == "" || gridmapfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]"
                  << " [graph partition file]"
                  << " [gridmap]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the grid
    std::shared_ptr<warthog::gridmap> map(
            new warthog::gridmap(gridmapfile.c_str()));

    // load up the graph
    std::shared_ptr<warthog::graph::xy_graph> pg(
            new warthog::graph::xy_graph());
    if(!pg->load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files " 
                  << "(one or both)\n";
        return;
    }
    warthog::graph::corner_point_graph cpg(map, pg);

    // load up the partition info
    std::vector<uint32_t> part;
    warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part);

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != cpg.get_num_nodes())
    {
        std::cerr 
            << "warn; partial contraction order. computing labels only"
            << "for contracted nodes\n";
    }

    // define the preprocessing workload 
    warthog::util::workload_manager workload(cpg.get_num_nodes());

    // users can limit the preprocessing to a specified id range
    uint32_t firstid = 0;
    uint32_t lastid = pg->get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the range which also appear in the order
    for(uint32_t i = 0; i < order.size(); i++)
    {
        if(order.at(i) < firstid || order.at(i) >= lastid) { continue; }
        workload.set_flag(order.at(i), true);
    }

    // node order becomes a lex order for the purposes of search
    warthog::ch::value_index_swap_dimacs(order);

    // gogogo
    std::cerr << "computing fch-af labelling... \n";
    std::function<warthog::fch_jpg_expansion_policy*(void)> fn_new_expander = 
        [&cpg, &order]() -> warthog::fch_jpg_expansion_policy*
        {
            return new warthog::fch_jpg_expansion_policy(&cpg, &order);
        };
    warthog::label::af_labelling* labelling = 
        warthog::label::af_labelling::compute
            <warthog::fch_jpg_expansion_policy>
                (pg.get(), &part, fn_new_expander, &workload);

    // save the result
    std::string outfile(grfile);
    outfile.append(".fch-af-jpg.arclabel");
    if(firstid != 0 || lastid != cpg.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "saving arclabels file " << outfile << std::endl;
    std::fstream out(outfile.c_str(), 
            std::ios_base::out | std::ios_base::trunc);
    if(!out.good())
    {
        std::cerr << "\nerror saving arclabels file " << grfile << std::endl;
    }
    else
    {
        labelling->print(out, firstid, lastid);
    }
    out.close();

    // cleanup
    delete labelling;
    std::cerr << "all done!\n";
    
}

void
compute_af_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [graph partition file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up (or create) the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part))
    {
        std::cerr << "err; could not load partition file\n"; 
        return;
    }
    
    // define the preprocessing workload size
    warthog::util::workload_manager workload(g.get_num_nodes());

    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the specified id range
    for(uint32_t i = firstid; i < lastid; i++)
    {
        workload.set_flag(i, true);
    }

    warthog::dummy_filter filter;

    // gogogo
    std::cerr << "computing af labelling... \n";
    std::function<warthog::graph_expansion_policy<warthog::dummy_filter>*
        (void)> fn_new_expander = 
        [&g, &filter]() -> 
        warthog::graph_expansion_policy<warthog::dummy_filter>*
        {
            return new warthog::graph_expansion_policy<warthog::dummy_filter>
                (&g, &filter);
        };
    warthog::label::af_labelling* labelling = 
        warthog::label::af_labelling::compute
            <warthog::graph_expansion_policy<warthog::dummy_filter>>
                (&g, &part, fn_new_expander, &workload);

    // save the result
    std::string outfile(grfile);
    outfile.append(".af.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "saving arclabels file " << outfile << std::endl;
    std::fstream out(outfile.c_str(), 
            std::ios_base::out | std::ios_base::trunc);
    if(!out.good())
    {
        std::cerr << "\nerror saving arclabels file " << grfile << std::endl;
    }
    else
    {
        labelling->print(out, firstid, lastid);
    }
    out.close();

    delete labelling;
    std::cerr << "all done!\n";
}

void
compute_bbaf_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [graph partition file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part))
    {
        std::cerr << "err; could not load partition file\n"; 
        return;
    }

    // define the preprocessing workload 
    warthog::util::workload_manager workload(g.get_num_nodes());

    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the specified id range
    for(uint32_t i = firstid; i < lastid; i++)
    {
        workload.set_flag(i, true);
    }

    // compute bbaf labels
    
    warthog::dummy_filter filter;
    std::function< 
        warthog::graph_expansion_policy<warthog::dummy_filter>* 
        (void)> fn_new_expander = 
        [&g, &filter]() -> 
        warthog::graph_expansion_policy<warthog::dummy_filter>*
        {
            return new warthog::graph_expansion_policy<warthog::dummy_filter>
                (&g, &filter);
        };

    std::shared_ptr<warthog::label::bbaf_labelling> 
        labelling(
            warthog::label::bbaf_labelling::compute
                <warthog::graph_expansion_policy<warthog::dummy_filter>>
                    (&g, &part, fn_new_expander, &workload));

    // save the result
    std::string outfile(grfile);
    outfile.append(".bbaf.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "\ndone; \nsaving to " << outfile << "\n";
    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }
    fs_out.close();
    std::cerr << "done!\n";
}

void
compute_fch_bbaf_labels()
{
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "" || orderfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]"
                  << " [graph partition file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part))
    {
        std::cerr << "err; could not load partition file\n"; 
        return;
    }

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, false))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr 
            << "warn; partial contraction order. computing labels only"
            << "for contracted nodes\n";
    }

    // define the preprocessing workload size
    warthog::util::workload_manager workload(g.get_num_nodes());

    uint32_t firstid = 0;
    uint32_t lastid = g.get_num_nodes();
    if(cfg.get_num_values("type") == 2)
    {
        std::string first = cfg.get_param_value("type");
        std::string last = cfg.get_param_value("type");
        if(strtol(first.c_str(), 0, 10) != 0)
        {
            firstid = strtol(first.c_str(), 0, 10);
        }
        if(strtol(last.c_str(), 0, 10) != 0)
        {
            lastid = strtol(last.c_str(), 0, 10);
        }
    }

    // we only target nodes in the range which also appear in the order
    for(uint32_t i = 0; i < order.size(); i++)
    {
        if(order.at(i) < firstid || order.at(i) >= lastid) { continue; }
        workload.set_flag(order.at(i), true);
    }

    // node order becomes a lex order for the purposes of search
    warthog::ch::value_index_swap_dimacs(order);

    // FCH assumes the successors list is sorted 
    warthog::ch::fch_sort_successors(&g, &order);

    // gogogo
    std::function<warthog::fch_expansion_policy*(void)> fn_new_expander = 
    [&g, &order]() -> warthog::fch_expansion_policy*
    {
        return new warthog::fch_expansion_policy(&g, &order);
    };

    std::shared_ptr<warthog::label::bbaf_labelling> 
        labelling(
            warthog::label::bbaf_labelling::compute
                <warthog::fch_expansion_policy>
                    (&g, &part, fn_new_expander, &workload));
    
    // save the result
    std::string outfile(grfile);
    outfile.append(".fch-bbaf.arclabel");
    if(firstid != 0 || lastid != g.get_num_nodes())
    {
        outfile.append(".");
        outfile.append(std::to_string(firstid));
        outfile.append(".");
        outfile.append(std::to_string(lastid));
    }

    std::cerr << "\ndone; \nsaving to " << outfile << "\n";
    std::fstream fs_out(outfile.c_str(), 
                        std::ios_base::out | std::ios_base::trunc);
    if(!fs_out.good())
    {
        std::cerr << "\nerror opening output file " << outfile << std::endl;
    }
    else
    {
        labelling->print(fs_out, firstid, lastid);
    }
    fs_out.close();
    std::cerr << "done!\n";
}

void
compute_fch_dfs_labels(std::string alg_name)
{
    std::string alg_params = cfg.get_param_value("type");
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");
    std::string partfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || partfile == "" || orderfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]"
                  << " [graph partition file]\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr 
            << "err; partial contraction ordering not supported\n";
    }

    // sort the edges of each node according to the order (descending)
    warthog::ch::fch_sort_successors(&g, &order);

    // load up the partition info
    std::vector<uint32_t> part;
    if(!warthog::helpers::load_integer_labels_dimacs(partfile.c_str(), part))
    {
        std::cerr << "err; could not load partition file\n"; 
        return;
    }

    // define the preprocessing workload size
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

    // gogogogo
    std::shared_ptr<warthog::label::dfs_labelling> lab(
            warthog::label::dfs_labelling::compute(&g, &part, &order, 
                &workload));

    std::string arclab_file =  grfile + "." + alg_name + "." + "label";
    warthog::label::dfs_labelling::save(arclab_file.c_str(), *lab);
    std::cerr << "done.\n";
}

void
compute_fch_fm_labels(std::string alg_name)
{
    std::string alg_params = cfg.get_param_value("type");
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");
    std::string orderfile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "" || orderfile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << " [node ordering file]"
                  << "\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // load up the node ordering
    std::vector<uint32_t> order;
    if(!warthog::ch::load_node_order(orderfile.c_str(), order, true))
    {
        std::cerr << "err; could not load node order input file\n";
        return;
    }

    if(order.size() != g.get_num_nodes())
    {
        std::cerr 
            << "err; partial contraction ordering not supported\n";
    }

    // sort the edges of each node according to the order (descending)
    warthog::ch::fch_sort_successors(&g, &order);

    // define the preprocessing workload size
    double cutoff = 1;
    if(alg_params != "")
    {
        std::cerr << "wtf params " << alg_params << std::endl;
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

    std::cerr << "computing fch-firstmove labelling... \n";
    std::function<warthog::fch_expansion_policy*(void)> fn_new_expander = 
        [&g, &order]() -> warthog::fch_expansion_policy*
        {
            return new warthog::fch_expansion_policy(&g, &order);
        };

    // now we need to specify a column order to use during run-length 
    // compression
    std::vector<uint32_t> column_order;
    //warthog::label::dfs_labelling::compute_dfs_ids(&g, &order, &column_order);
    //warthog::label::compute_fm_dfs_preorder(g, column_order);
    warthog::label::compute_fm_fch_dfs_preorder(g, order, column_order);
    //warthog::label::compute_fm_fch_dijkstra_dfs_preorder(g, order, column_order);
    
    // gogogogo
    std::shared_ptr<warthog::label::firstmove_labelling> lab(
            warthog::label::firstmove_labelling::compute
                <warthog::fch_expansion_policy> 
                    (&g, &column_order, fn_new_expander, &workload));

    std::string arclab_file =  grfile + "." + alg_name + "." + "label";
    warthog::label::firstmove_labelling::save(arclab_file.c_str(), *lab);
    std::cerr << "done.\n";
}

void
compute_fm_labels(std::string alg_name)
{
    std::string alg_params = cfg.get_param_value("type");
    std::string grfile = cfg.get_param_value("input");
    std::string cofile = cfg.get_param_value("input");

    if( grfile == "" || cofile == "")
    {
        std::cerr << "err; insufficient input parameters."
                  << " required, in order:\n"
                  << " --input [gr file] [co file]"
                  << "\n";
        return;
    }
    std::cerr << "computing labels" << std::endl;

    // load up the graph
    warthog::graph::xy_graph g;
    if(!g.load_from_dimacs(grfile.c_str(), cofile.c_str(), false, true))
    {
        std::cerr << "err; could not load gr or co input files (one or both)\n";
        return;
    }

    // define the preprocessing workload size
    int32_t pct_dijkstra = 100;
    if(alg_params != "") { pct_dijkstra = std::stoi(alg_params.c_str()); } 

    if(!(pct_dijkstra >= 0 && pct_dijkstra <= 100))
    {
        std::cerr << "dijkstra percentage must be in range 0-100\n";
        return;
    }
    alg_name += "-dijk-";
    alg_name += std::to_string(pct_dijkstra);

    warthog::util::workload_manager workload(g.get_num_nodes());
    for(uint32_t i = 0; i < g.get_num_nodes(); i++)
    {
        if(rand() % 100 <= pct_dijkstra)
        { workload.set_flag(i, true); }
    }

    // now we need to specify a column order to use during run-length 
    // compression; here we use a dfs ordering from a random seed node
    std::vector<uint32_t> column_order;
    warthog::label::compute_fm_dfs_preorder(g, column_order);

    std::cerr << "computing firstmove labelling... \n";
    std::function<warthog::graph_expansion_policy<warthog::dummy_filter>*(void)> fn_new_expander = 
        [&g]() -> warthog::graph_expansion_policy<warthog::dummy_filter>*
        {
            return new warthog::graph_expansion_policy<warthog::dummy_filter>(&g);
        };
    
    // gogogogo
    std::shared_ptr<warthog::label::firstmove_labelling> lab(
            warthog::label::firstmove_labelling::compute
                <warthog::graph_expansion_policy<warthog::dummy_filter>> 
                    (&g, &column_order, fn_new_expander, &workload));

    std::string arclab_file =  grfile + "." + alg_name + "." + "label";
    warthog::label::firstmove_labelling::save(arclab_file.c_str(), *lab);
    std::cerr << "done.\n";
}

int main(int argc, char** argv)
{

	// parse arguments
    int print_help=false;
	warthog::util::param valid_args[] = 
	{
		{"help", no_argument, &print_help, 1},
		{"verbose", no_argument, &verbose, 1},
		{"input",  required_argument, 0, 2},
        {"type", required_argument, 0, 1}
	};
	cfg.parse_args(argc, argv, "-hvd:o:p:a:", valid_args);

    if(argc == 1 || print_help)
    {
		help();
        return EINVAL;
    }

    // parse the type of labelling and the source nodes to be processed
    // source nodes are in the range: [first_id, last_id)
    std::string arclabel = cfg.get_param_value("type");
    if(arclabel.compare("fch-bb") == 0)
    {
        compute_fch_bb_labels();
    }
    else if(arclabel.compare("bb") == 0)
    {
        compute_bb_labels();
    }
    else if(arclabel.compare("fch-af") == 0)
    {
        compute_fch_af_labels();
    }
    else if(arclabel.compare("af") == 0)
    {
        compute_af_labels();
    }
    else if(arclabel.compare("bbaf") == 0)
    {
        compute_bbaf_labels();
    }
    else if(arclabel.compare("fch-bbaf") == 0)
    {
        compute_fch_bbaf_labels();
    }
    else if(arclabel.compare("fch-bb-jpg") == 0)
    {
        compute_fch_bb_jpg_labels();
    }
    else if(arclabel.compare("fch-af-jpg") == 0)
    {
        compute_fch_af_jpg_labels();
    }
    else if(arclabel.compare("fch-dfs") == 0)
    {
        compute_fch_dfs_labels(arclabel);
    }
    else if(arclabel.compare("fch-fm") == 0)
    {
        compute_fch_fm_labels(arclabel);
    }
    else if(arclabel.compare("fm") == 0)
    {
        compute_fm_labels(arclabel);
    }
    else
    {
        std::cerr << "invalid or missing argument: --type\n";
    }
    return 0;
}
