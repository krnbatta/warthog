#include "cfg.h"
#include "contraction.h"
#include "dimacs_parser.h"
#include <iostream>

void
help()
{
    std::cerr 
       << "Converts between the graph format used at the 9th DIMACS Implementation\n"
       << "Challenge and the graph format used by the METIS graph partitioning library.\n"
       << "Usage: ./dimacs2metis --input [dimacs .gr file] <optional params>\n"
       << "valid parameters:\n"
       << "\t--input <dimacs .gr file> \n"
       << "\t--core <integer in the range (0-100]> <node order> (optional)\n"
       << "\tconvert only the top k% of nodes in a hierarchy specified by "
       << "the given node order\n";
}

int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"core",  required_argument, 0, 2},
		{"input",  required_argument, 0, 2},
	};

    warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, "-hc:", valid_args);
    
    if(argc < 2)
    {
		help();
        exit(0);
    }

    std::string gr_file = cfg.get_param_value("input");
    if(gr_file == "")
    {
        std::cerr << "err; missing --input <gr file>\n";
        return EINVAL;
    }

    if(cfg.get_num_values("core") > 0)
    {
        // input is a contraction hierarchy and we only want to convert the
        // top k% of nodes (aka. the "core")
        if(cfg.get_num_values("core") != 2)
        {        
            help();
            return EINVAL;
        }

        std::string str_core_pct = cfg.get_param_value("core");
        std::string str_lex_order = cfg.get_param_value("core");

        double core_pct_value = 0.9;
        if(str_core_pct != "")
        {
            double tmp = atof(str_core_pct.c_str());
            core_pct_value = (double)tmp / 100.0;
            if(core_pct_value <= 0 || core_pct_value > 1)
            {
                std::cerr << "err; --core requires a parameter value in the "
                          << "range (0-100]\n";
                return EINVAL;
            }
        }

        // load up the node order
        std::vector<uint32_t> order;
        bool lex=true;
        if(!warthog::ch::load_node_order(str_lex_order.c_str(), order, lex))
        {
            std::cerr << "err; could not load node order input file\n";
            return EINVAL;
        }

        warthog::dimacs_parser parser;
        parser.load_graph(gr_file.c_str());
        parser.print_undirected_unweighted_metis(
                std::cout, core_pct_value, &order);

    }
    else
    {
        // input is a regular graph; convert the entire thing
        warthog::dimacs_parser parser;
        parser.load_graph(gr_file.c_str());
        parser.print_undirected_unweighted_metis(std::cout);
    }

    return 0;
}

