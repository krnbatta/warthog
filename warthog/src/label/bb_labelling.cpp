#include "bb_labelling.h"

#include <algorithm>
#include <fstream>

warthog::label::bb_labelling::bb_labelling(warthog::graph::xy_graph* g)
{
    g_ = g;

    // allocate memory for every label
    labels_ = new std::vector<std::vector<warthog::geom::rectangle>>();
    labels_->resize(g->get_num_nodes());
}

warthog::label::bb_labelling::~bb_labelling()
{
    delete labels_;
}

void
warthog::label::bb_labelling::print(
        std::ostream& out, uint32_t first_id, uint32_t last_id)
{
    if(first_id > labels_->size()) { return; }
    if(last_id > labels_->size()) { last_id = labels_->size(); }

    out << "# Before printing labels are sorted by tail index. \n"
        << "# Ties are broken using the order edges appear in the file \n"
        << "# " << g_->get_filename() << std::endl
        << "# (lower is better)\n";

    warthog::geom::rectangle dummy;
    for(uint32_t i = first_id; i < last_id; i++)
    {
        std::vector<warthog::geom::rectangle> vrect = labels_->at(i);
        for(uint32_t j = 0; j < vrect.size(); j++)
        {
            warthog::geom::rectangle rect = vrect.at(j);
            if(rect != dummy) { assert(rect.is_valid()); }
            out 
                //<< i << " " << j << " " << 
                << rect.x1 << "\t" << rect.y1 << "\t"
                << rect.x2 << "\t" << rect.y2 << "\t"
                << std::endl;
        }
    }
}

warthog::label::bb_labelling*
warthog::label::bb_labelling::load(
        const char* filename, warthog::graph::xy_graph* g)
{
    std::ifstream ifs(filename, std::ios_base::in);
    if(!ifs.good())
    {
        std::cerr << "\nerror trying to load bbox values from file " 
            << filename << std::endl;
        ifs.close();
        return 0;
    }

    uint32_t lines = 1;
    // skip comment lines
    while(ifs.peek() == '#')
    {
        while(ifs.good() && ifs.get() != '\n');
        lines++;
    }

    warthog::label::bb_labelling* bbl = new warthog::label::bb_labelling(g);
    warthog::geom::rectangle dummy;
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            int32_t x1, y1, x2, y2;
            ifs >> x1 >> y1 >> x2 >> y2;
            if(ifs.eof() || !ifs.good())
            {
                std::cerr << "unexpected error while reading" 
                    << filename << "; line=" << lines << "\n";
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n";
                delete bbl; 
                return 0;
            }

            warthog::geom::rectangle rect = 
                warthog::geom::rectangle(x1, y1, x2, y2);
            if(rect != dummy && !rect.is_valid())
            {
                std::cerr << "err; invalid label on line " << lines <<"\n";
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n";
                delete bbl;
                return 0;
            }
            bbl->labels_->at(i).push_back(rect);
            lines++;
        }
    }
    ifs.close();
    return bbl;
}

bool
warthog::label::bb_labelling::load_bch_labels(
        const char* filename, warthog::graph::xy_graph* g,
        std::vector<uint32_t>* lex_order, 
        warthog::label::bb_labelling*& out_lab_fwd,
        warthog::label::bb_labelling*& out_lab_bwd)
{
    std::cerr << "loading arcflags labelling file\n";
    std::ifstream ifs(filename, std::ios_base::in);
    if(!ifs.good())
    {
        std::cerr << "\nerror trying to load arcflags file " 
            << filename << std::endl;
        ifs.close();
        return false;
    }

    // skip comment lines
    uint32_t lines = 1;
    while(ifs.peek() == '#')
    {
        while(ifs.get() != '\n');
        lines++;
    }

    out_lab_fwd = new warthog::label::bb_labelling(g);
    out_lab_bwd = new warthog::label::bb_labelling(g);

    warthog::geom::rectangle dummy;
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);

        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            int32_t x1, y1, x2, y2;
            ifs >> x1 >> y1 >> x2 >> y2;
            if(ifs.eof() || !ifs.good())
            {
                std::cerr << "unexpected error while reading" 
                    << filename << "; line=" << lines << "\n";
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n";
                delete out_lab_fwd;
                delete out_lab_bwd;
                return false;
            }

            warthog::geom::rectangle rect = 
                warthog::geom::rectangle(x1, y1, x2, y2);
            if(rect != dummy && !rect.is_valid())
            {
                std::cerr << "err; invalid label on line " << lines <<"\n";
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n";
                delete out_lab_fwd;
                delete out_lab_bwd;
                return false;
            }

            // store the label
            warthog::graph::edge* e = n->outgoing_begin() + j;

            // every down arc becomes reversed and stored in the bwd direction
            if(lex_order->at(e->node_id_) < lex_order->at(i)) 
            { 
                out_lab_bwd->labels_->at(e->node_id_).push_back(rect);
            }
            // every up arc is stored in the fwd direction
            else
            {
                out_lab_fwd->labels_->at(i).push_back(rect);
            }

            lines++;
        }
    }
    return true;
}
