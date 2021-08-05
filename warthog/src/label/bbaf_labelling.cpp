#include "bbaf_labelling.h"

#include <fstream>
#include <pthread.h>
warthog::label::bbaf_labelling::bbaf_labelling(
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* partitioning)
{
    g_ = g;
    part_ = partitioning;

    // figure out how many bytes are required per label
    uint32_t max_id = *(std::max_element(part_->begin(), part_->end()));
    bytes_per_af_label_ = (max_id / 8) + !!(max_id % 8);
    
    // allocate memory for every node
    labels_.resize(g_->get_num_nodes());
}

warthog::label::bbaf_labelling::~bbaf_labelling()
{
    for(uint32_t i = 0; i < labels_.size(); i++)
    {
        std::vector<bbaf_label>& arclabs = labels_.at(i);
        for(uint32_t j = 0; j < arclabs.size(); j++)
        {
            bbaf_label& label = arclabs.back();
            delete [] label.flags_;
            arclabs.pop_back();
        }
    }
}

void
warthog::label::bbaf_labelling::print(std::ostream& out, 
        uint32_t first_id, uint32_t last_id)
{
    out 
    << "# Each line (aside from comments & header) is an edge label.\n"
    << "# A label comprises a set of arcflags (written out as 64bit words) \n"
    << "# and a rectangular bounding box which contains all nodes in the \n"
    << "# down-closure of the associated edge. The last 4 digits of each\n"
    << "# line describe the bounding box: minx, miny, maxx, maxy.\n"
    << "#\n"
    << "# NB: Before printing, labels are sorted by edge-tail index. \n"
    << "# Ties are broken using the order edges appear in the file \n"
    << "# " << g_->get_filename() << std::endl
    << "# (lower/earlier is better)\n";

    // we split labels into 64bit words for printing
    const uint32_t word_sz = sizeof(uint64_t);
    uint32_t words_per_label = bytes_per_af_label_ / word_sz;
    if((bytes_per_af_label_ % word_sz) > 0) words_per_label++;

    // iterate over the labels for each outgoing arc of each node
    warthog::geom::rectangle dummy;
    last_id = last_id > g_->get_num_nodes() ? 
        g_->get_num_nodes() : last_id;
    for(uint32_t i = first_id; i < last_id; i++)
    {
        std::vector<bbaf_label>& arclabs = labels_.at(i);
        for(uint32_t j = 0; j < arclabs.size(); j++)
        {
            // print the arcflags
            bbaf_label& label = arclabs.at(j);
            for(uint32_t word = 0; word < words_per_label; word++)
            {
                uint8_t printed_word[word_sz];
                for(uint32_t k = 0; k < word_sz; k++)
                {
                    // read the flag label, byte by byte, one word at a time
                    if((word*word_sz+k) < bytes_per_af_label_)
                    {
                        printed_word[k] = label.flags_[word*word_sz+k];
                    }
                    // pad the last word with leading zeroes if necessary
                    else
                    {
                        printed_word[k] = 0;
                    }
                }
                out << *((uint64_t*)&(printed_word)) << " ";
            }

            // print the bounding box
            assert(label.bbox_ == dummy || label.bbox_.is_valid());
            out << label.bbox_.x1 << " " << label.bbox_.y1 << " " 
                << label.bbox_.x2 << " " << label.bbox_.y2;
            out << std::endl;
        }
    }
}

warthog::label::bbaf_labelling*
warthog::label::bbaf_labelling::load(const char* filename, 
        warthog::graph::xy_graph* g, 
        std::vector<uint32_t>* partitioning)
{
    std::cerr << "loading bbaf file\n";
    std::ifstream ifs(filename);

    if(!ifs.good())
    {
        std::cerr << "\nerror trying to load bbaf labelling file " 
            << filename << std::endl;
        ifs.close();
        return 0;
    }

    warthog::label::bbaf_labelling* lab = 
        new warthog::label::bbaf_labelling(g, partitioning);

    uint32_t lines = 1;

    // skip comment lines
    while(ifs.peek() == '#')
    {
        while(ifs.get() != '\n');
        lines++;
    }

    // read labels for each outgoing arc
    std::string token;
    const uint32_t word_sz = sizeof(uint64_t);
    uint32_t words_per_label = (lab->bytes_per_af_label_ / word_sz);
    if(lab->bytes_per_af_label_ % word_sz != 0) { words_per_label++; }
    for(uint32_t i = 0; i < lab->g_->get_num_nodes(); i++)
    {
        warthog::graph::node* n = lab->g_->get_node(i);
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {
            // create one label per outgoing edge
            bbaf_label label;
            label.flags_= new uint8_t[words_per_label*word_sz];

            for(uint32_t word_idx = 0; word_idx< words_per_label; word_idx++)
            {
                if(!ifs.good())
                {
                    std::cerr 
                        << "error; invalid bbaf label on line "
                        << lines << " (af part); aborting\n";
                    std::cerr 
                        << "[debug info] node: " << i 
                        << " out-edge-index: " << j << "\n";
                    delete lab;
                    return 0;
                }

                uint64_t word = 0;
                ifs >> word;
                ((uint64_t*)(label.flags_))[word_idx] = word;
            }

            int32_t x1, y1, x2, y2;
            ifs >> x1 >> y1 >> x2 >> y2;
            warthog::geom::rectangle dummy; 
            warthog::geom::rectangle rect(x1, y1, x2, y2); 
            if(rect != dummy && !rect.is_valid())
            {
                std::cerr 
                    << "err; invalid label on line " << lines 
                    << " (bb part); aborting\n";
                delete lab;
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n"
                    << "x1 " << x1 << x2;
                return 0;
            }
            label.bbox_ = rect;
            lab->labels_.at(i).push_back(label);
            assert(lab->labels_.at(i).back().bbox_ == rect);

            lines++;
       }
    }
    return lab;
}

bool
warthog::label::bbaf_labelling::load_bch_labels(
        const char* filename, warthog::graph::xy_graph* g,
        std::vector<uint32_t>* partitioning,
        std::vector<uint32_t>* lex_order, 
        warthog::label::bbaf_labelling*& out_lab_fwd,
        warthog::label::bbaf_labelling*& out_lab_bwd)
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

    out_lab_fwd = new warthog::label::bbaf_labelling(g, partitioning);
    out_lab_bwd = new warthog::label::bbaf_labelling(g, partitioning);

    // allocate enough memory to label the outgoing edge of every node 
    // and then read the labels from file
    // NB: label range is : [firstid, lastid)
    const uint32_t word_sz = sizeof(uint64_t);
    uint32_t words_per_label = 
        ceil(out_lab_fwd->bytes_per_af_label_ / (double)word_sz);
    for(uint32_t i = 0; i < g->get_num_nodes(); i++)
    {
        warthog::graph::node* n = g->get_node(i);
        for(uint32_t j = 0; j < n->out_degree(); j++)
        {

            // create one label per outgoing edge
            bbaf_label label;
            label.flags_= new uint8_t[words_per_label*word_sz];

            // read the arcflags part of the label
            for(uint32_t word_idx = 0; word_idx < words_per_label; word_idx++)
            {
                if(ifs.eof() || !ifs.good())
                {
                    std::cerr << "unexpected error while reading " 
                        << filename << "; line=" << lines << "\n";
                    std::cerr 
                        << "[debug info] node: " << i 
                        << " out-edge-index: " << j << "\n";
                    delete out_lab_fwd;
                    delete out_lab_bwd;
                    return false;
                }
                uint64_t word = 0;
                ifs >> word;
                ((uint64_t*)(label.flags_))[word_idx] = word;
            }

            // read the bbox part of the label
            int32_t x1, y1, x2, y2;
            ifs >> x1 >> y1 >> x2 >> y2;
            warthog::geom::rectangle dummy; 
            warthog::geom::rectangle rect(x1, y1, x2, y2); 
            if(rect != dummy && !rect.is_valid())
            {
                std::cerr 
                    << "err; invalid label on line " << lines 
                    << " (bb part); aborting\n";
                std::cerr 
                    << "[debug info] node: " << i 
                    << " out-edge-index: " << j << "\n"
                    << "x1 " << x1 << x2;
                delete out_lab_fwd;
                delete out_lab_bwd;
                return false;
            }
            label.bbox_ = rect;

            // assign the label to the appropriate collection
            // (with the set of up-edge labels or with the set of 
            // reversed down-edge labels)
            warthog::graph::edge* e = n->outgoing_begin() + j;
            if(lex_order->at(e->node_id_) < lex_order->at(i)) 
            { 
                // reverse down-edge label (used by bwd bch search)
                out_lab_bwd->labels_.at(e->node_id_).push_back(label);

                assert( rect == 
                        out_lab_bwd->labels_.at(e->node_id_).back().bbox_);
                for(uint32_t word_idx = 0; word_idx < words_per_label; word_idx++)
                {
                    assert( ((uint64_t*)(out_lab_bwd->labels_.at(e->node_id_)
                                    .back().flags_))[word_idx] == 
                            ((uint64_t*)(label.flags_))[word_idx]);
                }
            }
            else
            {
                // up-edge label (used by fwd bch search)
                out_lab_fwd->labels_.at(i).push_back(label);

                assert(out_lab_fwd->labels_.at(i).back().bbox_ == rect);
                for(uint32_t word_idx = 0; word_idx < words_per_label; word_idx++)
                {
                    assert( ((uint64_t*)(out_lab_fwd->labels_.at(i)
                                    .back().flags_))[word_idx] == 
                            ((uint64_t*)(label.flags_))[word_idx]);
                }
            }

            lines++;
        }
    }
    return true;
}
