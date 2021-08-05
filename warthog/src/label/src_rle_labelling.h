#ifndef WARTHOG_LABEL_SRC_RLE_LABELLING_H
#define WARTHOG_LABEL_SRC_RLE_LABELLING_H

// label/src_rle_labelling.h
//
// Compressed Path Databases (a labelling on the nodes of the graph)
// implemented using single-row run-length encoding.
// Before compression we order the columns of the APSP matrix using a 
// DFS pre-order. For theoretical details see:
//
// B. Strasser, A. Botea. and D. Harabor. 
// Compressing Optimal Paths with Run Length Encoding. 
// 2015. 
// In Journal of Artificial Intelligence Research (JAIR). 
// Volume 54.
//
// @author: dharabor
// @created: 2017-05-05
//

namespace warthog
{

namespace label
{

class src_rle_labelling
{
    public:

        warthog::label::src_rle_labelling*
        load(const char* filename, warthog::graph::planar_graph* g)
        {
        }

        template <typename t_expander>
        static warthog::label::bb_labelling*
        compute(warthog::graph::planar_graph* g, 
                std::function<t_expander*(void)>& fn_new_expander,
                uint32_t first_id=0, uint32_t last_id=UINT32_MAX,
                std::vector<uint32_t>* node_order = 0)
        {
            if(!node_order) { dfs_preorder(node_order); }
        }

    private:
        // creation via ::compute and ::load only, please
        src_rle_labelling(warthog::graph::planar_graph* g);


        std::vector<std::vector<uint64_t>> labels_;
        warthog::graph::planar_graph* g_;

        

};

}

}

#endif

