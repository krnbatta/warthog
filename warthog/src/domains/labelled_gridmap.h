#ifndef WARTHOG_LABELLED_GRIDMAP_H
#define WARTHOG_LABELLED_GRIDMAP_H

// domains/weighted_gridmap.h
//
// A gridmap with weights/labels. This data structure supports
// labels on grid edges as well as vertices. 
// This implementation stores the map as a flat 1d array of "cell"
// objects. We add some padding around the map to simplify access 
// operations:
//  - a terminator character is added to indicate end-of-row.
//  - a line of terminator characters are added before the first row.
//  - a line of terminator characters are added after the last row.
//
// @author: dharabor
// @created: 2018-11-08
// 

#include "constants.h"
#include "helpers.h"
#include "gm_parser.h"

#include <climits>
#include <stdint.h>

namespace warthog
{

// we store labels for each vertex and each of its up to 5 applicable actions
struct labelled_cell
{
    labelled_cell()
    {
        // everything is blocked by default
        v_lab_ = UINT32_MAX;
        e_lab_[0] = 1;
        e_lab_[1] = 1;
        e_lab_[2] = 1;
        e_lab_[3] = 1;
        e_lab_[4] = 1;
    }

    labelled_cell&
    operator=(const labelled_cell& other)
    {
        v_lab_ = other.v_lab_;
        e_lab_[0] = other.e_lab_[0];
        e_lab_[1] = other.e_lab_[1];
        e_lab_[2] = other.e_lab_[2];
        e_lab_[3] = other.e_lab_[3];
        e_lab_[4] = other.e_lab_[4];
        return *this;
    }

    labelled_cell&
    operator=(const char raw_input_tile)
    {
        if(raw_input_tile == '.')
        {
            v_lab_ = 0;
        }
        else
        {
            v_lab_ = UINT32_MAX;
        }
        e_lab_[0] = 1;
        e_lab_[1] = 1;
        e_lab_[2] = 1;
        e_lab_[3] = 1;
        e_lab_[4] = 1;
        return *this;
    }

    bool
    operator==(const labelled_cell& other)
    {
        return 
            v_lab_ == other.v_lab_ &&
            e_lab_[0] == other.e_lab_[0] && 
            e_lab_[1] == other.e_lab_[1] && 
            e_lab_[2] == other.e_lab_[2] && 
            e_lab_[3] == other.e_lab_[3] && 
            e_lab_[4] == other.e_lab_[4];
    }

    uint64_t v_lab_;
    double e_lab_[5];

};


template<class CELL>
class labelled_gridmap
{
	public:
        labelled_gridmap(unsigned int h, unsigned int w)
            : header_(h, w, "octile")
        {	
            this->init_db();
        }

        labelled_gridmap(const char* filename)
        {
            // SPECIALISE ME
        }

        ~labelled_gridmap()
        {
            delete [] db_;
        }

        
		// here we convert from the coordinate space of 
		// the original grid to the coordinate space of db_. 
		inline uint32_t
		to_padded_id(uint32_t node_id)
		{
			return node_id + 
				// padded rows before the actual map data starts
				padded_rows_before_first_row_*padded_width_ +
			   	// padding from each row of data before this one
				(node_id / header_.width_) * padding_per_row_;
		}

		inline uint32_t
		to_padded_id(uint32_t x, uint32_t y)
		{
			return to_padded_id(y * this->header_width() + x);
		}

		inline void
		to_unpadded_xy(uint32_t padded_id, uint32_t& x, uint32_t& y)
		{
			padded_id -= padded_rows_before_first_row_* padded_width_;
			y = padded_id / padded_width_;
			x = padded_id % padded_width_;
		}

		// get the label associated with the padded coordinate pair (x, y)
		inline CELL&
		get_label(uint32_t x, unsigned int y)
		{
			return this->get_label(y*padded_width_+x);
		}

		inline CELL&
		get_label(uint32_t padded_id)
		{
            return db_[padded_id];
		}

        inline void
		get_neighbours(uint32_t db_id, uint32_t ids[9], CELL labels[9])
		{
            // calculate ids of all adjacent neighbours
            ids[4] = db_id;
            ids[3] = db_id-1; // west
            ids[5] = db_id+1; // east
            ids[1] = db_id - this->padded_width_; // north
            ids[7] = db_id + this->padded_width_; // south
            ids[0] = ids[3] - this->padded_width_; // northwest
            ids[2] = ids[5] - this->padded_width_; // northeast
            ids[6] = ids[3] + this->padded_width_; // southwest
            ids[8] = ids[5] + this->padded_width_; // southeast

            // read terrain costs
            labels[0] = db_[ids[0]];
            labels[1] = db_[ids[1]];
            labels[2] = db_[ids[2]];
            labels[3] = db_[ids[3]];
            labels[4] = db_[ids[4]];
            labels[5] = db_[ids[5]];
            labels[6] = db_[ids[6]];
            labels[7] = db_[ids[7]];
            labels[8] = db_[ids[8]];
		}

		// set the label associated with the padded coordinate pair (x, y)
		inline void
		set_label(uint32_t x, unsigned int y, CELL label)
		{
			this->set_label(y*padded_width_+x, label);
		}

		inline void 
		set_label(uint32_t padded_id, CELL label)
		{
            db_[padded_id] = label;
		}

		inline uint32_t 
		height() const
		{ 
			return this->padded_height_;
		} 

		inline uint32_t 
		width() const 
		{ 
			return this->padded_width_;
		}

		inline uint32_t 
		header_height()
		{
			return this->header_.height_;
		}

		inline uint32_t 
		header_width()
		{
			return this->header_.width_;
		}

		inline const char*
		filename()
		{
			return this->filename_;
		}

		inline uint32_t 
		mem()
		{
			return sizeof(*this) +
			sizeof(CELL) * db_size_;
		}


	private:

		char filename_[256];
		warthog::gm_header header_;
		CELL* db_;

        uint32_t db_size_;
        uint32_t padding_per_row_;
		uint32_t padded_rows_before_first_row_;
		uint32_t padded_rows_after_last_row_;
        uint32_t padded_width_;
        uint32_t padded_height_;

		labelled_gridmap(const warthog::labelled_gridmap<CELL>& other) {}
		labelled_gridmap& operator=(const warthog::labelled_gridmap<CELL>& other) { return *this; }

        void
        init_db()
        {
            // when storing the grid we pad the edges of the map.
            // this eliminates the need for bounds checking when
            // fetching the neighbours of a node. 
            this->padded_rows_before_first_row_ = 3;
            this->padded_rows_after_last_row_ = 3;
            this->padding_per_row_ = 1;

            this->padded_width_ = this->header_.width_ + this->padding_per_row_;
            this->padded_height_ = this->header_.height_ + 
                this->padded_rows_after_last_row_ +
                this->padded_rows_before_first_row_;

            this->db_size_ = this->padded_height_ * padded_width_;

            // create a one dimensional dbword array to store the grid
            this->db_ = new CELL[db_size_];
        }
};

// vertex-labelled gridmap
typedef warthog::labelled_gridmap<warthog::dbword> vl_gridmap;

// edge-and-vertex-labelled gridmap 
typedef warthog::labelled_gridmap<warthog::labelled_cell> evl_gridmap;

template<>
inline
warthog::labelled_gridmap<warthog::dbword>::labelled_gridmap(const char* filename)
{
    warthog::gm_parser parser(filename);
    strcpy(filename_, filename);
    init_db();

    // populate matrix
    for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
    {
        char c = parser.get_tile_at(i);
        switch(c)
        {
            // explicit obstacle
            case '@':  
                set_label(to_padded_id(i), c);
                assert(get_label(to_padded_id(i)) == c);
                break;
            // other tiles have terrain cost equal to their ascii value
            default: 
                set_label(to_padded_id(i), c);
                assert(get_label(to_padded_id(i)) == c);
                break;
        }
    }
}

template<>
inline
warthog::labelled_gridmap<warthog::labelled_cell>::labelled_gridmap(
        const char* filename)
{
    warthog::gm_parser parser(filename);
	header_ = parser.get_header();
    strcpy(filename_, filename);
    init_db();

    // populate matrix
    for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
    {
        char c = parser.get_tile_at(i);
        warthog::labelled_cell cell;
        switch(c)
        {
            // these tiles are traversable
            case '.':
                cell.v_lab_ = 0;
                set_label(to_padded_id(i), cell);
                assert(get_label(to_padded_id(i)) == cell);
                break;
            // everything else is an obstacle
            default:
                cell.v_lab_ = UINT32_MAX;
                assert(get_label(to_padded_id(i)).v_lab_ == UINT32_MAX);
                break;
        }
    }
}

}

#endif

