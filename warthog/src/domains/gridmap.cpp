#include "gridmap.h"

#include <cassert>
#include <cstring>

warthog::gridmap::gridmap(unsigned int h, unsigned int w)
	: header_(h, w, "octile")
{	
	this->init_db();
}

warthog::gridmap::gridmap(const char* filename)
{
	strcpy(filename_, filename);
	warthog::gm_parser parser(filename);
	this->header_ = parser.get_header();

	init_db();
	// populate matrix
	for(unsigned int i = 0; i < parser.get_num_tiles(); i++)
	{
		char c = parser.get_tile_at(i);
		switch(c)
		{
			case 'S':
			case 'W': 
			case 'T':
			case '@':
			case 'O': // these terrain types are obstacles
				this->set_label(to_padded_id(i), 0); 
				assert(this->get_label(to_padded_id(i)) == 0);
				break;
			default: // everything else is traversable
				this->set_label(to_padded_id(i), 1); 
				assert(this->get_label(to_padded_id(i)) == 1);
				break;
		}
	}
}

void
warthog::gridmap::init_db()
{
	// when storing the grid we pad the edges of the map with
	// zeroes. this eliminates the need for bounds checking when
	// fetching the neighbours of a node. 
	this->padded_rows_before_first_row_ = 3;
	this->padded_rows_after_last_row_ = 3;
	this->dbheight_ = this->header_.height_ + 
		padded_rows_after_last_row_ +
		padded_rows_before_first_row_;
	this->dbwidth_  = (this->header_.width_ >> warthog::LOG2_DBWORD_BITS) + 1;

	// calculate # of extra/redundant padding bits required,
	// per row, to align map width with dbword size
	this->padded_height_ = this->dbheight_;
	this->padded_width_ = (this->dbwidth_ * warthog::DBWORD_BITS);
	this->padding_per_row_ = this->padded_width_ - this->header_.width_;
	this->db_size_ = this->dbwidth_ * this->dbheight_;

	// create a one dimensional dbword array to store the grid
	this->db_ = new warthog::dbword[db_size_];
	for(unsigned int i=0; i < db_size_; i++)
	{
		db_[i] = 0;
	}

	max_id_ = db_size_-1;
}

warthog::gridmap::~gridmap()
{
	delete [] db_;
}

void 
warthog::gridmap::print(std::ostream& out)
{
	out << "printing padded map" << std::endl;
	out << "-------------------" << std::endl;
	out << "type "<< header_.type_ << std::endl;
	out << "height "<< this->height() << std::endl;
	out << "width "<< this->width() << std::endl;
	out << "map" << std::endl;
	for(unsigned int y=0; y < this->height(); y++)
	{
		for(unsigned int x=0; x < this->width(); x++)
		{
			char c = this->get_label(y*this->width()+x);
			out << (c ? '.' : '@');
		}
		out << std::endl;
	}	
}
