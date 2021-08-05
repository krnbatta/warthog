#ifndef WARTHOG_SEARCH_NODE_H
#define WARTHOG_SEARCH_NODE_H

// search_node.h
//
// @author: dharabor
// @created: 10/08/2012
//

#include "constants.h"
#include "cpool.h"
#include "jps.h"

#include <iostream>

namespace warthog
{

const uint32_t STATUS_MASK = 1;
class search_node
{
	public:
		search_node(uint32_t id=warthog::NODE_NONE)
			: id_and_status_(id << 1), f_(warthog::INF),
            g_(warthog::INF), parent_id_(warthog::NODE_NONE),
			priority_(warthog::INF), searchid_(0)
		{
			assert(this->get_id() <= ((1ul<<31)-1));
            set_pdir(warthog::jps::direction::NONE);
						set_parent(-1);
			refcount_++;
		}

		~search_node()
		{
			refcount_--;
		}

		inline void
		init(uint32_t searchid, uint32_t parent_id, double g, double f)
		{
			id_and_status_ &= ~1;
            parent_id_= parent_id;
            f_ = f;
            g_ = g;
			searchid_ = searchid;
		}

		inline uint32_t
		get_search_id()
		{
			return searchid_;
		}

		inline void
		set_searchid(uint32_t searchid)
		{
			searchid_ = searchid;
		}

		inline uint32_t
		get_id() const { return id_and_status_ >> 1; }

		inline void
		set_id(uint32_t id)
		{
			id_and_status_ = (id << 1) | (id_and_status_ & 1);
		}

		inline warthog::jps::direction
		get_pdir() const
		{
			return (warthog::jps::direction)jps_parent_direction_;

		}

		inline void
		set_pdir(warthog::jps::direction d)
		{
			jps_parent_direction_ = d;
		}

		inline bool
		get_expanded() const { return (id_and_status_ & STATUS_MASK); }

		inline void
		set_expanded(bool expanded)
		{
			id_and_status_ &= ~STATUS_MASK; // reset bit0
			id_and_status_ ^= (uint32_t)(expanded?1:0); // set it anew
		}

		inline uint32_t
		get_parent() const { return parent_id_; }

		inline void
		set_parent(uint32_t parent_id) { parent_id_ = parent_id; }

		inline uint32_t
		get_priority() const { return priority_; }

		inline void
		set_priority(uint32_t priority) { priority_ = priority; }

		inline double
		get_g() const { return g_; }

		inline void
		set_g(double g) { g_ = g; }

		inline double
		get_f() const { return f_; }

		inline void
		set_f(double f) { f_ = f; }

		inline void
		relax(double g, uint32_t parent_id)
		{
			assert(g < g_);
			f_ = (f_ - g_) + g;
			g_ = g;
			parent_id_ = parent_id;
		}

		inline bool
		operator<(const warthog::search_node& other) const
		{
			if(f_ < other.f_)
			{
				return true;
			}
			if(f_ > other.f_)
			{
				return false;
			}

			// break ties in favour of larger g
			if(g_ > other.g_)
			{
				return true;
			}
			return false;
		}

		inline bool
		operator>(const warthog::search_node& other) const
		{
			if(f_ > other.f_)
			{
				return true;
			}
			if(f_ < other.f_)
			{
				return false;
			}

			// break ties in favour of larger g
			if(g_ > other.g_)
			{
				return true;
			}
			return false;
		}

		inline bool
		operator==(const warthog::search_node& other) const
		{
			if( !(*this < other) && !(*this > other))
			{
				return true;
			}
			return false;
		}

		inline bool
		operator<=(const warthog::search_node& other) const
		{
			if(*this < other)
			{
				return true;
			}
			if(!(*this > other))
			{
				return true;
			}
			return false;
		}

		inline bool
		operator>=(const warthog::search_node& other) const
		{
			if(*this > other)
			{
				return true;
			}
			if(!(*this < other))
			{
				return true;
			}
			return false;
		}

		inline void
		print(std::ostream&  out) const
		{
			out << "search_node id:" << get_id();
            out << " p_id: ";
            out << parent_id_;
            out << " g: "<<g_ <<" f: "<<this->get_f()
            << " expanded: " << get_expanded() << " "
            << " searchid: " << searchid_
            << " pdir: "<< get_pdir() << " ";
		}

		static uint32_t
		get_refcount() { return refcount_; }

		uint32_t
		mem()
		{
			return sizeof(*this);
		}

	private:
		uint64_t id_and_status_; // bit 0 is expansion status; 1-63 are id
		double f_;
		double g_;
        uint64_t parent_id_;
		uint32_t priority_; // expansion priority
		uint32_t searchid_;
        uint8_t jps_parent_direction_; // hack

		static uint32_t refcount_;
};

struct cmp_less_search_node
{
    inline bool
    operator()(
            const warthog::search_node& first,
            const warthog::search_node& second)
    {
        return first < second;
    }
};

struct cmp_greater_search_node
{
    inline bool
    operator()(
            const warthog::search_node& first,
            const warthog::search_node& second)
    {
        return first > second;
    }
};

struct cmp_less_search_node_f_only
{
    inline bool
    operator()(
            const warthog::search_node& first,
            const warthog::search_node& second)
    {
        return first.get_f() < second.get_f();
    }
};

}

#endif
