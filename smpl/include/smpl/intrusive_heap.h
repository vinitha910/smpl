////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_INTRUSIVE_HEAP_H
#define SMPL_INTRUSIVE_HEAP_H

#include <vector>

namespace sbpl {

template <typename Compare>
class intrusive_heap;

struct heap_element
{

    heap_element() : m_heap_index(0) { }

private:

    int m_heap_index;

    template <typename Compare> friend class intrusive_heap<Compare>;
};

template <typename Compare>
class intrusive_heap
{
public:

    typedef Compare compare;

    typedef std::vector<heap_element*> container_type;
    typedef typename container_type::size_type size_type;

    typedef container_type::iterator iterator;
    typedef container_type::const_iterator const_iterator;

    intrusive_heap(
        const compare& comp = compare(),
        const container_type& elements = container_type());
    intrusive_heap(const intrusive_heap&) = delete;
    intrusive_heap(intrusive_heap&& o);

    ~intrusive_heap();

    intrusive_heap& operator=(const intrusive_heap&) = delete;
    intrusive_heap& operator=(intrusive_heap&& rhs);

    heap_element* min() const;

    const_iterator begin() const;
    const_iterator end() const;

    bool empty() const;
    size_type size() const;
    size_type max_size() const;
    void reserve(size_type new_cap);

    void clear();
    heap_element* push(heap_element* e);
    void pop();
    bool contains(heap_element* e);
    void update(heap_element* e);
    void increase(heap_element* e);
    void decrease(heap_element* e);
    void erase(heap_element* e);

    void swap(intrusive_heap& o);

private:

    container_type m_data;
    Compare m_comp;

    size_type ipow2(size_type i);
    size_type ilog2(size_type i);
    bool ispow2(size_type val);

    void make_heap(const container_type& elements);
    void make_heap(
        const container_type& elements,
        size_type root, size_type start, size_type end);

    size_type right_child(size_type index) const;
    size_type left_child(size_type index) const;
};

} // namespace sbpl

#include "detail/intrusive_heap.hpp"

#endif
