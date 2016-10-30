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

#ifndef SMPL_INTRUSIVE_HEAP_HPP
#define SMPL_INTRUSIVE_HEAP_HPP

#include "../intrusive_heap.h"

namespace sbpl {

template <typename Compare>
intrusive_heap<Compare>::intrusive_heap(
    const compare& comp,
    const container_type& elements)
:
    m_data(),
    m_comp(comp)
{
}

template <typename Compare>
intrusive_heap<Compare>::intrusive_heap(intrusive_heap&& o)
{
}

template <typename Compare>
intrusive_heap<Compare>::~intrusive_heap()
{
}

template <typename Compare>
intrusive_heap<Compare>&
intrusive_heap<Compare>::operator=(intrusive_heap&& rhs)
{
    return *this;
}

template <typename Compare>
heap_element* intrusive_heap<Compare>::min() const
{
    return nullptr;
}

template <typename Compare>
typename intrusive_heap<Compare>::const_iterator
intrusive_heap<Compare>::begin() const
{
    return m_data.begin();
}

template <typename Compare>
typename intrusive_heap<Compare>::const_iterator
intrusive_heap<Compare>::end() const
{
    return m_data.end();
}

template <typename Compare>
bool intrusive_heap<Compare>::empty() const
{
    return false;
}

template <typename Compare>
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::size() const
{
    return 0;
}

template <typename Compare>
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::max_size() const
{
    return 0;
}

template <typename Compare>
void intrusive_heap<Compare>::reserve(size_type new_cap)
{
}

template <typename Compare>
void intrusive_heap<Compare>::clear()
{
}

template <typename Compare>
heap_element* intrusive_heap<Compare>::push(heap_element* e)
{
    return nullptr;
}

template <typename Compare>
void intrusive_heap<Compare>::pop()
{
}

template <typename Compare>
bool intrusive_heap<Compare>::contains(heap_element* e)
{
}

template <typename Compare>
void intrusive_heap<Compare>::update(heap_element* e)
{
}

template <typename Compare>
void intrusive_heap<Compare>::increase(heap_element* e)
{
}

template <typename Compare>
void intrusive_heap<Compare>::decrease(heap_element* e)
{
}

template <typename Compare>
void intrusive_heap<Compare>::erase(heap_element* e)
{
}

template <typename Compare>
void intrusive_heap<Compare>::swap(intrusive_heap& o)
{
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::ipow2(size_type exp)
{
    return (exp == 0) ? 1 : ((exp % 2) ? 2 : 1) * ipow2(exp >> 1) * ipow2(exp >> 1);
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::ilog2(size_type i)
{
    std::size_t r = 0;
    while (i >>= 1) {
        ++r;
    }
    return r;
}

template <typename Compare>
inline
bool intrusive_heap<Compare>::ispow2(size_type val)
{
    // does not check for val == 0
    return !(val & (val - 1));
}

template <typename Compare>
void intrusive_heap<Compare>::make_heap(const container_type& elements)
{
    m_data.resize(elements.size() + 1);
    make_heap(elements, 1, 0, elements.size());
}

template <typename Compare>
void intrusive_heap<Compare>::make_heap(
    const container_type& elements,
    size_type root, size_type start, size_type end)
{
    if (end - start <= 0) {
        return;
    }

    m_data[root] = elements[start];
    m_data[root]->m_heap_index = root;
    size_type left = left_child(root);
    size_type right = right_child(root);

    size_type n = end - start;
    if (n == 1) {
        return;
    }

    size_type f = ilog2(n) - 1;
    size_type f2 = ipow2(f);
    size_type l = f2 - 1 + std::min(n - 2 * f2 + 1, f2);
    size_type r = n - 1 - l;

    size_type new_start = start + 1;
    size_type mid = new_start + l;

    make_heap(elements, left, new_start, mid);
    make_heap(elements, right, mid, end);
//    percolate_down(root);
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::right_child(size_type index) const
{
    return (index << 1) + 1;
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::left_child(size_type index) const
{
    return index << 1;
}

} // namespace sbpl

#endif
