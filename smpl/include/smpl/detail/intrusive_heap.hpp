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
intrusive_heap<Compare>::intrusive_heap(const compare& comp) :
    m_data(1, nullptr),
    m_comp(comp)
{
}

template <typename Compare>
template <typename InputIt>
intrusive_heap<Compare>::intrusive_heap(
    const compare& comp,
    InputIt first,
    InputIt last)
:
    m_data(),
    m_comp(comp)
{
    make_heap(first, last);
}

template <typename Compare>
template <typename InputIt>
intrusive_heap<Compare>::intrusive_heap(InputIt first, InputIt last) :
    m_data(),
    m_comp()
{
    make_heap(first, last);
}

template <typename Compare>
intrusive_heap<Compare>::intrusive_heap(intrusive_heap&& o) :
    m_data(std::move(o.m_data)),
    m_comp(std::move(o.m_comp))
{
}

template <typename Compare>
intrusive_heap<Compare>&
intrusive_heap<Compare>::operator=(intrusive_heap&& rhs)
{
    if (this != &rhs) {
        m_data = std::move(rhs.m_data);
        m_comp = std::move(rhs.m_comp);
    }
    return *this;
}

template <typename Compare>
heap_element* intrusive_heap<Compare>::min() const
{
    return m_data[1];
}

template <typename Compare>
typename intrusive_heap<Compare>::const_iterator
intrusive_heap<Compare>::begin() const
{
    return m_data.begin() + 1;
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
    return m_data.size() == 1;
}

template <typename Compare>
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::size() const
{
    return m_data.size() - 1;
}

template <typename Compare>
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::max_size() const
{
    return m_data.max_size() - 1;
}

template <typename Compare>
void intrusive_heap<Compare>::reserve(size_type new_cap)
{
    m_data.reserve(new_cap + 1);
}

template <typename Compare>
void intrusive_heap<Compare>::clear()
{
    for (size_t i = 1; i < m_data.size(); ++i) {
        m_data[i]->m_heap_index = 0;
    }
    m_data.resize(1);
}

template <typename Compare>
void intrusive_heap<Compare>::push(heap_element* e)
{
    e->m_heap_index = m_data.size();
    m_data.push_back(e);
    percolate_up(m_data.size() - 1);
}

template <typename Compare>
void intrusive_heap<Compare>::pop()
{
    if (!empty()) {
        m_data[1]->m_heap_index = 0;
        m_data[1] = m_data.back();
        m_data[1]->m_heap_index = 1;
        m_data.pop_back();
        percolate_down(1);
    }
}

template <typename Compare>
bool intrusive_heap<Compare>::contains(heap_element* e)
{
    return e->m_heap_index != 0;
}

template <typename Compare>
void intrusive_heap<Compare>::update(heap_element* e)
{
    erase(e);
    push(e);
}

template <typename Compare>
void intrusive_heap<Compare>::increase(heap_element* e)
{
    percolate_down(e->m_heap_index);
}

template <typename Compare>
void intrusive_heap<Compare>::decrease(heap_element* e)
{
    percolate_up(e->m_heap_index);
}

template <typename Compare>
void intrusive_heap<Compare>::erase(heap_element* e)
{
    size_type pos = e->m_heap_index;
    m_data[pos] = m_data.back();
    m_data[pos]->m_heap_index = pos;
    e->m_heap_index = 0;
    m_data.pop_back();
    percolate_down(pos);
}

template <typename Compare>
void intrusive_heap<Compare>::make()
{
    auto b = m_data.begin();
    ++b;
    auto e = m_data.end();
    make_heap(b, e);
}

template <typename Compare>
void intrusive_heap<Compare>::swap(intrusive_heap& o)
{
    // TODO: implement
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::ipow2(size_type exp)
{
    if (exp == 0) {
        return 1;
    }

    size_type res = ipow2(exp >> 1) * ipow2(exp >> 1);
    if (exp % 2) {
        res *= 2;
    }
    return res;
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
template <typename InputIt>
void intrusive_heap<Compare>::make_heap(InputIt first, InputIt last)
{
    clear();
    for (auto it = first; it != last; ++it) {
        push(*it);
    }

//    m_data.resize(std::distance(first, last) + 1);
//    make_heap(first, last, 1);
}

template <typename Compare>
template <typename InputIt>
void intrusive_heap<Compare>::make_heap(
    InputIt first,
    InputIt last,
    size_type root)
{
    const auto n = std::distance(first, last);
    printf("make heap from %d elements from %zu\n", n, root);
    print();

    if (n <= 0) {
        return;
    }

    printf(" -> data[%zu] = %p\n", root, *first);
    m_data[root] = *first;
    m_data[root]->m_heap_index = root;

    if (n == 1) {
        return;
    }

    const size_type left = left_child(root);
    const size_type right = right_child(root);

    auto f = ilog2(n) - 1;
    size_type f2 = ipow2(f);
    size_type l = f2 - 1 + std::min(n - 2 * f2 + 1, f2);
    size_type r = n - 1 - l;

    InputIt new_start = std::next(first);
    InputIt mid = std::next(new_start, l);

    make_heap(new_start, mid, left);
    make_heap(mid, last, right);
    percolate_down(root);
}

template <typename Compare>
inline
typename intrusive_heap<Compare>::size_type
intrusive_heap<Compare>::parent(size_type index) const
{
    return index >> 1;
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

template <typename Compare>
inline
void intrusive_heap<Compare>::percolate_down(size_type pivot)
{
    if (is_external(pivot)) {
        return;
    }

    size_type left = left_child(pivot);
    size_type right = right_child(pivot);

    heap_element* tmp = m_data[pivot];
    while (is_internal(left)) {
        size_type s = right;
        if (is_external(right) || m_comp(m_data[left], m_data[right])) {
            s = left;
        }

        if (m_comp(m_data[s], tmp)) {
            m_data[pivot] = m_data[s];
            m_data[pivot]->m_heap_index = pivot;
            pivot = s;
        } else {
            break;
        }

        left = left_child(pivot);
        right = right_child(pivot);
    }
    m_data[pivot] = tmp;
    m_data[pivot]->m_heap_index = pivot;
}

template <typename Compare>
inline
void intrusive_heap<Compare>::percolate_up(size_type pivot)
{
    heap_element* tmp = m_data[pivot];
    while (pivot != 1) {
        size_type p = parent(pivot);
        if (m_comp(m_data[p], tmp)) {
            break;
        }
        m_data[pivot] = m_data[p];
        m_data[pivot]->m_heap_index = pivot;
        pivot = p;
    }
    m_data[pivot] = tmp;
    m_data[pivot]->m_heap_index = pivot;
}

template <typename Compare>
inline
bool intrusive_heap<Compare>::is_internal(size_type index) const
{
    return index < m_data.size();
}

template <typename Compare>
inline
bool intrusive_heap<Compare>::is_external(size_type index) const
{
    return index >= m_data.size();
}

template <typename Compare>
void intrusive_heap<Compare>::print() const
{
    printf("[ null, ");
    for (int i = 1; i < m_data.size(); ++i) {
        printf(" %p", m_data[i]);
        if (i == m_data.size() - 1) {
            printf(" ");
        } else {
            printf(", ");
        }
    }
    printf("]\n");
}

} // namespace sbpl

#endif
