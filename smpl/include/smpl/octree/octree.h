////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_OCTREE_H
#define SMPL_OCTREE_H

// standard includes
#include <cstdlib>
#include <memory>
#include <utility>
#include <stack>

#include "detail/octree_base.h"

namespace sbpl {

/// A tree data structure storing elements that have either 8 children, if the
/// node is internal, or no children, if the node is a leaf node. This class
/// contains functionality for maintenance of the tree (expanding or collapsing
/// nodes), traversal of the tree, and basic properties of the tree such as its
/// current depth.
template <
    class T,
    class Allocator = std::allocator<T>>
class OcTree : protected detail::OcTreeBase<T, Allocator>
{
protected:

    typedef detail::OcTreeBase<T, Allocator> Base;

public:

    typedef T value_type;
    typedef Allocator allocator_type;

    typedef std::size_t size_type;

    using typename Base::node_type;

    struct iterator
    {
        std::stack<node_type*, std::vector<node_type*>> s;

        iterator& operator++()
        {
            node_type* t = s.top();
            s.pop();
            if (t->children) {
                for (int i = 0; i < 8; ++i) {
                    s.push(&t->children[i]);
                }
            }
            return *this;
        }

        iterator operator++(int)
        { iterator i = *this; ++*this; return i; }

        value_type& operator*() { return s.top()->value; }

        value_type* operator->() { return &s.top()->value; }

        bool operator==(const iterator& i) const { return s == i.s; }

        bool operator!=(const iterator& i) const { return !(s == i.s); }
    };

    OcTree();
    OcTree(const T& value);

    explicit OcTree(const Allocator& alloc);
    explicit OcTree(const T& value, const Allocator& alloc);

    OcTree(const OcTree& o);
    OcTree(OcTree&& o);

    ~OcTree();

    OcTree& operator=(const OcTree& rhs);
    OcTree& operator=(OcTree&& rhs);

    allocator_type get_allocator() const { return Base::get_allocator(); }

    /// \name Size Properties
    ///@{
    int depth() const;

    size_type num_nodes() const;
    size_type num_leaves() const;

    size_type mem_usage() const;
    ///@}

    node_type* root();
    const node_type* root() const;

    // Collapse a node by removing its children and assigning the node a new
    // value. The node must not have any grandchildren, or they will be lost.
    void collapse_node(node_type* n, const T& value);

    // Expand a node by creating all of its children and assigning them the
    // value stored at this node. The node must not currently have any children.
    void expand_node(node_type* n);

    /// \name Iterators
    ///@{
    std::pair<iterator, iterator> nodes()
    {
        std::pair<iterator, iterator> p;
        p.first.s.push(root());
        return p;
    }

    struct leaf_iterator;
    std::pair<leaf_iterator, leaf_iterator> leaves();
    ///@}

    template <typename Callable>
    void accept(Callable c);

protected:

    typedef typename Base::node_allocator_type node_allocator_type;

    using Base::m_impl;
    using Base::get_node_allocator;
    using Base::alloc_children;
    using Base::dealloc_children;
    using Base::clear;
    using Base::construct_node;
//    using Base::destroy_node;
    using Base::construct_children;
    using Base::destroy_children;

    typedef std::allocator_traits<allocator_type> atraits;

    int depth(const node_type* n) const;
    size_type count_nodes(const node_type* n) const;
    size_type count_leaves(const node_type* n) const;

    size_type mem_usage(const node_type* n) const;

    void clone_children(node_type *nout, const node_type *nin);
    void move_children(node_type *nout, node_type *nin);

    template <typename Callable>
    void accept(Callable c, node_type* n);
};

} // namespace sbpl

#include "detail/octree.hpp"

#endif
