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

// standard includes
#include <algorithm>
#include <iostream>

#define BOOST_TEST_MODULE ExperienceGraphTest
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

// system includes
#include <smpl/graph/experience_graph.h>

namespace smpl = sbpl::motion;

bool IteratedAllNodes(const smpl::ExperienceGraph& eg)
{
    std::vector<bool> found(eg.num_nodes(), false);
    auto nodes = eg.nodes();
    for (auto it = nodes.first; it != nodes.second; ++it) {
        found[*it] = true;
    }
    return std::count(found.begin(), found.end(), true) == found.size();
}

bool IteratedAllEdges(const smpl::ExperienceGraph& eg)
{
    std::vector<bool> found(eg.num_edges(), false);
    auto edges = eg.edges();
    for (auto it = edges.first; it != edges.second; ++it) {
        found[*it] = true;
    }
    return std::count(found.begin(), found.end(), true) == found.size();
}

BOOST_AUTO_TEST_CASE(DefaultConstructorTest)
{
    smpl::ExperienceGraph eg;
    BOOST_CHECK_EQUAL(eg.num_nodes(), 0);
    BOOST_CHECK(eg.nodes().first == eg.nodes().second);
    BOOST_CHECK_EQUAL(eg.num_edges(), 0);
    BOOST_CHECK(eg.edges().first == eg.edges().second);
}

BOOST_AUTO_TEST_CASE(InsertNodesTest)
{
    smpl::ExperienceGraph eg;
    smpl::RobotState zero_state;

    smpl::ExperienceGraph::node_id n1 = eg.insert_node(zero_state);
    BOOST_CHECK(eg.edges(n1).first == eg.edges(n1).second);
    BOOST_CHECK_EQUAL(eg.degree(n1), 0);
    BOOST_CHECK(eg.adjacent_nodes(n1).first == eg.adjacent_nodes(n1).second);
    BOOST_CHECK_EQUAL(eg.num_nodes(), 1);
    BOOST_CHECK_EQUAL(std::distance(eg.nodes().first, eg.nodes().second), 1);
    BOOST_CHECK_EQUAL(*eg.nodes().first, n1);

    smpl::ExperienceGraph::node_id n2 = eg.insert_node(zero_state);
    BOOST_CHECK(eg.edges(n2).first == eg.edges(n2).second);
    BOOST_CHECK_EQUAL(eg.degree(n2), 0);
    BOOST_CHECK(eg.adjacent_nodes(n2).first == eg.adjacent_nodes(n2).second);
    BOOST_CHECK_EQUAL(eg.num_nodes(), 2);
    BOOST_CHECK_EQUAL(std::distance(eg.nodes().first, eg.nodes().second), 2);

    smpl::ExperienceGraph::node_id n3 = eg.insert_node(zero_state);
    BOOST_CHECK(eg.edges(n3).first == eg.edges(n3).second);
    BOOST_CHECK_EQUAL(eg.degree(n3), 0);
    BOOST_CHECK(eg.adjacent_nodes(n3).first == eg.adjacent_nodes(n3).second);
    BOOST_CHECK_EQUAL(eg.num_nodes(), 3);
    BOOST_CHECK_EQUAL(std::distance(eg.nodes().first, eg.nodes().second), 3);

    BOOST_CHECK(IteratedAllNodes(eg));
}

BOOST_AUTO_TEST_CASE(InsertEdgesTest)
{
    smpl::ExperienceGraph eg;
    smpl::RobotState zero_state;

    smpl::ExperienceGraph::node_id n1 = eg.insert_node(zero_state);
    smpl::ExperienceGraph::node_id n2 = eg.insert_node(zero_state);
    smpl::ExperienceGraph::node_id n3 = eg.insert_node(zero_state);

    smpl::ExperienceGraph::edge_id e12 = eg.insert_edge(n1, n2);

    BOOST_CHECK_EQUAL(eg.num_edges(), 1);
    BOOST_CHECK_EQUAL(std::distance(eg.edges().first, eg.edges().second), 1);
    BOOST_CHECK(IteratedAllEdges(eg));
    BOOST_CHECK_EQUAL(std::distance(eg.adjacent_nodes(n1).first, eg.adjacent_nodes(n1).second), 1);
    BOOST_CHECK_EQUAL(std::distance(eg.adjacent_nodes(n2).first, eg.adjacent_nodes(n2).second), 1);
    BOOST_CHECK_EQUAL(*eg.adjacent_nodes(n1).first, n2);
    BOOST_CHECK_EQUAL(*eg.adjacent_nodes(n2).first, n1);

    BOOST_CHECK_EQUAL(std::distance(eg.edges(n1).first, eg.edges(n1).second), 1);
    BOOST_CHECK_EQUAL(std::distance(eg.edges(n2).first, eg.edges(n2).second), 1);
    BOOST_CHECK_EQUAL(*eg.edges(n1).first, e12);
    BOOST_CHECK_EQUAL(*eg.edges(n2).first, e12);
    BOOST_CHECK(eg.edges(n3).first == eg.edges(n3).second);
}

BOOST_AUTO_TEST_CASE(DumpTest)
{
    smpl::ExperienceGraph eg;

    smpl::RobotState zero_state;

    auto u = eg.insert_node(zero_state);
    std::cout << "inserted node " << u << std::endl;

    auto v = eg.insert_node(zero_state);
    std::cout << "inserted node " << v << std::endl;

    auto uv = eg.insert_edge(u, v);
    std::cout << "inserted edge " << uv << std::endl;

    auto nits = eg.nodes();
    for (auto it = nits.first; it != nits.second; ++it) {
        std::cout << eg.degree(*it) << " edges" << std::endl;
    }

    auto anits = eg.adjacent_nodes(u);
    std::cout << "adjacent(" << u << "): ";
    for (auto it = anits.first; it != anits.second; ++it) {
        std::cout << *it << ' ';
    }
    std::cout << std::endl;

    anits = eg.adjacent_nodes(v);
    std::cout << "adjacent(" << v << "): ";
    for (auto it = anits.first; it != anits.second; ++it) {
        std::cout << *it << ' ';
    }
    std::cout << std::endl;

    eg.erase_edge(u, v);
}
