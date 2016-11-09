#ifndef SMPL_EXPERIENCE_GRAPH_H
#define SMPL_EXPERIENCE_GRAPH_H

// standard includes
#include <cstdlib>
#include <vector>

// system includes
#include <smpl/types.h>

namespace sbpl {
namespace motion {

// Strategies for building experience graphs from continuous path data:
//
// 1. The path data comes in the form as a set of continuous states
//
// * discretize every state in the set of continuous states
// * remove any duplicate discrete states
// * for each pair of discrete points
// *     if there exists an action that transitions between the two points
// *         add the transition path to the experience graph
//
// 2. The path data comes in the form as a set of paths (sequences of continuous states)
//
// * for each path, p
// *     dp0 = discretize(p0)
// *     dpf = discretize(p0)
// *     edge <- [ p0 ]
// *     for each point pi in path \ p0
// *         dpf = discretize(pi)
// *         if dpf != dp0
// *             add edge (dp0, dpf, edge)
// *         else
// *             edge = edge + [ pi ]

// undirected
class ExperienceGraph
{
    struct Node;
    struct Edge;

    typedef std::vector<Node> node_container;
    typedef std::vector<Edge> edge_container;

public:

    typedef node_container::size_type node_id;
    typedef edge_container::size_type edge_id;

private:

    struct Node
    {
        typedef std::vector<edge_id> adjacent_edge_container;

        RobotState state;
        adjacent_edge_container edges;
    };

    struct Edge
    {
        std::vector<RobotState> waypoints;
        node_id snode;
        node_id tnode;

        Edge(node_id uid, node_id vid) : waypoints(), snode(uid), tnode(vid) { }

        Edge(const std::vector<RobotState>& waypoints, node_id uid, node_id vid)
            : waypoints(waypoints), snode(uid), tnode(vid) { }
    };

public:

    ExperienceGraph(/*const RobotPlanningSpacePtr& pspace*/);

    /// \name IncidenceGraph Interface
    ///@{
    typedef Node::adjacent_edge_container::size_type degree_size_type;
    typedef Node::adjacent_edge_container::const_iterator out_edge_iterator;
    std::pair<out_edge_iterator, out_edge_iterator> out_edges(node_id id);
    node_id source(edge_id id) const;
    node_id target(edge_id id) const;
    degree_size_type out_degree(node_id id) const;
    ///@}

    /// \name BidirectionalGraph Interface
    ///@{
    typedef Node::adjacent_edge_container::const_iterator in_edge_iterator;
    std::pair<in_edge_iterator, in_edge_iterator> in_edges(node_id id);
    degree_size_type in_degree(node_id id);
    degree_size_type degree(node_id id);
    ///@}

    /// \name AdjacencyGraph Interface
    ///@{
    struct adjacency_iterator :
        std::iterator<std::random_access_iterator_tag, node_id>
    {
        adjacency_iterator(
            const edge_container& edges_ref,
            out_edge_iterator it);

        const value_type operator*() const;
        adjacency_iterator operator++(int);
        adjacency_iterator& operator++();

        adjacency_iterator& operator+=(difference_type n);
        adjacency_iterator& operator-=(difference_type n);
        difference_type operator-(adjacency_iterator it);
        bool operator==(adjacency_iterator it) const;
        bool operator!=(adjacency_iterator it) const;

    private:

        node_id m_node;
        const edge_container* m_edges_ref;
        out_edge_iterator m_it;
    };

//    typedef Node::adjacent_edge_container::iterator adjacency_iterator;
    std::pair<adjacency_iterator, adjacency_iterator> adjacent_nodes(node_id id);
    ///@}

    /// \name VertexListGraph Interface
    ///@{
    typedef std::vector<Node>::iterator node_iterator;
    typedef std::vector<Node>::const_iterator const_node_iterator;
    typedef node_container::size_type nodes_size_type;
    nodes_size_type num_nodes() const { return m_nodes.size(); }
    std::pair<node_iterator, node_iterator> nodes();
    std::pair<const_node_iterator, const_node_iterator> nodes() const;
    ///@}

    /// \name EdgeListGraph Interface
    ///@{
    typedef std::vector<Edge>::iterator edge_iterator;
    typedef std::vector<Edge>::const_iterator const_edge_iterator;
    typedef edge_container::size_type edges_size_type;
    edges_size_type num_edges() const { return m_edges.size(); }
    std::pair<edge_iterator, edge_iterator> edges();
    std::pair<const_edge_iterator, const_edge_iterator> edges() const;
    ///@}

    /// \name MutableGraph Interface
    node_id insert_node();
    void erase_node(node_id id);

    edge_id insert_edge(node_id uid, node_id vid);
    void erase_edge(node_id uid, node_id vid);
    void erase_edge(edge_id id);
    ///@}

    node_id get_id(const_node_iterator it) const;
    edge_id get_id(const_edge_iterator it) const;

    void insert_path(const std::vector<RobotState>& path);

private:

    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;
};

inline
ExperienceGraph::adjacency_iterator::adjacency_iterator(
    const edge_container& edges_ref,
    out_edge_iterator it)
:
    m_edges_ref(&edges_ref),
    m_it(it)
{
}

inline
const ExperienceGraph::adjacency_iterator::value_type
ExperienceGraph::adjacency_iterator::operator*() const
{
    return (*m_edges_ref)[*m_it].tnode;
}

inline
ExperienceGraph::adjacency_iterator
ExperienceGraph::adjacency_iterator::operator++(int)
{
    adjacency_iterator it(*m_edges_ref, m_it);
    ++m_it;
    return it;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator++()
{
    ++m_it;
    return *this;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator+=(difference_type n)
{
    m_it += n;
    return *this;
}

inline
ExperienceGraph::adjacency_iterator&
ExperienceGraph::adjacency_iterator::operator-=(difference_type n)
{
    return operator+=(-n);
}

inline
ExperienceGraph::adjacency_iterator::difference_type
ExperienceGraph::adjacency_iterator::operator-(adjacency_iterator it)
{
    return m_it - it.m_it;
}

inline
bool
ExperienceGraph::adjacency_iterator::operator==(adjacency_iterator it) const
{
    return it.m_it == m_it;
}

inline
bool
ExperienceGraph::adjacency_iterator::operator!=(adjacency_iterator it) const
{
    return it.m_it != m_it;
}

} // namespace motion
} // namespace sbpl

#endif
