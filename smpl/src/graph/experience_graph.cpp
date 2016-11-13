#include <smpl/graph/experience_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sbpl {
namespace motion {

ExperienceGraph::ExperienceGraph()
{
}

std::pair<
    ExperienceGraph::node_iterator,
    ExperienceGraph::node_iterator>
ExperienceGraph::nodes() const
{
    return std::make_pair(
            node_iterator(0),
            node_iterator(m_nodes.size()));
}

std::pair<
    ExperienceGraph::edge_iterator,
    ExperienceGraph::edge_iterator>
ExperienceGraph::edges() const
{
    return std::make_pair(
            edge_iterator(0),
            edge_iterator(m_edges.size()));
}

std::pair<
    ExperienceGraph::incident_edge_iterator,
    ExperienceGraph::incident_edge_iterator>
ExperienceGraph::edges(node_id id) const
{
    return std::make_pair(
            incident_edge_iterator(m_nodes[id].edges.begin()),
            incident_edge_iterator(m_nodes[id].edges.end()));
}

std::pair<
    ExperienceGraph::adjacency_iterator,
    ExperienceGraph::adjacency_iterator>
ExperienceGraph::adjacent_nodes(node_id id) const
{
    return std::make_pair(
            adjacency_iterator(m_nodes[id].edges.begin()),
            adjacency_iterator(m_nodes[id].edges.end()));
}

ExperienceGraph::degree_size_type ExperienceGraph::degree(node_id id) const
{
    return m_nodes[id].edges.size();
}

ExperienceGraph::node_id ExperienceGraph::source(edge_id id) const
{
    return m_edges[id].snode;
}

ExperienceGraph::node_id ExperienceGraph::target(edge_id id) const
{
    return m_edges[id].tnode;
}

ExperienceGraph::node_id ExperienceGraph::insert_node(const RobotState& state)
{
    m_nodes.emplace_back(state);
    return m_nodes.size() - 1;
}

void ExperienceGraph::erase_node(node_id id)
{
    if (id >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_node called with invalid node id");
    }
    // remove the node
    m_nodes.erase(m_nodes.begin() + id);

    auto will_remove = [id](const Edge& e) {
        return e.snode == id | e.tnode == id;
    };

    // count the number of edges to be removed (for updating adjacency edge ids)
    auto rem_edge_count = std::count_if(
            m_edges.begin(), m_edges.end(), will_remove);

    for (auto& node : m_nodes) {
        // remove adjacency edges that will be removed
        auto it = std::remove_if(
                node.edges.begin(), node.edges.end(),
                [&](const Node::adjacency& a) { return will_remove(m_edges[a.first]); });
        node.edges.resize(std::distance(node.edges.begin(), it));

        // update edge ids of remaining adjacency edges
        for (Node::adjacency& a : node.edges) {
            a.first -= rem_edge_count;
        }
    }

    // remove the edges to be removed
    auto remit = std::remove_if(m_edges.begin(), m_edges.end(), will_remove);
    m_edges.erase(remit, m_edges.end());

    // update the node ids stored in edges
    for (auto& edge : m_edges) {
        if (edge.snode > id) {
            --edge.snode;
        }
        if (edge.tnode > id) {
            --edge.tnode;
        }
    }
}

ExperienceGraph::edge_id ExperienceGraph::insert_edge(node_id uid, node_id vid)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(eid, uid, vid);
    return eid;
}

ExperienceGraph::edge_id ExperienceGraph::insert_edge(
    node_id uid,
    node_id vid,
    const std::vector<RobotState>& path)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::insert_edge called with invalid node ids");
    }

    m_edges.emplace_back(path, uid, vid);
    ExperienceGraph::edge_id eid = m_edges.size() - 1;
    insert_incident_edge(eid, uid, vid);
    return eid;
}

/// Erase the edge between two nodes, if one exists. All incident edge and
/// adjacent node iterators for the source and target nodes, edge iterators
/// pointing to edges after the erased edge, and edge ids greater than the
/// erased edge id are invalidated. TODO: erase all parallel edges?
void ExperienceGraph::erase_edge(node_id uid, node_id vid)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid node ids");
    }

    // search through the smaller of the two adjacency lists for the edge id
    // and call erase_edge on it when found
    if (m_nodes[uid].edges.size() < m_nodes[vid].edges.size()) {
        auto& out_edges = m_nodes[uid].edges;
        auto it = std::find_if(
                out_edges.begin(),
                out_edges.end(),
                [&](const Node::adjacency& a) {
                    return a.second == vid;
                });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    } else {
        auto& out_edges = m_nodes[vid].edges;
        auto it = std::find_if(
                out_edges.begin(),
                out_edges.end(),
                [&](const Node::adjacency& a) {
                    return a.second == uid;
                });
        if (it != out_edges.end()) {
            erase_edge(it->first);
        }
    }
}

/// Erase an edge. All incident edge and adjacent node iterators for the source
/// and target nodes, edge iterators pointing to edges after the erased edge,
/// and edge ids greater than the erased edge id are invalidated.
void ExperienceGraph::erase_edge(edge_id id)
{
    if (id >= m_edges.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid edge id");
    }

    const Edge& e = m_edges[id];

    // remove incident edge from source node and update edge ids
    auto ait = std::remove_if(
            m_nodes[e.snode].edges.begin(),
            m_nodes[e.snode].edges.end(),
            [&](Node::adjacency& a) {
                bool rem = a.first == id;
                if (a.first > id) {
                    --id;
                }
                return rem;
            });
    m_nodes[e.snode].edges.erase(ait, m_nodes[e.snode].edges.end());

    // for non-self-loops, remove incident edge from target node and update edge
    // ids
    if (e.tnode != e.snode) {
        ait = std::remove_if(
                m_nodes[e.tnode].edges.begin(),
                m_nodes[e.tnode].edges.end(),
                [&](Node::adjacency& a) {
                    bool rem = a.first == id;
                    if (a.first > id) {
                        --id;
                    }
                    return rem;
                });
        m_nodes[e.tnode].edges.erase(ait, m_nodes[e.tnode].edges.end());
    }

    // remove the edge
    m_edges.erase(m_edges.begin() + id);
}

inline
void ExperienceGraph::insert_incident_edge(edge_id eid, node_id uid, node_id vid)
{
    if (vid != uid) {
        m_nodes[uid].edges.emplace_back(eid, vid);
        m_nodes[vid].edges.emplace_back(eid, uid);
    } else {
        m_nodes[uid].edges.emplace_back(eid, vid);
    }
}

} // namespace motion
} // namespace sbpl
