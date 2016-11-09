#include <smpl/graph/experience_graph.h>

#include <algorithm>
#include <stdexcept>

namespace sbpl {
namespace motion {

ExperienceGraph::ExperienceGraph(/*const RobotPlanningSpacePtr& pspace*/)
{
}

std::pair<ExperienceGraph::out_edge_iterator, ExperienceGraph::out_edge_iterator>
ExperienceGraph::out_edges(node_id id)
{
    return std::make_pair(m_nodes[id].edges.begin(), m_nodes[id].edges.end());
}

ExperienceGraph::node_id ExperienceGraph::source(edge_id id) const
{
    return m_edges[id].snode;
}

ExperienceGraph::node_id ExperienceGraph::target(edge_id id) const
{
    return m_edges[id].tnode;
}

ExperienceGraph::degree_size_type
ExperienceGraph::out_degree(node_id id) const
{
    return m_nodes[id].edges.size();
}

std::pair<
    ExperienceGraph::in_edge_iterator,
    ExperienceGraph::in_edge_iterator>
ExperienceGraph::in_edges(node_id id)
{
    return std::make_pair(m_nodes[id].edges.begin(), m_nodes[id].edges.end());
}

ExperienceGraph::degree_size_type ExperienceGraph::in_degree(node_id id)
{
    return m_nodes[id].edges.size();
}

ExperienceGraph::degree_size_type ExperienceGraph::degree(node_id id)
{
    return m_nodes[id].edges.size();
}

std::pair<
    ExperienceGraph::adjacency_iterator,
    ExperienceGraph::adjacency_iterator>
ExperienceGraph::adjacent_nodes(node_id id)
{
    return std::make_pair(
            adjacency_iterator(m_edges, m_nodes[id].edges.begin()),
            adjacency_iterator(m_edges, m_nodes[id].edges.end()));
}

std::pair<
    ExperienceGraph::node_iterator,
    ExperienceGraph::node_iterator>
ExperienceGraph::nodes()
{
    return std::make_pair(m_nodes.begin(), m_nodes.end());
}

std::pair<
    ExperienceGraph::const_node_iterator,
    ExperienceGraph::const_node_iterator>
ExperienceGraph::nodes() const
{
    return std::make_pair(m_nodes.begin(), m_nodes.end());
}

std::pair<
    ExperienceGraph::edge_iterator,
    ExperienceGraph::edge_iterator>
ExperienceGraph::edges()
{
    return std::make_pair(m_edges.begin(), m_edges.end());
}

std::pair<
    ExperienceGraph::const_edge_iterator,
    ExperienceGraph::const_edge_iterator>
ExperienceGraph::edges() const
{
    return std::make_pair(m_edges.begin(), m_edges.end());
}

ExperienceGraph::node_id ExperienceGraph::insert_node()
{
    m_nodes.emplace_back();
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
                [&](edge_id eid) { return will_remove(m_edges[eid]); });
        node.edges.resize(std::distance(node.edges.begin(), it));

        // update edge ids of remaining adjacency edges
        for (edge_id& eid : node.edges) {
            eid -= rem_edge_count;
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
    m_nodes[uid].edges.emplace_back(eid);
    m_nodes[vid].edges.emplace_back(eid);
    return eid;
}

void ExperienceGraph::erase_edge(node_id uid, node_id vid)
{
    if (uid >= m_nodes.size() || vid >= m_nodes.size()) {
        throw std::out_of_range("ExperienceGraph::erase_edge called with invalid node ids");
    }

    // search through the smaller of the two adjacency lists for the edge id
    auto& out_edges = m_nodes[uid].edges.size() < m_nodes[vid].edges.size() ?
            m_nodes[uid].edges : m_nodes[vid].edges;

    for (const edge_id eid : out_edges) {
        if (m_edges[eid].snode == uid || m_edges[eid].snode == vid) {
            // TODO: remove edge
        }
    }
}

void ExperienceGraph::erase_edge(edge_id id)
{

}

ExperienceGraph::node_id ExperienceGraph::get_id(const_node_iterator it) const
{
    return (node_id)std::distance(m_nodes.begin(), it);
}

ExperienceGraph::edge_id ExperienceGraph::get_id(const_edge_iterator it) const
{
    return (edge_id)std::distance(m_edges.begin(), it);
}

} // namespace motion
} // namespace sbpl
