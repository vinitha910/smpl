#ifndef SMPL_EXPERIENCE_GRAPH_H
#define SMPL_EXPERIENCE_GRAPH_H

namespace sbpl {
namespace motion {

class ExperienceGraph
{
public:

    // if the experience graph comes as just a collection of states
    //   discrete_states = unique(discretize(vertices))
    //   for all (s, t) in discrete_state x discrete_states
    //   if t in U(s):
    //     eg.addEdge(s, t)
    // else if the experience graph comes as a collection of vertices and edges
    //   for each path
    //     pdp = discretize(p0) // pivot discrete point
    //     edge = []
    //     for each point p in path \ p0
    //       dp = disretize(p)
    //       if dp != pdp
    //         eg.addEdge(pdp, edge)
    //         pdp = dp
    //       else
    //         edge = edge + [p]
    //
    // everytime a point is discretized

    ExperienceGraph(const RobotPlanningSpacePtr& pspace)
    {
    }

    void insertState();
    void eraseState();
    void insertEdge();
    void eraseEdge();

    size_t numNodes() const { return m_nodes.size(); }

    typedef std::vector<Node>::iterator node_iterator;
    typedef std::vector<Node>::const_iterator const_node_iterator;

    typedef std::vector<Edge>::iterator edge_iterator;
    typedef std::vector<Edge>::const_iterator const_edge_iterator;

    node_iterator nodes_begin() { return m_nodes.begin(); }
    node_iterator nodes_end() const { return m_nodes.end(); }

    edge_iterator edges_begin() { return m_edges.begin(); }
    edge_iterator edges_end() { return m_edges.end(); }

    std::pair<std::vector<Node>::iterator, std::vector<Node>::iterator>
    vertices() { return { m_nodes.begin(), m_nodes.end() }; }

    std::pair<std::vector<Node>::const_iterator, std::vector<Node>::const_iterator>
    vertices() const { return { m_nodes.begin(), m_nodes.end() }; }

    std::pair<std::vector<Edge>::iterator, std::vector<Edge>::iterator>
    edges() { return { m_edges.begin(), m_edges.end() }; }

    std::pair<std::vector<Edge>::const_iterator, std::vector<Edge>::const_iterator>
    edges() const { return { m_edges.begin(), m_edges.end() }; }

private:

    struct Edge;

    struct Node
    {
        RobotState state;
        std::vector<Edge*> edges;
    };

    struct Edge
    {
        std::vector<RobotState> waypoints;
        Node* from;
        Node* to;
    };

    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;
};

} // namespace motion
} // namespace sbpl

#endif
