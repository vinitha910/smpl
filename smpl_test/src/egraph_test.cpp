// standard includes
#include <algorithm>
#include <iostream>

// system includes
#include <smpl/graph/experience_graph.h>

namespace smpl = sbpl::motion;

int main(int argc, char* argv[])
{
    smpl::ExperienceGraph eg;

    auto u = eg.insert_node();
    std::cout << "inserted node " << u << std::endl;

    auto v = eg.insert_node();
    std::cout << "inserted node " << v << std::endl;

    auto uv = eg.insert_edge(u, v);
    std::cout << "inserted edge " << uv << std::endl;

    auto nits = eg.nodes();
    for (auto it = nits.first; it != nits.second; ++it) {
        std::cout << it->edges.size() << " edges" << std::endl;
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

    return 0;
}
