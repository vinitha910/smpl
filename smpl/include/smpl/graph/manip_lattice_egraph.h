#ifndef SMPL_MANIP_LATTICE_EGRAPH_H
#define SMPL_MANIP_LATTICE_EGRAPH_H

namespace sbpl {
namespace motion {

class ExperienceGraph;

class ExperienceGraphExtension : public virtual Extension
{
public:

    virtual bool LoadExperienceGraph(const std::string& path) = 0;

    virtual void GetShortcutSuccPath(
        int state_id,
        std::vector<int>& succ_ids,
        std::vector<int>& costs) = 0;

    virtual void GetSnapSuccs(
        int state_id,
        std::vector<int>& succ_ids,
        std::vector<int>& costs) = 0;

    virtual bool IsOnExperienceGraph(int state_id) = 0;

    virtual const ExperienceGraph* GetExperienceGraph() const = 0;
    virtual ExperienceGraph* GetExperienceGraph() = 0;
};

class ManipLatticeEgraph : public ManipLattice, public ExperienceGraphExtension
{
public:

private:
};

} // namespace motion
} // namespace sbpl

#endif
