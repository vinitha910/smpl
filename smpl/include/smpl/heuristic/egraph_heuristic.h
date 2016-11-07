#ifndef SMPL_EGRAPH_HEURISTIC_H
#define SMPL_EGRAPH_HEURISTIC_H

namespace sbpl {
namespace motion {

class ExperienceGraphHeuristicExtension : public virtual Extension
{
public:

    virtual void GetExperienceGraphStatesWithSameHeuristic(
        int state_id,
        std::vector<int>& ids) const = 0;

    virtual void GetShortcutSuccs(
        int state_id,
        std::vector<int>& shortcut_ids) = 0;

private:
};

} // namespace motion
} // namespace sbpl

#endif
