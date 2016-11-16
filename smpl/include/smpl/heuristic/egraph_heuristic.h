#ifndef SMPL_EGRAPH_HEURISTIC_H
#define SMPL_EGRAPH_HEURISTIC_H

namespace sbpl {
namespace motion {

class ExperienceGraphHeuristicExtension : public virtual Extension
{
public:

    /// Return the state ids of experience graph states that have the same
    /// heuristic value as the input state
    virtual void getEquivalentStates(
        int state_id,
        std::vector<int>& ids) const = 0;

    /// Return the state ids of experience graph shortcut states available from
    /// the input state
    virtual void getShortcutSuccs(
        int state_id,
        std::vector<int>& shortcut_ids) = 0;

private:
};

} // namespace motion
} // namespace sbpl

#endif
