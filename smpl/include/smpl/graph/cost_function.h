#ifndef sbpl_manip_cost_function_h
#define sbpl_manip_cost_function_h

namespace sbpl {
namespace motion {

class CostFunction
{
public:

    CostFunction();
    virtual ~CostFunction() { }

    virtual int getCost(const RobotState& state, const RobotState& state) const = 0;

private:
};

} // namespace motion
} // namespace sbpl
#endif
