////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/planning_params.h>

// standard includes
#include <sstream>

// system includes
#include <ros/console.h>

namespace sbpl {
namespace motion {

const std::string PlanningParams::DefaultRobotModelLog = "robot";
const std::string PlanningParams::DefaultGraphLog = "graph";
const std::string PlanningParams::DefaultHeuristicLog = "heuristic";
const std::string PlanningParams::DefaultExpandsLog = "expands";
const std::string PlanningParams::DefaultPostProcessingLog = "post_process";
const std::string PlanningParams::DefaultSolutionLog = "solution";
const std::string PlanningParams::DefaultSuccessorsLog = "successors";

std::string to_string(ShortcutType type)
{
    switch (type) {
    case ShortcutType::INVALID_SHORTCUT_TYPE:
        return "INVALID_SHORTCUT_TYPE";
    case ShortcutType::JOINT_SPACE:
        return "JOINT_SPACE";
    case ShortcutType::JOINT_POSITION_VELOCITY_SPACE:
        return "JOINT_POSITION_VELOCITY_SPACE";
    case ShortcutType::EUCLID_SPACE:
        return "EUCLID_SPACE";
    default:
        return "UNRECOGNIZED_SHORTCUT_TYPE";
    }
}


Parameter::Parameter(const Parameter& o)
{
    copy(o);
}

Parameter& Parameter::operator=(const Parameter& value)
{
    if (this != &value) {
        copy(value);
    }
    return *this;
}

Parameter& Parameter::operator=(bool value)
{
    if (isDynamic()) {
        destroy();
    }
    m_type = Type::Bool;
    m_value.asBool = value;
    return *this;
}

Parameter& Parameter::operator=(int value)
{
    if (isDynamic()) {
        destroy();
    }
    m_type = Type::Int;
    m_value.asInt = value;
    return *this;
}

Parameter& Parameter::operator=(double value)
{
    if (isDynamic()) {
        destroy();
    }
    m_type = Type::Double;
    m_value.asDouble = value;
    return *this;
}

Parameter& Parameter::operator=(const std::string &value)
{
    if (m_type != Type::String) {
        if (isDynamic()) {
            destroy();
        }
        m_type = Type::String;
        m_value.asString = new std::string(value);
    } else {
        *m_value.asString = value;
    }
    return *this;
}

Parameter& Parameter::operator=(const char* value)
{
    if (m_type != Type::String) {
        if (isDynamic()) {
            destroy();
        }
        m_type = Type::String;
        m_value.asString = new std::string(value);
    } else {
        m_value.asString->assign(value);
    }
    return *this;
}

Parameter::~Parameter()
{
    switch (m_type) {
    case Type::String:
        delete m_value.asString;
        break;
    }
}

Parameter::operator bool&()
{
    if (m_type != Type::Bool) {
        throw ParameterException("parameter is not a boolean");
    }
    return m_value.asBool;
}

Parameter::operator int&()
{
    if (m_type != Type::Int) {
        throw ParameterException("parameter is not an int");
    }
    return m_value.asInt;
}

Parameter::operator double&()
{
    if (m_type != Type::Double) {
        throw ParameterException("parameter is not a double");
    }
    return m_value.asDouble;
}

Parameter::operator std::string&()
{
    if (m_type != Type::String) {
        throw ParameterException("parameter is not a string");
    }
    return *m_value.asString;
}

Parameter::operator const bool&() const
{ return const_cast<Parameter*>(this)->operator bool&(); }

Parameter::operator const int&() const
{ return const_cast<Parameter*>(this)->operator int&(); }

Parameter::operator const double&() const
{ return const_cast<Parameter*>(this)->operator double&(); }

Parameter::operator const std::string&() const
{ return const_cast<Parameter*>(this)->operator std::string&(); }

void Parameter::destroy()
{
    switch (m_type) {
    case Type::String:
        if (m_value.asString) {
            delete m_value.asString;
            m_value.asString = nullptr;
        }
        break;
    }
    m_type = Type::Invalid;
}

void Parameter::copy(const Parameter& o)
{
    destroy();

    m_type = o.m_type;
    if (o.m_type == Type::String) {
        m_value.asString = new std::string(*o.m_value.asString);
    } else {
        m_value = o.m_value;
    }
}

PlanningParams::PlanningParams() :
    planning_frame(),

    cost_per_cell(DefaultCostPerCell),

    planning_link_sphere_radius(DefaultPlanningLinkSphereRadius),

    epsilon(DefaultEpsilon),

    shortcut_path(DefaultShortcutPath),
    interpolate_path(DefaultInterpolatePath),
    shortcut_type(DefaultShortcutType),

    print_path(true),
    robot_log(DefaultRobotModelLog),
    graph_log(DefaultGraphLog),
    heuristic_log(DefaultHeuristicLog),
    expands_log(DefaultExpandsLog),
    successors_log(DefaultSuccessorsLog),
    post_processing_log(DefaultPostProcessingLog),
    solution_log(DefaultSolutionLog),

    m_warn_defaults(false)
{
}

template <typename T>
std::string construct_warn_string(const std::string& name, T def)
{
    std::stringstream ss;
    ss << "Missing parameter '" << name << "'. Default set to " << def;
    return ss.str();
}

void PlanningParams::addParam(const std::string& name, bool val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, int val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, double val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, const std::string& val)
{
    params[name] = val;
}

void PlanningParams::param(const std::string& name, bool& val, bool def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        std::stringstream ss;
        val = def;
        if (m_warn_defaults) {
            ROS_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToBool(it->second, val);
}

void PlanningParams::param(const std::string& name, int& val, int def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            ROS_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToInt(it->second, val);
}

void PlanningParams::param(
    const std::string& name,
    double& val,
    double def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            ROS_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToDouble(it->second, val);
}

void PlanningParams::param(
    const std::string& name,
    std::string& val,
    const std::string& def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            ROS_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToString(it->second, val);
}

bool PlanningParams::getParam(const std::string& name, bool& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToBool(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, int& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToInt(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, double& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToDouble(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, std::string& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToString(it->second, val);
    return true;
}

bool PlanningParams::hasParam(const std::string& name) const
{
    auto it = params.find(name);
    return it != params.end();
}

void PlanningParams::convertToBool(const Parameter& p, bool& val) const
{
    switch (p.type()) {
    case Parameter::Bool:
        val = (bool&)p;
        break;
    case Parameter::Int:
        val = (bool)((int&)p);
        break;
    case Parameter::Double:
        val = (bool)((double&)p);
        break;
    case Parameter::String: {
        const std::string& pstr(p);
        if (pstr == "true") {
            val = true;
        } else if (pstr == "false") {
            val = false;
        }
        throw ParameterException("Could not convert parameter string to bool");
    }   break;
    default:
        throw ParameterException("Parameter is untyped");
        break;
    }
}

void PlanningParams::convertToInt(const Parameter& p, int& val) const
{
    switch (p.type()) {
    case Parameter::Bool:
        val = (int)((bool&)p);
        break;
    case Parameter::Int:
        val = ((int&)p);
        break;
    case Parameter::Double:
        val = (int)((double&)p);
        break;
    case Parameter::String:
        try {
            val = std::stoi((std::string&)p);
        } catch (const std::exception& ex) {
            throw ParameterException("Failed to convert string to int");
        }
        break;
    default:
        throw ParameterException("Parameter is untyped");
        break;
    }
}

void PlanningParams::convertToDouble(const Parameter& p, double& val) const
{
    switch (p.type()) {
    case Parameter::Bool:
        val = (double)((bool&)p);
        break;
    case Parameter::Int:
        val = (double)((int&)p);
        break;
    case Parameter::Double:
        val = ((double&)p);
        break;
    case Parameter::String:
        try {
            val = std::stod((std::string&)p);
        } catch (const std::exception& ex) {
            throw ParameterException("Failed to convert string to double");
        }
        break;
    default:
        throw ParameterException("Parameter is untyped");
        break;
    }
}

void PlanningParams::convertToString(const Parameter& p, std::string& val) const
{
    switch (p.type()) {
    case Parameter::Bool:
        val = ((bool&)p) ? "true" : "false";
        break;
    case Parameter::Int:
        val = std::to_string((int&)p);
        break;
    case Parameter::Double:
        val = std::to_string((double&)p);
        break;
    case Parameter::String:
        val = ((std::string&)p);
        break;
    default:
        throw ParameterException("Parameter is untyped");
        break;
    }
}

} // namespace motion
} // namespace sbpl
