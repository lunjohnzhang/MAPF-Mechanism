#pragma once
#include "CBSNode.h"
#include "Conflict.h"
#include "common.h"

class PBSNode : public HLNode
{
public:
    Constraint constraint;         // new constraint
    list<pair<int, Path> > paths;  // new paths
    int cost = 0;                  // sum of costs

    size_t depth = 0;     // depath of this CT node
    size_t makespan = 0;  // makespan over all paths

    uint64_t time_expanded = 0;
    uint64_t time_generated = 0;

    // conflicts in the current paths
    list<shared_ptr<Conflict> > conflicts;
    // The chosen conflict
    shared_ptr<Conflict> conflict;

    PBSNode* parent = nullptr;
    PBSNode* children[2] = {nullptr, nullptr};

    PBSNode() = default;
    PBSNode(PBSNode& parent)
        : cost(parent.cost),
          depth(parent.depth + 1),
          makespan(parent.makespan),
          conflicts(parent.conflicts),
          parent(&parent)
    {
    }
    void clear();
    void printConstraints(int id) const;
    inline int getNumNewPaths() const override { return (int)paths.size(); }
    inline string getName() const override { return "PBS Node"; }
    list<int> getReplannedAgents() const override
    {
        list<int> rst;
        for (const auto& path : paths) rst.push_back(path.first);
        return rst;
    }

    // Remove virtual function implementation warning. Should never be used.
    inline int getFHatVal() const override { return -1; }
};

std::ostream& operator<<(std::ostream& os, const PBSNode& node);
