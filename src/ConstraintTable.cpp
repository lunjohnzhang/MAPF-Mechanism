#include "ConstraintTable.h"

int ConstraintTable::getMaxTimestep()
    const  // everything is static after the max timestep
{
    int rst = max(max(ct_max_timestep, cat_max_timestep), length_min);
    if (length_max < MAX_TIMESTEP)
        rst = max(rst, length_max);
    if (!landmarks.empty())
        rst = max(rst, landmarks.rbegin()->first);
    return rst;
}
int ConstraintTable::getLastCollisionTimestep(int location) const
{
    int rst = -1;
    if (!cat.empty())
    {
        for (auto t = cat[location].size() - 1; t > rst; t--)
        {
            if (cat[location][t])
                return t;
        }
    }
    return rst;
}
void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
    insert2CT(getEdgeIndex(from, to), t_min, t_max);
}
void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max)
{
    assert(loc >= 0);
    ct[loc].emplace_back(t_min, t_max);
    if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep)
    {
        ct_max_timestep = t_max;
    }
    else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep)
    {
        ct_max_timestep = t_min;
    }
}
// build the constraint table for the given agent at the give node
void ConstraintTable::insert2CT(const HLNode& node, int agent)
{
    auto curr = &node;
    while (curr->parent != nullptr)
    {
        insert2CT(curr->constraints, agent);
        curr = curr->parent;
    }
}
// add constraints for the given agent
void ConstraintTable::insert2CT(const list<Constraint>& constraints, int agent)
{
    if (constraints.empty())
        return;
    int a, x, y, t;
    constraint_type type;
    tie(a, x, y, t, type) = constraints.front();
    switch (type)
    {
    case constraint_type::POSITIVE_VERTEX:
        assert(constraints.size() == 1);
        if (agent == a)  // this agent has to be at x at timestep t
        {
            insertLandmark(x, t);
        }
        else  // other agents cannot stay at x at timestep t
        {
            insert2CT(x, t, t + 1);
        }
        break;
    case constraint_type::POSITIVE_EDGE:
        assert(constraints.size() == 1);
        if (agent == a)  // this agent has to be at x at timestep t - 1 and be
                         // at y at timestep t
        {
            insertLandmark(x, t - 1);
            insertLandmark(y, t);
        }
        else  // other agents cannot stay at x at timestep t - 1, be at y at
              // timestep t, or traverse edge (y, x) from timesteps t - 1 to t
        {
            insert2CT(x, t - 1, t);
            insert2CT(y, t, t + 1);
            insert2CT(y, x, t, t + 1);
        }
        break;
    case constraint_type::VERTEX:
        if (a == agent)
        {
            for (const auto& constraint : constraints)
            {
                tie(a, x, y, t, type) = constraint;
                insert2CT(x, t, t + 1);
            }
        }
        break;
    case constraint_type::EDGE:
        assert(constraints.size() == 1);
        if (a == agent)
            insert2CT(x, y, t, t + 1);
        break;
    case constraint_type::BARRIER:
        if (a == agent)
        {
            for (auto constraint : constraints)
            {
                tie(a, x, y, t, type) = constraint;
                auto states = decodeBarrier(x, y, t);
                for (const auto& state : states)
                {
                    insert2CT(state.first, state.second, state.second + 1);
                }
            }
        }
        break;
    case constraint_type::RANGE:
        if (a == agent)
        {
            assert(constraints.size() == 1);
            insert2CT(x, y, t + 1);  // the agent cannot stay at x from timestep
                                     // y to timestep t.
        }
        break;

    // Supress warning:
    case constraint_type::LEQLENGTH:
    case constraint_type::GLENGTH:
    case constraint_type::PBS_ORDER:
        break;
    }
}

void ConstraintTable::insert2CT(const Path& path)
{
    int prev_location = path.front().location;
    int prev_timestep = 0;
    for (int timestep = 0; timestep < (int)path.size(); timestep++)
    {
        auto curr_location = path[timestep].location;
        if (prev_location != curr_location)
        {
            insert2CT(prev_location, prev_timestep,
                      timestep);  // add vertex conflict
            insert2CT(curr_location, prev_location, timestep,
                      timestep + 1);  // add edge conflict
            prev_location = curr_location;
            prev_timestep = timestep;
        }
    }
    // We ignore target conflicts
    // insert2CT(path.back().location, (int)path.size() - 1, MAX_TIMESTEP);
    insert2CT(path.back().location, (int)path.size() - 1, (int)path.size());
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
    auto it = landmarks.find(t);
    if (it == landmarks.end())
    {
        landmarks[t] = loc;
    }
    else
        assert(it->second == loc);
}

// build the conflict avoidance table
void ConstraintTable::insert2CAT(int agent, const vector<Path*>& paths)
{
    for (size_t ag = 0; ag < paths.size(); ag++)
    {
        if (ag == agent || paths[ag] == nullptr || paths[ag]->size() == 1)
            continue;
        insert2CAT(*paths[ag]);
    }
}
void ConstraintTable::insert2CAT(const Path& path)
{
    if (cat.empty())
    {
        cat.resize(map_size + 1);  // Add 1 for dummy start loc
    }
    for (auto timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (cat[loc].size() <= timestep)
            cat[loc].resize(timestep + 1, false);
        cat[loc][timestep] = true;
    }
    cat_max_timestep = max(cat_max_timestep, (int)path.size() - 1);
}

// return the location-time pairs on the barrier in an increasing order of their
// timesteps
list<pair<int, int> > ConstraintTable::decodeBarrier(int x, int y, int t) const
{
    list<pair<int, int> > rst;
    int x1 = x / num_col, y1 = x % num_col;
    int x2 = y / num_col, y2 = y % num_col;
    if (x1 == x2)
    {
        if (y1 < y2)
            for (int i = min(y2 - y1, t); i >= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 - i, t - i);
            }
        else
            for (int i = min(y1 - y2, t); i >= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 + i, t - i);
            }
    }
    else  // y1== y2
    {
        if (x1 < x2)
            for (int i = min(x2 - x1, t); i >= 0; i--)
            {
                rst.emplace_back((x2 - i) * num_col + y1, t - i);
            }
        else
            for (int i = min(x1 - x2, t); i >= 0; i--)
            {
                rst.emplace_back((x2 + i) * num_col + y1, t - i);
            }
    }
    return rst;
}

bool ConstraintTable::constrained(size_t loc, int t) const
{
    assert(loc >= 0);
    if (loc == GLOBAL_VAR::dummy_start_loc)
        return false;
    if (loc < map_size)
    {
        const auto& it = landmarks.find(t);
        if (it != landmarks.end() && it->second != loc)
            return true;  // violate the positive vertex constraint
    }

    const auto& it = ct.find(loc);
    if (it == ct.end())
    {
        return false;
    }
    for (const auto& constraint : it->second)
    {
        if (constraint.first <= t && t < constraint.second)
            return true;
    }
    return false;
}
bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc,
                                  int next_t) const
{
    if (curr_loc == GLOBAL_VAR::dummy_start_loc ||
        next_loc == GLOBAL_VAR::dummy_start_loc)
        return false;
    return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
    length_min = other.length_min;
    length_max = other.length_max;
    num_col = other.num_col;
    map_size = other.map_size;
    ct = other.ct;
    ct_max_timestep = other.ct_max_timestep;
    cat = other.cat;
    cat_max_timestep = other.cat_max_timestep;
    landmarks = other.landmarks;
}

int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id,
                                              int next_timestep) const
{
    int rst = 0;
    if (!cat.empty())
    {
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            rst++;
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and
            cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1] and cat[curr_id][next_timestep])
            rst++;
    }
    return rst;
}
bool ConstraintTable::hasConflictForStep(size_t curr_id, size_t next_id,
                                         int next_timestep) const
{
    if (!cat.empty())
    {
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            return true;
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and
            cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1] and cat[curr_id][next_timestep])
            return true;
    }
    return false;
}
bool ConstraintTable::hasEdgeConflict(size_t curr_id, size_t next_id,
                                      int next_timestep) const
{
    assert(curr_id != next_id);
    return !cat.empty() and curr_id != next_id and
           cat[next_id].size() >= next_timestep and
           cat[curr_id].size() > next_timestep and
           cat[next_id][next_timestep - 1] and cat[curr_id][next_timestep];
}
// int ConstraintTable::getFutureNumOfCollisions(int loc, int t) const
// {
//     int rst = 0;
//     if (!cat.empty())
//     {
//         for (auto timestep = t + 1; timestep < cat[loc].size(); timestep++)
//         {
//             rst += (int)cat[loc][timestep];
//         }
//     }
//     return rst;
// }
