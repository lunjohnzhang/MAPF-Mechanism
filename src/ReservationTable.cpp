#include "ReservationTable.h"


/*int ReservationTable::get_holding_time(int location)
{ 
	auto it = constraints.find(location);
	if (it != constraints.end())
	{
		for (auto constraint : it->second)
			insert_constraint(location, constraint.first, constraint.second);
	}
	
	if (RT.find(location) == RT.end()) 
	{
		return 0;
	}
	int t = std::get<1>(RT[location].back());
	if (t < INTERVAL_MAX)
		return INTERVAL_MAX;
	for (auto p =  RT[location].rbegin(); p != RT[location].rend(); ++p)
	{
		if (t == std::get<1>(*p))
			t = std::get<0>(*p);
		else
			break;
	}
	return t;
}*/


void ReservationTable::insert2SIT(int location, int t_min, int t_max)
{
    assert(t_min >= 0 and t_min < t_max and !sit[location].empty());
    for (auto it = sit[location].begin(); it != sit[location].end();)
    {
        if (t_min >= get<1>(*it))
            ++it;
        else if (t_max <= get<0>(*it))
            break;
        else if (get<0>(*it) < t_min && get<1>(*it) <= t_max)
        {
            (*it) = make_tuple(get<0>(*it), t_min, get<2>(*it));
            ++it;
        }
        else if (t_min <= get<0>(*it) && t_max < get<1>(*it))
        {
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        }
        else if (get<0>(*it) < t_min && t_max < get<1>(*it))
        {
            sit[location].insert(it, make_tuple(get<0>(*it), t_min, get<2>(*it)));
            (*it) = make_tuple(t_max, get<1>(*it), get<2>(*it));
            break;
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            it = sit[location].erase(it);
        }
    }
}

void ReservationTable::insertSoftConstraint2SIT(int location, int t_min, int t_max)
{
    assert(t_min >= 0 && t_min < t_max and !sit[location].empty());
    for (auto it = sit[location].begin(); it != sit[location].end(); ++it)
    {
        if (t_min >= get<1>(*it) || get<2>(*it))
            continue;
        else if (t_max <= get<0>(*it))
            break;

        auto i_min = get<0>(*it);
        auto i_max = get<1>(*it);
        if (i_min < t_min && i_max <= t_max)
        {
            if (it != sit[location].end() and std::next(it) != sit[location].end() and
                i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
            {
                (*it) = make_tuple(i_min, t_min, false);
                ++it;
                (*it) = make_tuple(t_min, get<1>(*it), true);
            }
            else
            {
                sit[location].insert(it, make_tuple(i_min, t_min, false));
                (*it) = make_tuple(t_min, i_max, true);
            }

        }
        else if (t_min <= i_min && t_max < i_max)
        {
            if (it != sit[location].begin() and
                i_min == get<1>(*std::prev(it)) and get<2>(*std::prev(it))) // we can merge the current interval with the previous one
            {
                (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), t_max, true);
            }
            else
            {
                sit[location].insert(it, make_tuple(i_min, t_max, true));
            }
            (*it) = make_tuple(t_max, i_max, false);
        }
        else if (i_min < t_min && t_max < i_max)
        {
            sit[location].insert(it, make_tuple(i_min, t_min, false));
            sit[location].insert(it, make_tuple(t_min, t_max, true));
            (*it) = make_tuple(t_max, i_max, false);
        }
        else // constraint_min <= get<0>(*it) && get<1> <= constraint_max
        {
            if (it != sit[location].begin() and
                i_min == get<1>(*std::prev(it)) and get<2>(*std::prev(it))) // we can merge the current interval with the previous one
            {
                if (it != sit[location].end() and std::next(it) != sit[location].end() and
                    i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
                {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), get<1>(*std::next(it)), true);
                    sit[location].erase(std::next(it));
                    it = sit[location].erase(it);
                }
                else
                {
                    (*std::prev(it)) = make_tuple(get<0>(*std::prev(it)), i_max, true);
                    it = sit[location].erase(it);
                }
                --it;
            }
            else
            {
                if (it != sit[location].end() and std::next(it) != sit[location].end() and
                    i_max == get<0>(*std::next(it)) and get<2>(*std::next(it))) // we can merge the current interval with the next one
                {
                    (*it) = make_tuple(i_min, get<1>(*std::next(it)), true);
                    sit[location].erase(std::next(it));
                }
                else
                {
                    (*it) = make_tuple(i_min, i_max, true);
                }
            }
        }
    }
}

// update SIT at the given location
void ReservationTable::updateSIT(int location)
{
    assert(sit[location].empty());

    // negative constraints
    const auto& it = constraint_table.ct.find(location);
    if (it != constraint_table.ct.end())
    {
        for (auto time_range : it->second)
            insert2SIT(location, time_range.first, time_range.second);
    }

    // positive constraints
    if (location < constraint_table.map_size)
    {
        for (auto landmark : constraint_table.landmarks)
        {
            if (landmark.second != location)
            {
                insert2SIT(location, landmark.first, landmark.first + 1);
            }
        }
    }

    // soft constraints
    if (!constraint_table.cat.empty())
    {
        for (auto t = 0; t < constraint_table.cat[location].size(); t++)
        {
            if (constraint_table.cat[location][t])
                insertSoftConstraint2SIT(location, t, t + 1);
        }
    }
}

// return <upper_bound, low, high,  vertex collision, edge collision>
list<tuple<int, int, int, bool, bool>> ReservationTable::get_safe_intervals(int from, int to, int lower_bound, int upper_bound)
{
    list<tuple<int, int, int, bool, bool>> rst;
    if (lower_bound >= upper_bound)
        return rst;

    if (sit[to].empty())
        updateSIT(to);

    for(auto interval : sit[to])
    {
        if (lower_bound >= get<1>(interval))
            continue;
        else if (upper_bound <= get<0>(interval))
            break;
        // the interval overlaps with [lower_bound, upper_bound)
        auto t1 = get_earliest_arrival_time(from, to,
                                            max(lower_bound, get<0>(interval)), min(upper_bound, get<1>(interval)));
        if (t1 < 0) // the interval is not reachable
            continue;
        else if (get<2>(interval)) // the interval has collisions
        {
            rst.emplace_back(get<1>(interval), t1, get<1>(interval), true, false);
        }
        else // the interval does not have collisions
        { // so we need to check the move action has collisions or not
            auto t2 = get_earliest_no_collision_arrival_time(from, to, interval, t1, upper_bound);
            if (t1 == t2)
                rst.emplace_back(get<1>(interval), t1, get<1>(interval), false, false);
            else if (t2 < 0)
                rst.emplace_back(get<1>(interval), t1, get<1>(interval), false, true);
            else
            {
                rst.emplace_back(get<1>(interval), t1, t2, false, true);
                rst.emplace_back(get<1>(interval), t2, get<1>(interval), false, false);
            }
        }
    }
    return rst;
}

Interval ReservationTable::get_first_safe_interval(size_t location)
{
    if (sit[location].empty())
        updateSIT(location);
    return sit[location].front();
}

// find a safe interval with t_min as given
bool ReservationTable::find_safe_interval(Interval& interval, size_t location, int t_min)
{
    if (t_min >= min(constraint_table.length_max, MAX_TIMESTEP - 1) + 1)
        return false;
    if (sit[location].empty())
        updateSIT(location);
    for( auto & i : sit[location])
    {
        if ((int)get<0>(i) <= t_min && t_min < (int)get<1>(i))
        {
            interval = Interval(t_min, get<1>(i), get<2>(i));
            return true;
        }
        else if (t_min < (int)get<0>(i))
            break;
    }
    return false;
}

int ReservationTable::get_earliest_arrival_time(int from, int to, int lower_bound, int upper_bound) const
{
    for (auto t = lower_bound; t < upper_bound; t++)
    {
        if (!constraint_table.constrained(from, to, t))
            return t;
    }
    return -1;
}
int ReservationTable::get_earliest_no_collision_arrival_time(int from, int to, const Interval& interval,
                                                             int lower_bound, int upper_bound) const
{
    for (auto t = max(lower_bound, get<0>(interval)); t < min(upper_bound, get<1>(interval)); t++)
    {
        if (!constraint_table.hasEdgeConflict(from, to, t))
            return t;
    }
    return -1;
}