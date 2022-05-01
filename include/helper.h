//
// Created by redwan on 4/30/22.
//

#ifndef MPCHRVO_HELPER_H
#define MPCHRVO_HELPER_H

#include <iostream>
#include <vector>
using namespace std;



struct TRAJECTORY{
    vector<float>x, y, z;


    size_t size()
    {
        return x.size();
    }

    void clear()
    {
        x.clear();
        y.clear();
        z.clear();
    }

    void copy(TRAJECTORY& other)
    {
        other.clear();
        std::copy(x.begin(), x.end(), back_inserter(other.x));
        std::copy(y.begin(), y.end(), back_inserter(other.y));
    }

    friend ostream &operator<<(ostream &os, const TRAJECTORY &traj) {
        for (int i = 0; i < traj.z.size(); ++i) {
            os << "z[" << i << "] : " << traj.z[i] << "\n";
        }

        return os;
    }
};
#endif //MPCHRVO_HELPER_H
