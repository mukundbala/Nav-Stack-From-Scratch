#include <ros/ros.h> // for students to debug
#include <cmath>
#include <vector>
#include "common.hpp"

#ifndef LOS_HPP
#define LOS_HPP
class LOS
{   
    private:
        static Index get_idx_swap(int l, int s);
        static Index get_idx_noswap(int l, int s);
        static Index (*get_idx)(int l, int s);
        Index cur;

        /*
        /////////////////////////////////////////
        /////// BRESENHAM float /////////////////
        /////////////////////////////////////////
        int Di, Dj, Dl, Ds, l, s, ds, dl, lf, sf;
        double es, ps;
        */

        /////////////////////////////////////////
        ///////// BRESENHAM INT /////////////////
        /////////////////////////////////////////
        int Di, Dj, Dl, Ds, l, s, ds, dl, lf, sf,
            abs_Dl, d_esl, esl;

    public:
        std::vector<Index> get(Index src, Index tgt);
        void reset(Index src, Index tgt);
        Index next();
};
#endif