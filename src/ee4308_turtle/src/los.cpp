#include "los.hpp"
Index LOS::get_idx_swap(int l, int s) 
{
    return Index(s, l);
}
Index LOS::get_idx_noswap(int l, int s) 
{
    return Index(l, s);
}
Index (*LOS::get_idx)(int l, int s) = nullptr;
/*
/////////////////////////////////////////////////
/////////////// BRESENHAM float /////////////////
/////////////////////////////////////////////////
void LOS::reset(Index src, Index tgt)
{
    int Di = tgt.i - src.i;
    int Dj = tgt.j - src.j;
    if (abs(Di) > abs(Dj))
    {
        Dl = Di;
        Ds = Dj;
        l = src.i;
        s = src.j;
        lf = tgt.i;
        sf = tgt.j;
        get_idx = &get_idx_noswap;
    }
    else
    {
        Dl = Dj;
        Ds = Di;
        l = src.j;
        s = src.i;
        lf = tgt.j;
        sf = tgt.i;
        get_idx = &get_idx_swap;
    }
    ds = sign(Ds); // sign returns double and accepts double, but ds and Ds are int. It does not matter here due to typecasting to the same values.
    dl = sign(Dl);
    ps = ((double) Ds) / abs(Dl); // type cast first operand (Ds) to double to perform correct division
    es = 0;
}
Index LOS::next()
{
    l += dl;
    es += ps;
    if (abs(es) >= 0.5)
    {
        es -= ds;
        s += ds;
    }
    cur = get_idx(l, s);
    return cur;
}
std::vector<Index> LOS::get(Index src, Index tgt)
{
    std::vector<Index> line = {src}; // empty vector of index

    reset(src, tgt);
    
    while (l != lf || s != sf) // DeMorgans rule --> equivalent to !(l == lf && s == sf)
    {
        line.push_back(next());
    }

    return line;
}
*/
/////////////////////////////////////////////////
/////////////// BRESENHAM double /////////////////
/////////////////////////////////////////////////
void LOS::reset(Index src, Index tgt)
{
    int Di = tgt.i - src.i;
    int Dj = tgt.j - src.j;
    if (abs(Di) > abs(Dj))
    {
        Dl = Di;
        Ds = Dj;
        l = src.i;
        s = src.j;
        lf = tgt.i;
        sf = tgt.j;
        get_idx = &get_idx_noswap;
    }
    else
    {
        Dl = Dj;
        Ds = Di;
        l = src.j;
        s = src.i;
        lf = tgt.j;
        sf = tgt.i;
        get_idx = &get_idx_swap;
    }
    ds = sign(Ds); // sign returns double and accepts double, but ds and Ds are int. It does not matter here due to typecasting to the same values.
    dl = sign(Dl);
    abs_Dl = abs(Dl);
    d_esl = abs(Dl) * ds;
    esl = 0;
}
Index LOS::next()
{
    l += dl;
    esl += Ds;
    if (2*abs(esl) >= abs_Dl)
    {
        esl -= d_esl;
        s += ds;
    }
    cur = get_idx(l, s);
    return cur;
}
std::vector<Index> LOS::get(Index src, Index tgt)
{
    std::vector<Index> line = {src}; // empty vector of index

    reset(src, tgt);

    
    while (l != lf || s != sf) // DeMorgans rule --> equivalent to !(l == lf && s == sf)
    {
        line.push_back(next());
    }

    return line;
}