#ifndef LEVEL_SET_FUNCTIONS_AH
#define LEVEL_SET_FUNCTIONS_AH

namespace LevelsetFunctions
{

    extern float fractionInside(float phi_left, float phi_right);
    extern float fractionInside(float phi_bl, float phi_br, float phi_tl, float phi_tr);

    extern void _cycleArray(float *arr, int size);

    extern float areaFraction(float phi0, float phi1, float phi2);
    extern double areaFraction(double phi0, double phi1, double phi2);

    extern float areaFraction(float phi00, float phi10, float phi01, float phi11);
    extern double areaFraction(double phi00, double phi10, double phi01, double phi11);

    extern float volumeFraction(float phi0, float phi1, float phi2, float phi3);
    extern double volumeFraction(double phi0, double phi1, double phi2, double phi3);

    extern float volumeFraction(float phi000, float phi100,
                                float phi010, float phi110,
                                float phi001, float phi101,
                                float phi011, float phi111);
    extern double volumeFraction(double phi000, double phi100,
                                 double phi010, double phi110,
                                 double phi001, double phi101,
                                 double phi011, double phi111);

    template <class T>
    static T _sortedTriangleFraction(T phi0, T phi1, T phi2)
    {
        return phi0 * phi0 / (2 * (phi0 - phi1) * (phi0 - phi2));
    }

    template <class T>
    static T _sortedTetFraction(T phi0, T phi1, T phi2, T phi3)
    {
        return phi0 * phi0 * phi0 / ((phi0 - phi1) * (phi0 - phi2) * (phi0 - phi3));
    }

    template <class T>
    static T _sortedPrismFraction(T phi0, T phi1, T phi2, T phi3)
    {
        T a = phi0 / (phi0 - phi2);
        T b = phi0 / (phi0 - phi3);
        T c = phi1 / (phi1 - phi3);
        T d = phi1 / (phi1 - phi2);
        return a * b * (1 - d) + b * (1 - c) * d + c * d;
    }

    template <class T>
    void _swap(T &a, T &b)
    {
        T c(a);
        a = b;
        b = c;
    }

    template <class T>
    inline void _sort(T &a, T &b, T &c, T &d)
    {
        if (a > b)
        {
            _swap(a, b);
        }
        if (c > d)
        {
            _swap(c, d);
        }
        if (a > c)
        {
            _swap(a, c);
        }
        if (b > d)
        {
            _swap(b, d);
        }
        if (b > c)
        {
            _swap(b, c);
        }
    }

}

#endif