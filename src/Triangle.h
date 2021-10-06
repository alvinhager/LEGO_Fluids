#ifndef TRIANGLE_AH
#define TRIANGLE_AH

/** Simple triangle struct used to store the indices
 *  to a vertex list that define the positions of the vertices of a triangle */
struct Triangle
{
    // list of indices that are used in a vertex list to get the position in space of the triangle's vertices
    int tri_vertex_indices[3];

    /** empty constructor */
    Triangle()
    {
        tri_vertex_indices[0] = 0;
        tri_vertex_indices[1] = 0;
        tri_vertex_indices[2] = 0;
    }

    /** constructor that builds a triangle by taking in the indices of the triangle's vertices */
    Triangle(int p1, int p2, int p3)
    {
        tri_vertex_indices[0] = p1;
        tri_vertex_indices[1] = p2;
        tri_vertex_indices[2] = p3;
    }
};

#endif