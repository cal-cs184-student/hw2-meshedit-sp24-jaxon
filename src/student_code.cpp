#include "student_code.h"
#include "CGL/vector3D.h"
#include "halfEdgeMesh.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (class member).
 *
 * @param points A vector of points in 2D
 * @return A vector containing intermediate points or the final interpolated
 * vector
 */
std::vector<Vector2D>
BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
  // TODO Part 1.
  // Claim: use chatgpt here, I write down the total logic and the loop frame,
  // it helps me fullfill. I leaned how push_back works and know the calculating
  // process
  vector<Vector2D> midPoints;
  size_t v_len = points.size() - 1;

  for (size_t i = 0; i < v_len; ++i) {
    Vector2D p_i = points[i];
    Vector2D p_next = points[i + 1];
    Vector2D p_mid = (1 - t) * p_i + t * p_next;

    midPoints.push_back(p_mid);
  }

  return midPoints;

  /*return std::vector<Vector2D>();*/
}

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (function parameter).
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return A vector containing intermediate points or the final interpolated
 * vector
 */
std::vector<Vector3D>
BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const {
  // TODO Part 2.
  vector<Vector3D> midPoints;
  size_t v_len = points.size() - 1;

  for (size_t i = 0; i < v_len; ++i) {
    Vector3D p_i = points[i];
    Vector3D p_next = points[i + 1];
    Vector3D p_mid = (1 - t) * p_i + t * p_next;

    midPoints.push_back(p_mid);
  }

  return midPoints;
}

/**
 * Fully evaluates de Casteljau's algorithm for a vector of points at scalar
 * parameter t
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points,
                                 double t) const {
  // TODO Part 2.
  std::vector<Vector3D> tempPoints = points;
  while (tempPoints.size() > 1) {
    tempPoints = evaluateStep(tempPoints, t);
  }
  return tempPoints[0];
}

/**
 * Evaluates the Bezier patch at parameter (u, v)
 *
 * @param u         Scalar interpolation parameter
 * @param v         Scalar interpolation parameter (along the other axis)
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate(double u, double v) const {
  // TODO Part 2.
  // Claim: use chatgpt here to fullill the condition of loop const auto& row :
  // controlPoints This written style is new to me, so I use it to substitute my
  // original condition
  vector<Vector3D> rowPoints;
  for (const auto &row : controlPoints) {
    rowPoints.push_back(evaluate1D(row, u));
  }

  return evaluate1D(rowPoints, v);
}

Vector3D Vertex::normal(void) const {
  // TODO Part 3.
  // Returns an approximate unit normal at this vertex, computed by
  // taking the area-weighted average of the normals of neighboring
  // triangles, then normalizing.
  // claim: used chatgpt here, I ask it what funtion can use in the face amd
  // edge amd vertex class to calculate the poisition and norm After that, I
  // review the  halfEdgeMesh.h

  Vector3D N(0, 0, 0);

  HalfedgeCIter h = halfedge();
  double totalArea = 0.0;
  do {
    if (!h->face()->isBoundary()) {

      Vector3D p0 = h->vertex()->position;
      Vector3D p1 = h->next()->vertex()->position;
      Vector3D p2 = h->next()->next()->vertex()->position;

      Vector3D v1 = p1 - p0;
      Vector3D v2 = p2 - p0;

      Vector3D faceNormal = cross(v1, v2);

      double area = faceNormal.norm() / 2.0;
      totalArea += area;

      N += area * faceNormal;
    }
    h = h->twin()->next();
  } while (h != halfedge());

  Vector3D averageNormal = N / totalArea;
  return averageNormal.unit();
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO Part 4.
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

  // shape:
  //    c           c
  // a  |  d ->  a-----d
  //    b           b

  if (!e0->isBoundary()) {
    HalfedgeIter h_bc = e0->halfedge();
    HalfedgeIter h_ca = h_bc->next();
    HalfedgeIter h_ab = h_ca->next();
    HalfedgeIter h_cb = h_bc->twin();
    HalfedgeIter h_bd = h_cb->next();
    HalfedgeIter h_dc = h_bd->next();

    VertexIter v_a = h_ab->vertex();
    VertexIter v_b = h_bc->vertex();
    VertexIter v_c = h_cb->vertex();
    VertexIter v_d = h_dc->vertex();

    FaceIter f_abc = h_bc->face();
    FaceIter f_bdc = h_cb->face();

    // begin edge flip
    h_bc->setNeighbors(h_dc, h_cb, v_a, e0, f_abc);
    h_cb->setNeighbors(h_ab, h_bc, v_d, e0, f_bdc);
    h_dc->setNeighbors(h_ca, h_dc->twin(), v_d, h_dc->edge(), f_abc);
    h_ca->setNeighbors(h_bc, h_ca->twin(), v_c, h_ca->edge(), f_abc);
    h_ab->setNeighbors(h_bd, h_ab->twin(), v_a, h_ab->edge(), f_bdc);
    h_bd->setNeighbors(h_cb, h_bd->twin(), v_b, h_bd->edge(), f_bdc);

    v_a->halfedge() = h_bc;
    v_b->halfedge() = h_bd;

    f_abc->halfedge() = h_bc;
    f_bdc->halfedge() = h_cb;
  }

  return e0;
}

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO Part 5.
  // This method should split the given edge and return an iterator to the newly
  // inserted vertex. The halfedge of this vertex should point along the edge
  // that was split, rather than the new edges.
  if (!e0->isBoundary()) {

    HalfedgeIter h_bc = e0->halfedge();
    HalfedgeIter h_ca = h_bc->next();
    HalfedgeIter h_ab = h_ca->next();
    HalfedgeIter h_cb = h_bc->twin();
    HalfedgeIter h_bd = h_cb->next();
    HalfedgeIter h_dc = h_bd->next();

    VertexIter v_a = h_ab->vertex();
    VertexIter v_b = h_bc->vertex();
    VertexIter v_c = h_cb->vertex();
    VertexIter v_d = h_dc->vertex();

    FaceIter f_abc = h_ab->face();
    FaceIter f_bdc = h_cb->face();

    // new element

    HalfedgeIter h_am = newHalfedge();
    HalfedgeIter h_ma = newHalfedge();
    HalfedgeIter h_mc = newHalfedge();
    HalfedgeIter h_cm = newHalfedge();
    HalfedgeIter h_md = newHalfedge();
    HalfedgeIter h_dm = newHalfedge();

    EdgeIter e_mc = newEdge();
    EdgeIter e_am = newEdge();
    EdgeIter e_md = newEdge();

    FaceIter f_amc = newFace();
    FaceIter f_cmd = newFace();

    VertexIter v_m = newVertex();

    // begin edge split
    v_m->position = (v_b->position + v_c->position) * 0.5;
    v_m->halfedge() = h_mc;
    v_m->isNew = true;

    h_bc->setNeighbors(h_ma, h_cm, v_b, e0, f_abc);
    h_cb->setNeighbors(h_md, h_mc, v_c, e_mc, f_bdc);
    h_dc->setNeighbors(h_cb, h_dc->twin(), v_d, h_dc->edge(), f_bdc);
    h_ca->setNeighbors(h_am, h_ca->twin(), v_c, h_ca->edge(), f_amc);
    h_ab->setNeighbors(h_bc, h_ab->twin(), v_a, h_ab->edge(), f_abc);
    h_bd->setNeighbors(h_dm, h_bd->twin(), v_b, h_bd->edge(), f_cmd);

    h_am->setNeighbors(h_mc, h_ma, v_a, e_am, f_amc);
    h_ma->setNeighbors(h_ab, h_am, v_m, e_am, f_abc);
    h_mc->setNeighbors(h_ca, h_cb, v_m, e_mc, f_amc);
    h_cm->setNeighbors(h_bd, h_bc, v_m, e0, f_cmd);
    h_md->setNeighbors(h_dc, h_dm, v_m, e_md, f_bdc);
    h_dm->setNeighbors(h_cm, h_md, v_d, e_md, f_cmd);

    f_abc->halfedge() = h_bc;
    f_amc->halfedge() = h_mc;
    f_cmd->halfedge() = h_cm;
    f_bdc->halfedge() = h_cb;

    e_mc->halfedge() = h_mc;
    e_am->halfedge() = h_am;
    e_md->halfedge() = h_md;

    e0->isNew = false;
    e_mc->isNew = false;
    e_am->isNew = true;
    e_md->isNew = true;

    return v_m;
    //   HalfedgeIter h_bc = e0->halfedge();
    //   HalfedgeIter h_ca = h_bc->next();
    //   HalfedgeIter h_ab = h_ca->next();
    //   HalfedgeIter h_cb = h_bc->twin();
    //   HalfedgeIter h_bd = h_cb->next();
    //   HalfedgeIter h_dc = h_bd->next();

    //   // HalfedgeIter h_ba = h_ab->twin();
    //   // HalfedgeIter h_ac = h_ca->twin();
    //   // HalfedgeIter h_db = h_bd->twin();
    //   // HalfedgeIter h_cd = h_dc->twin();

    //   VertexIter v_a = h_ab->vertex();
    //   VertexIter v_b = h_bc->vertex();
    //   VertexIter v_c = h_ca->vertex();
    //   VertexIter v_d = h_dc->vertex();

    //   EdgeIter e_ab = h_ab->edge();
    //   EdgeIter e_bc = h_bc->edge();
    //   EdgeIter e_ca = h_ca->edge();
    //   EdgeIter e_bd = h_bd->edge();
    //   EdgeIter e_dc = h_dc->edge();

    //   FaceIter f_abc = h_ab->face();
    //   FaceIter f_bdc = h_cb->face();

    //   // new elements
    //   HalfedgeIter h_am = newHalfedge();
    //   HalfedgeIter h_ma = newHalfedge();
    //   HalfedgeIter h_mc = newHalfedge();
    //   HalfedgeIter h_cm = newHalfedge();
    //   HalfedgeIter h_md = newHalfedge();
    //   HalfedgeIter h_dm = newHalfedge();

    //   VertexIter v_m = newVertex();

    //   EdgeIter e_am = newEdge();
    //   EdgeIter e_mc = newEdge();
    //   EdgeIter e_md = newEdge();

    //   FaceIter f_amc = newFace();
    //   FaceIter f_cmd = newFace();

    //   // begin edge split
    //   v_m->isNew = true;
    //   v_m->position = (v_c->position + v_b->position) * 0.5;

    //   h_ab->setNeighbors(h_ab->next(), h_ab->twin(), h_ab->vertex(),
    //   h_ab->edge(),
    //                      f_abc);
    //   h_bc->setNeighbors(h_ma, h_bc->twin(), h_bc->vertex(), h_bc->edge(),
    //   f_abc); h_ca->setNeighbors(h_am, h_ca->twin(), h_ca->vertex(),
    //   h_ca->edge(), f_amc); h_cb->setNeighbors(h_cb->next(), h_cb->twin(),
    //   v_m, h_cb->edge(), f_bdc); h_bd->setNeighbors(h_dm, h_bd->twin(),
    //   h_bd->vertex(), h_bd->edge(), f_bdc); h_dc->setNeighbors(h_cm,
    //   h_dc->twin(), h_dc->vertex(), h_dc->edge(), f_cmd);

    //   h_am->setNeighbors(h_mc, h_ma, v_a, e_am, f_amc);
    //   h_ma->setNeighbors(h_ab, h_am, v_m, e_am, f_abc);
    //   h_cm->setNeighbors(h_md, h_mc, v_c, e_mc, f_cmd);
    //   h_mc->setNeighbors(h_ca, h_cm, v_m, e_mc, f_amc);
    //   h_md->setNeighbors(h_dc, h_dm, v_m, e_md, f_cmd);
    //   h_dm->setNeighbors(h_cb, h_md, v_d, e_md, f_bdc);

    //   f_abc->halfedge() = h_bc;
    //   f_bdc->halfedge() = h_cb;
    //   f_amc->halfedge() = h_mc;
    //   f_cmd->halfedge() = h_cm;

    //   e_am->halfedge() = h_am;
    //   e_am->isNew = true;
    //   e_mc->halfedge() = h_mc;
    //   e_mc->isNew = false;
    //   e_md->halfedge() = h_md;
    //   e_md->isNew = true;

    //   return v_m;
  } else {

    HalfedgeIter h_bc = e0->halfedge();
    HalfedgeIter h_cb = h_bc->twin();
    // no abc, only bdc
    if (h_bc->isBoundary()) {

      // HalfedgeIter h_bd = h_cb->next();
      // HalfedgeIter h_dc = h_bd->next();

      // VertexIter v_b = h_bc->vertex();
      // VertexIter v_c = h_cb->vertex();
      // VertexIter v_d = h_dc->vertex();

      // FaceIter f_abc = h_bc->face();
      // FaceIter f_bdc = h_cb->face();

      //  // new element
      //   HalfedgeIter h_mc = newHalfedge();

      //   HalfedgeIter h_cm = newHalfedge();
      //   HalfedgeIter h_md = newHalfedge();
      //   HalfedgeIter h_dm = newHalfedge();

      //   EdgeIter e_mc = newEdge();

      //   EdgeIter e_md = newEdge();

      //   FaceIter f_cmd = newFace();

      //   VertexIter v_m = newVertex();

      //   // begin edge split
      //   v_m->position = (v_b->position + v_c->position) * 0.5;
      //   v_m->halfedge() = h_mc;
      //   v_m->isNew = true;

      //       h_mc->setNeighbors( h_bc-> , h_cb, v_m, e_mc, f_abc);
      //   h_bc->setNeighbors( h_mc , h_cm, v_b, e0, f_abc);
      //   h_cb->setNeighbors( h_md , h_mc, v_c, e_mc, f_bdc);
      //   h_dc->setNeighbors( h_cb , h_dc->twin(), v_d, h_dc->edge(),f_bdc);

      //   h_bd->setNeighbors( h_dm , h_bd->twin(), v_b, h_bd->edge(),f_cmd);

      //   h_cm->setNeighbors( h_bd , h_bc, v_m, e0, f_cmd);
      //   h_md->setNeighbors( h_dc , h_dm, v_m, e_md, f_bdc);
      //   h_dm->setNeighbors( h_cm , h_md, v_d, e_md, f_cmd);

      //   f_abc->halfedge() = h_bc;

      //   f_cmd->halfedge() = h_cm;
      //   f_bdc->halfedge() = h_cb;

      //   e_mc->halfedge() = h_mc;

      //   e_md->halfedge() = h_md;

      //   e0->isNew = false;
      //   e_mc->isNew = false;

      //   e_md->isNew = true;

      //   return v_m;
    }
    // no bdc, only abc
    if (h_cb->isBoundary()) {

      //   HalfedgeIter h_ca = h_bc->next();
      //   HalfedgeIter h_ab = h_ca->next();

      //   VertexIter v_a = h_ab->vertex();
      //   VertexIter v_b = h_bc->vertex();
      //   VertexIter v_c = h_cb->vertex();

      //   FaceIter f_abc = h_ab->face();
      //   FaceIter f_bdc = h_cb->face();

      //  // new element
      //   HalfedgeIter h_mc = newHalfedge();
      //   HalfedgeIter h_am = newHalfedge();
      //   HalfedgeIter h_ma = newHalfedge();
      //   HalfedgeIter h_cm = newHalfedge();

      //   EdgeIter e_mc = newEdge();
      //   EdgeIter e_am = newEdge();

      //   FaceIter f_amc = newFace();

      //   VertexIter v_m = newVertex();

      //   // begin edge split
      //   v_m->position = (v_b->position + v_c->position) * 0.5;
      //   v_m->halfedge() = h_mc;
      //   v_m->isNew = true;

      //   h_bc->setNeighbors( h_ma , h_cm, v_b, e0, f_abc);
      //   h_cb->setNeighbors( h_md , h_mc, v_c, e_mc, f_bdc);
      //   h_dc->setNeighbors( h_cb , h_dc->twin(), v_d, h_dc->edge(),f_bdc);
      //   h_ca->setNeighbors( h_am , h_ca->twin(), v_c, h_ca->edge(),f_amc);
      //   h_ab->setNeighbors( h_bc , h_ab->twin(), v_a, h_ab->edge(),f_abc);
      //   h_bd->setNeighbors( h_dm , h_bd->twin(), v_b, h_bd->edge(),f_cmd);

      //   h_mc->setNeighbors( h_ca , h_cb, v_m, e_mc, f_amc);
      //   h_am->setNeighbors( h_mc , h_ma, v_a, e_am, f_amc);
      //   h_ma->setNeighbors( h_ab , h_am, v_m, e_am, f_abc);
      //   h_cm->setNeighbors( h_bd , h_bc, v_m, e0, f_cmd);

      //   f_abc->halfedge() = h_bc;
      //   f_amc->halfedge() = h_mc;

      //   f_bdc->halfedge() = h_cb;

      //   e_mc->halfedge() = h_mc;
      //   e_am->halfedge() = h_am;

      //   e0->isNew = false;
      //   e_mc->isNew = false;
      //   e_am->isNew = true;
      //   e_md->isNew = true;

      //   return v_m;
    }
  }

  return VertexIter();
}

void MeshResampler::upsample(HalfedgeMesh &mesh) {
  // TODO Part 6.
  // This routine should increase the number of triangles in the mesh using Loop
  // subdivision. One possible solution is to break up the method as listed
  // below.

  // 1. Compute new positions for all the vertices in the input mesh, using the
  // Loop subdivision rule, and store them in Vertex::newPosition. At this
  // point, we also want to mark each vertex as being a vertex of the original
  // mesh.

  // 2. Compute the updated vertex positions associated with edges, and store it
  // in Edge::newPosition.

  // 3. Split every edge in the mesh, in any order. For future reference, we're
  // also going to store some information about which subdivide edges come from
  // splitting an edge in the original mesh, and which edges are new, by setting
  // the flat Edge::isNew. Note that in this loop, we only want to iterate over
  // edges of the original mesh---otherwise, we'll end up splitting edges that
  // we just split (and the loop will never end!)

  // 4. Flip any new edge that connects an old and new vertex.

  // 5. Copy the new vertex positions into final Vertex::position.

  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {

    HalfedgeIter h_bc = e->halfedge();
    HalfedgeIter h_ca = h_bc->next();
    HalfedgeIter h_ab = h_ca->next();
    HalfedgeIter h_cb = h_bc->twin();
    HalfedgeIter h_bd = h_cb->next();
    HalfedgeIter h_dc = h_bd->next();

    VertexIter v_a = h_ab->vertex();
    VertexIter v_b = h_bc->vertex();
    VertexIter v_c = h_ca->vertex();
    VertexIter v_d = h_dc->vertex();

    e->newPosition = 3.0 / 8.0 * (v_c->position + v_b->position) +
                     1.0 / 8.0 * (v_a->position + v_d->position);
    e->isNew = false;
  }
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {

    Vector3D neighbor_position_sum = Vector3D();
    HalfedgeIter h = v->halfedge();
    do {
      neighbor_position_sum += h->twin()->vertex()->position;
      h = h->twin()->next();

    } while (h != v->halfedge());
    int n = v->degree();
    float u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));

    v->newPosition = (1.0 - u * n) * v->position + u * neighbor_position_sum;
    v->isNew = false;
  }

  // split edges
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    VertexIter v_b = e->halfedge()->vertex();
    VertexIter v_c = e->halfedge()->twin()->vertex();

    if ((!v_b->isNew) && (!v_c->isNew)) {
      VertexIter v = mesh.splitEdge(e);
      v->newPosition = e->newPosition;
    }
  }
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    VertexIter v_b = e->halfedge()->vertex();
    VertexIter v_c = e->halfedge()->twin()->vertex();

    if ((e->isNew) && ((v_b->isNew == true && v_c->isNew == false) ||
                       (v_b->isNew == false && v_c->isNew == true))) {
      e = mesh.flipEdge(e);
    }
  }
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->position = v->newPosition;
  }
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->isNew = false;
  }
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    e->isNew = false;
  }
}

// debug code

} // namespace CGL
