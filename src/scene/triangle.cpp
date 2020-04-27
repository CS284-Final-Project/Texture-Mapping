#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
    Vector3D AB = p1 - p2;
    Vector3D AC = p1 - p3;
    Vector3D N = cross(AB, AC);
    N.normalize();
    double t = dot((p1 - r.o), N)/dot(r.d, N);
    if ((t>r.min_t) && (t<r.max_t)&& (t>0.0)){
        Vector3D p = r.o+ r.d * t;
        double alpha = (cross(p-p2, p-p3).norm())/(cross(AB,AC).norm());
        double beta = (cross(p-p3, p-p1).norm())/(cross(AB,AC).norm());
        //double gamma = 1.0 - alpha - beta;
        double gamma = (cross(p-p2, p-p1).norm())/(cross(AB,AC).norm());
        //if (alpha>=0 && beta>=0 && gamma>=0){
        if (abs(1-(alpha+beta+gamma))<0.00001){
            r.max_t = t;
            return true;
        }
    }
   return false;
  
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    //return true;
    Vector3D AB = p1 - p2;
    Vector3D AC = p1 - p3;
    Vector3D N = cross(AB, AC);
    N.normalize();
    double t = dot((p1 - r.o), N)/dot(r.d, N);
    if ((t>r.min_t) && (t<r.max_t) && (t>0.0)){
        Vector3D p = r.o+ r.d*t;
        double alpha = (cross(p-p2, p-p3).norm())/(cross(AB,AC).norm());
        double beta = (cross(p-p3, p-p1).norm())/(cross(AB,AC).norm());
        double gamma = (cross(p-p2, p-p1).norm())/(cross(AB,AC).norm());
        //if (alpha>=0 && beta>=0 && gamma>=0){
        if (abs(1-(alpha+beta+gamma))<0.00001){
            //cout<< alpha<<" "<<beta<<" "<<gamma<<" "<<endl;
            isect->t = t;
            Vector3D n_ = alpha*n1 + (beta*n2) + (gamma*n3);
            n_.normalize();
            isect->n = n_;
            isect->primitive = this;
            isect->bsdf = get_bsdf();
            r.max_t = t;
            return true;
        }
    }
    return false;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
