#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

        
    double xmin = (min[0] - r.o[0]) / r.d[0];
    double xmax = (max[0] - r.o[0]) / r.d[0];
    double ymin = (min[1] - r.o[1]) / r.d[1];
    double ymax = (max[1] - r.o[1]) / r.d[1];
    double zmin = (min[2] - r.o[2]) / r.d[2];
    double zmax = (max[2] - r.o[2]) / r.d[2];
    double xmint = std::min(xmin, xmax);
    double xmaxt = std::max(xmin, xmax);
    double ymint = std::min(ymin, ymax);
    double ymaxt = std::max(ymin, ymax);
    double zmint = std::min(zmin, zmax);
    double zmaxt = std::max(zmin, zmax);
    double mint = std::max(std::max(xmint, ymint), zmint);
    double maxt = std::min(std::min(xmaxt, ymaxt), zmaxt);
        
    if ((mint <= maxt )&& (std::max(mint, r.min_t) <= std::min(maxt, r.max_t))){
        t0 = mint;
        t1 = maxt;
        return true;
    }
        
    return false;
    /*
    double minx = std::min(min.x, max.x);
    double miny = std::min(min.y, max.y);
    double minz = std::min(min.z, max.z);
    double maxx = std::max(min.x, max.x);
    double maxy = std::max(min.y, max.y);
    double maxz = std::max(min.z, max.z);
    //double mint = std::max(std::min(r.min_t, r.max_t),std::min(t0,t1));
    //double maxt = std::min(std::max(r.min_t, r.max_t),std::max(t0,t1));
    double mint = std::min(r.min_t, r.max_t);
    double maxt = std::max(r.min_t, r.max_t);
    Vector3D Nx(1.0, 0.0, 0.0);
    double t = dot(min - r.o, Nx)/dot(r.d, Nx);
    if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        if ((miny<p.y) && (p.y<maxy) && (minz<p.z) && (p.z<maxz) ){
            return true;
        }
    }
    t = dot(max - r.o, Nx)/dot(r.d, Nx);
    if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        
        if ((miny<p.y) && (p.y<maxy) && (minz<p.z) && (p.z<maxz)){
            return true;
//            if (t0_temp == -1){
//                t0_temp = t;
//            }
//            else{
//                t1_temp = t;
//            }
        }
    }
    // intersect with y planes
    Vector3D Ny(0.0, 1.0, 0.0);
    t = dot(min - r.o, Ny)/dot(r.d, Ny);
    if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        if ((minx<p.x) && (p.x<maxx) && (minz<p.z) && (p.z<maxz)){
            return true;
//            if (t0_temp == -1){
//                t0_temp = t;
//            }
//            else{
//                t1_temp = t;
//            }
        }
    }
    t = dot(max - r.o, Ny)/dot(r.d, Ny);
   if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        //double t0_temp = -1;
        if ((minx<p.x) && (p.x<maxx) && (minz<p.z) && (p.z<maxz)){
            return true;
//            if (t0_temp == -1){
//                t0_temp = t;
//            }
//            else{
//                t1_temp = t;
//            }
        }
    }
    // intersect with x planes
    Vector3D Nz(0.0, 0.0, 1.0);
    t = dot(min - r.o, Nz)/dot(r.d, Nz);
    if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        
        if ((miny<p.y) && (p.y<maxy) && (minx<p.x) && (p.x<maxx)){
            return true;
//            if (t0_temp == -1){
//                t0_temp = t;
//            }
//            else{
//                t1_temp = t;
//            }
        }
    }
    t = dot(max - r.o, Nz)/dot(r.d, Nz);
    if (t>mint && t<maxt){
        Vector3D p = r.o + t*r.d;
        //double t0_temp = -1;
        if ((miny<p.y) && (p.y<maxy) && (minx<p.x) && (p.x<maxx)){
            return true;
//            if (t0_temp == -1){
//                t0_temp = t;
//            }
//            else{
//                t1_temp = t;
//            }
        }
    }
    return false;
     */
     
    
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
