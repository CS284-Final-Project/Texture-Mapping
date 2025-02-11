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
	double x1 = (this->min.x - r.o.x) / r.d.x;
	double x2 = (this->max.x - r.o.x) / r.d.x;
	double y1 = (this->min.y - r.o.y) / r.d.y;
	double y2 = (this->max.y - r.o.y) / r.d.y;
	double z1 = (this->min.z - r.o.z) / r.d.z;
	double z2 = (this->max.z - r.o.z) / r.d.z;

	double t_min = std::max(std::min(x1, x2), std::max(std::min(y1, y2), std::min(z1, z2)));
	double t_max = std::min(std::max(x1, x2), std::min(std::max(y1, y2), std::max(z1, z2)));
	if (t_min > t_max)
		return false;
	//Vector3D itsc = r.o + r.d * t0_new;
	/*itsc.x >= BBox::min.x && itsc.y >= BBox::min.y && itsc.z >= BBox::min.z &&
	itsc.x <= BBox::max.x && itsc.y <= BBox::max.y && itsc.z <= BBox::max.z*/
	return true;
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
