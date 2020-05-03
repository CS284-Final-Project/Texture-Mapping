#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
	double a = dot(r.d, r.d);
	double b = 2 * dot((r.o - this->o), r.d);
	double c = dot((r.o - this->o), (r.o - this->o)) - this->r2;
	double tmp = b * b - 4 * a*c;
	if (tmp < 0)
		return false;
	double tmp2 = (-b + sqrt(tmp)) / (2 * a);
	if (tmp2 < 0)
		return false;
	double tmp1 = (-b - sqrt(tmp)) / (2 * a);
	if (tmp1 <= EPS_D && tmp1 >= -EPS_D) {
		if (tmp2 > r.max_t || tmp2 < r.min_t)
			return false;
		t1 = 0; t2 = tmp2;
		return true;
	}
	else {
		if (tmp1 > r.max_t || tmp1 < r.min_t)
			return false;
		t1 = tmp1; t2 = tmp2;
		return true;
	}
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	/*double a = dot(r.d, r.d);
	double b = 2 * dot((r.o - this->o), r.d);
	double c = dot((r.o - this->o), (r.o - this->o)) - this->r2;
	double tmp = b * b - 4 * a*c;
	if (tmp < 0)
		return false;
	double t2 = (-b + sqrt(tmp)) / (2 * a);
	if (t2 < 0)
		return false;
	double t1 = (-b - sqrt(tmp)) / (2 * a);
	t1 = (t1 < 0) ? t2 : t1;
	if (t1 > r.max_t || t1 < r.min_t)
		return false;
  return true;*/
	double t1;
	double t2;
	return Sphere::test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	/*double a = dot(r.d, r.d);
	double b = 2 * dot((r.o - this->o), r.d);
	double c = dot((r.o - this->o), (r.o - this->o)) - this->r2;
	double tmp = b * b - 4 * a*c;
	if (tmp < 0)
		return false;
	double t2 = (-b + sqrt(tmp)) / (2 * a);
	if (t2 < 0)
		return false;
	double t1 = (-b - sqrt(tmp)) / (2 * a);
	t1 = (t1 < 0) ? t2 : t1;
	if (t1 >= r.max_t || t1 <= r.min_t)
		return false;*/
	double t1;
	double t2;
	if (!Sphere::test(r, t1, t2))
		return false;
	r.max_t = t1;
	i->t = t1;
	Vector3D normal = r.o + t1 * r.d - this->o;
	normal.normalize();
	i->n = normal;
	i->primitive = this;
	i->bsdf = this->get_bsdf();
  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
