#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

/**
 * This function creates a object space (basis vectors) from the normal vector
 */
void make_coord_space(Matrix3x3 &o2w, const Vector3D &n) {

  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

/**
 * Evaluate diffuse lambertian BSDF.
 * Given incident light direction wi and outgoing light direction wo. Note
 * that both wi and wo are defined in the local coordinate system at the
 * point of intersection.
 * \param wo outgoing light direction in local space of point of intersection
 * \param wi incident light direction in local space of point of intersection
 * \return reflectance in the given incident/outgoing directions
 */
Spectrum DiffuseBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  // TODO (Part 3.1):
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.

  return reflectance/(PI);
}

/**
 * Evalutate diffuse lambertian BSDF.
 */
Spectrum DiffuseBSDF::sample_f(const Vector3D &wo, Vector3D *wi, float *pdf) {
  // TODO (Part 3.1):
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).
  // You can use the `f` function. The reference solution only takes two lines.
	*wi = DiffuseBSDF::sampler.get_sample(pdf);
	return DiffuseBSDF::f(wo, *wi);
}

//===============================================================
// Project 3-2 Code. Don't worry about these for project 3-1
//===============================================================

/**
 * Evalutate Mirror BSDF
 */
Spectrum MirrorBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  // Project 3-2
	if (wo.z == wi.z)
		return reflectance * (1.0 / fabs(wi.z));
	else
		return Spectrum();
}

/**
 * Evalutate Mirror BSDF
 */
Spectrum MirrorBSDF::sample_f(const Vector3D &wo, Vector3D *wi, float *pdf) {
	reflect(wo, wi);
	*pdf = 1.0;
	return f(wo, *wi);
}

/**
 * Evalutate Glossy BSDF
 */
Spectrum GlossyBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  return Spectrum();
}

/**
 * Evalutate Glossy BSDF
 */
Spectrum GlossyBSDF::sample_f(const Vector3D &wo, Vector3D *wi, float *pdf) {
  return Spectrum();
}

/**
 * Evalutate Refraction BSDF
 */
Spectrum RefractionBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  return Spectrum();
}

/**
 * Evalutate Refraction BSDF
 */
Spectrum RefractionBSDF::sample_f(const Vector3D &wo, Vector3D *wi,
                                  float *pdf) {
	if (!refract(wo, wi, ior)) {
		*pdf = 1.0;
		reflect(wo, wi);
		return Spectrum(0, 0, 0);
	}

	float n1, n2, cos_theta;
	if (wo.z > 0) {
		n1 = 1.0;
		n2 = ior;
	}
	else {
		n2 = 1.0;
		n1 = ior;
	}

	refract(wo, wi, ior);
	*pdf = 1.0;
	Spectrum f = transmittance * (n2 * n2 / (n1 * n1 * fabs(wi->z)));
	return f;
}

/**
 * Evalutate Glass BSDF
 */
Spectrum GlassBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  return Spectrum();
}

/**
 * Evalutate Glass BSDF
 */
Spectrum GlassBSDF::sample_f(const Vector3D &wo, Vector3D *wi, float *pdf) {
	if (!refract(wo, wi, ior)) {
		*pdf = 1.0;
		reflect(wo, wi);
		return reflectance * (1.0 / fabs(wi->z));
	}

	float n1, n2, cos_theta;
	if (wo.z > 0) {
		n1 = 1.0;
		n2 = ior;
		cos_theta = fabs(wo.z);
	}
	else {
		n2 = 1.0;
		n1 = ior;
		cos_theta = fabs(wi->z);
	}
	float R0 = pow(((n1 - n2) / (n1 + n2)), 2);
	float Fr = R0 + (1.0 - R0) * pow((1.0 - cos_theta), 5);
	if ((double)(std::rand()) / RAND_MAX < Fr) {
		reflect(wo, wi);
		*pdf = Fr;
		return Fr * reflectance * (1.0 / fabs(wi->z));
	}
	else {
		refract(wo, wi, ior);
		*pdf = (1.0 - Fr);
		Spectrum f = transmittance * ((1.0 - Fr) * n2 * n2 / (n1 * n1 * fabs(wi->z)));
		return f;
	}
}

/**
 * Compute the reflection vector according to incident vector
 */
void BSDF::reflect(const Vector3D &wo, Vector3D *wi) {
	Vector3D normal;
	if (wo.z < 0)
		normal = Vector3D(0, 0, -1.0);
	else
		normal = Vector3D(0, 0, 1.0);

	*wi = -wo + 2.0 * dot(wo, normal) * normal;
}

/**
 * Compute the refraction vector according to incident vector and ior
 */
bool BSDF::refract(const Vector3D &wo, Vector3D *wi, float ior) {
	double sin_theta;
	float n;

	// decide wo entering surface / going out
	if (wo.z > 0) {  // entering
		n = 1.0 / ior;
		wi->z = -sqrt(1.0 - n * n * (1.0 - wo.z * wo.z));
	}
	else {
		n = ior;
		if ((1.0 - n * n * (1.0 - wo.z * wo.z)) < 0.)
			return false;
		wi->z = sqrt(1.0 - n * n * (1.0 - wo.z * wo.z));
	}
	wi->x = -n * wo.x;
	wi->y = -n * wo.y;
	wi->normalize();
	return true;
}

/**
 * Evalutate Emission BSDF (Light Source)
 */
Spectrum EmissionBSDF::f(const Vector3D &wo, const Vector3D &wi) {
  return Spectrum();
}

/**
 * Evalutate Emission BSDF (Light Source)
 */
Spectrum EmissionBSDF::sample_f(const Vector3D &wo, Vector3D *wi, float *pdf) {
  *pdf = 1.0 / PI;
  *wi = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
