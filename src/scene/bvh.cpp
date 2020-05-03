#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

// Another solution, sort, O(NlgN)
//bool cmp0(Primitive * a,
//		  Primitive * b) {
//	return (a->get_bbox().centroid()[0] < b->get_bbox().centroid()[0]);
//}
//bool cmp1(Primitive * a,
//		  Primitive * b) {
//	return (a->get_bbox().centroid()[1] < b->get_bbox().centroid()[1]);
//}
//bool cmp2(Primitive * a,
//		  Primitive * b) {
//	return (a->get_bbox().centroid()[2] < b->get_bbox().centroid()[2]);
//}

// find the k-th element in an array, partition, O(lgN) time
std::vector<Primitive *>::iterator findK(std::vector<Primitive *>::iterator start,
										 std::vector<Primitive *>::iterator end,
										 int k,
										 int axis){
	if (start+1 == end)
		return start;
	auto p = start;
	Primitive *pivot = *start;
	for (auto ite = start+1; ite != end; ite++) {
		if ((*ite)->get_bbox().centroid()[axis] < pivot->get_bbox().centroid()[axis]) {
			swap(*(++p), *ite);
		}
	}
	swap(*start, *p);
	int tmp = p - start;
	if (tmp == k)
		return p;
	else if (tmp > k)
		return findK(start, p+1, k, axis);
	else return findK(p+1, end, k - tmp - 1, axis);

	// Another solution, sort, O(NlgN)
	//if (axis == 0)
	//	sort(start, end, cmp0);
	//else if (axis == 1)
	//	sort(start, end, cmp1);
	//else if (axis == 2)
	//	sort(start, end, cmp2);
	//return start + k;
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  int cnt = 0;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
	cnt++;
  }

  if (start == end || cnt <= max_leaf_size) { // leaf node
	  BVHNode *node = new BVHNode(bbox);
	  node->start = start;
	  node->end = end;
	  return node;
  }
  Vector3D wlh = bbox.max - bbox.min;
  int axis;
  if (wlh.x > wlh.y && wlh.x > wlh.z)
	  axis = 0;
  else if (wlh.y > wlh.x && wlh.y > wlh.z)
	  axis = 1;
  else axis = 2;

  double mid = bbox.centroid()[axis];
  vector<Primitive *> leftVct;
  vector<Primitive *> rightVct;
  for (auto ite = start; ite != end; ite++) {
	  if ((*ite)->get_bbox().centroid()[axis] < mid)
		  leftVct.push_back(*ite);
	  else
		  rightVct.push_back(*ite);
  }

  if (leftVct.empty() || rightVct.empty()) {
	  // if generate an empty area
	  auto k = findK(start, end, (end - start - 1) / 2, axis);
	  double mid = (*k)->get_bbox().centroid()[axis];
	  leftVct.clear();
	  rightVct.clear();
	  for (auto ite = start; ite != end; ite++) {
		  if ((*ite)->get_bbox().centroid()[axis] <= mid)
			  leftVct.push_back(*ite);
		  else
			  rightVct.push_back(*ite);
	  }
	  if (leftVct.empty()) {
		  leftVct.push_back(rightVct.back());
		  rightVct.pop_back();
	  }
	  else if (rightVct.empty()) {
		  rightVct.push_back(leftVct.back());
		  leftVct.pop_back();
	  }
  }

  auto ite = start;
  for (Primitive *p : leftVct) {
	  (*ite) = p;
	  ite++;
  }
  auto midEnd = ite;
  for (Primitive *p : rightVct) {
	  (*ite) = p;
	  ite++;
  }

  BVHNode *node = new BVHNode(bbox);
  //if (leftVct.empty()) {
	 // node->l = BVHAccel::construct_bvh(start, midEnd, max_leaf_size);
	 // BBox bbox;
	 // for (Primitive *p : rightVct) {
		//  BBox bb = p->get_bbox();
		//  bbox.expand(bb);
	 // }
	 // node->r = new BVHNode(bbox);
	 // node->r->start = midEnd;
	 // node->r->end = end;
	 // return node;
  //}
  //if (rightVct.empty()) {
	 // node->r = BVHAccel::construct_bvh(midEnd, end, max_leaf_size);
	 // BBox bbox;
	 // for (Primitive *p : leftVct) {
		//  BBox bb = p->get_bbox();
		//  bbox.expand(bb);
	 // }
	 // node->l = new BVHNode(bbox);
	 // node->l->start = start;
	 // node->l->end = midEnd;
	 // return node;
  //}
  node->l = BVHAccel::construct_bvh(start, midEnd, max_leaf_size);
  node->r = BVHAccel::construct_bvh(midEnd, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
	double t0, t1;
	if (!node || !node->bb.intersect(ray, t0, t1)) {
		return false;
	}
	if (node->isLeaf()) {
		// test intersection with all primitives
		for (auto ite = node->start; ite != node->end; ite++) {
			total_isects++;
			if ((*ite)->has_intersection(ray))
				return true;
		}
		return false;
	}
	if (node->l && has_intersection(ray, node->l)) {
		return true;
	}
	if (node->r && has_intersection(ray, node->r)) {
		return true;
	}
	return false;
  /*for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;*/
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
	double t0, t1;
	if (!node || !node->bb.intersect(ray, t0, t1)) {
		return false;
	}
	bool hit = false;
	if (node->isLeaf()) {
		// test intersection with all primitives
		for (auto ite = node->start; ite != node->end; ite++) {
			total_isects++;
			hit = (*ite)->intersect(ray, i) || hit;
		}
		return hit;
	}

	bool left = false;
	bool right = false;
	if (node->l) {
		left = intersect(ray, i, node->l);
	}
	if (node->r) {
		right = intersect(ray, i, node->r);
	}
	return left || right;

  /*bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit; */
}

} // namespace SceneObjects
} // namespace CGL
