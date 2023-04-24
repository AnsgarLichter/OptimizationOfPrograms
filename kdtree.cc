#include "kdtree.h"
#include <algorithm>
#include <iostream>

BoundingBox::BoundingBox() {}

BoundingBox::BoundingBox(Vector<FLOAT, 3> min, Vector<FLOAT, 3> max)
    : min(min), max(max) {}

void BoundingBox::split(BoundingBox &left, BoundingBox &right)
{
  left.min = min;
  right.max = max;

  float lengthX = std::abs(max[0] - min[0]);
  float lengthY = std::abs(max[1] - min[1]);
  float lengthZ = std::abs(max[2] - min[2]);

  // Split at the longest axis
  if (lengthX >= lengthY && lengthX >= lengthZ)
  {
    float newWidth = lengthX / 2;
    left.max = Vector<float, 3>{min[0] + newWidth, max[1], max[2]};
    right.min = Vector<float, 3>{min[0] + newWidth, min[1], min[2]};
    return;
  }

  if (lengthY >= lengthX && lengthY >= lengthZ)
  {
    float newWidth = lengthY / 2;
    left.max = Vector<float, 3>{max[0], min[1] + newWidth, max[2]};
    right.min = Vector<float, 3>{min[0], min[1] + newWidth, min[2]};
    return;
  }

  float newWidth = lengthZ / 2;
  left.max = Vector<float, 3>{max[0], max[1], min[2] + newWidth};
  right.min = Vector<float, 3>{min[0], min[1], min[2] + newWidth};
}

bool BoundingBox::contains(Vector<FLOAT, 3> v)
{
  return v[0] >= min[0] && v[0] <= max[0] && v[1] >= min[1] && v[1] <= max[1] && v[2] >= min[2] && v[2] <= max[2];
}

bool BoundingBox::contains(Triangle<FLOAT> *triangle)
{
  return contains(triangle->p1) || contains(triangle->p2) || contains(triangle->p3);
}

bool BoundingBox::intersects(Vector<FLOAT, 3> eye, Vector<FLOAT, 3> direction)
{
  // slab test implementation
  FLOAT tmin[3] = {(min[0] - eye[0]) / direction[0],
                   (min[1] - eye[1]) / direction[1],
                   (min[2] - eye[2]) / direction[2]};
  FLOAT tmax[3] = {(max[0] - eye[0]) / direction[0],
                   (max[1] - eye[1]) / direction[1],
                   (max[2] - eye[2]) / direction[2]};
  FLOAT tminimum = std::min(tmin[0], tmax[0]);
  FLOAT tmaximum = std::max(tmin[0], tmax[0]);
  tminimum = std::max(tminimum, std::min(tmin[1], tmax[1]));
  tmaximum = std::min(tmaximum, std::max(tmin[1], tmax[1]));
  tminimum = std::max(tminimum, std::min(tmin[2], tmax[2]));
  tmaximum = std::min(tmaximum, std::max(tmin[2], tmax[2]));

  return tmaximum >= tminimum;
}

KDTree::~KDTree()
{
  delete left;
  delete right;
}

KDTree *KDTree::buildTree(KDTree *tree, std::vector<Triangle<FLOAT> *> &triangles)
{
  if (triangles.size() <= MAX_TRIANGLES_PER_LEAF)
  {
    tree->triangles.insert(std::end(tree->triangles), std::begin(triangles), std::end(triangles));
    return tree;
  }

  this->left = new KDTree();
  this->right = new KDTree();
  this->box.split(left->box, right->box);

  auto trianglesInLeftTree = std::vector<Triangle<float> *>();
  auto trianglesInRightTree = std::vector<Triangle<float> *>();
  for (auto const &triangle : triangles)
  {
    bool isPartOfLeftTree = tree->left->box.contains(triangle);
    bool isPartOfRightTree = tree->right->box.contains(triangle);
    if (isPartOfLeftTree && isPartOfRightTree)
    {
      tree->triangles.push_back(triangle);
    }
    else if (isPartOfLeftTree)
    {
      trianglesInLeftTree.push_back(triangle);
    }
    else if (isPartOfRightTree)
    {
      trianglesInRightTree.push_back(triangle);
    }
  }

  this->left = this->left->buildTree(left, trianglesInLeftTree);
  this->right = this->right->buildTree(right, trianglesInRightTree);
  return tree;
}

KDTree *KDTree::buildTree(std::vector<Triangle<FLOAT> *> &triangles)
{
  KDTree *root = new KDTree();
  Vector<FLOAT, 3> min = {triangles[0]->p1[0], triangles[0]->p1[0], triangles[0]->p1[0]};
  Vector<FLOAT, 3> max = {triangles[0]->p1[0], triangles[0]->p1[0], triangles[0]->p1[0]};
  for (auto iterator = std::next(triangles.begin()); iterator != triangles.end(); ++iterator)
  {
    Triangle<float> *triangle = *iterator;

    max[0] = std::max({max[0], triangle->p1[0], triangle->p2[0], triangle->p3[0]});
    max[1] = std::max({max[1], triangle->p1[1], triangle->p2[1], triangle->p3[1]});
    max[2] = std::max({max[2], triangle->p1[2], triangle->p2[2], triangle->p3[2]});

    min[0] = std::min({min[0], triangle->p1[0], triangle->p2[0], triangle->p3[0]});
    min[1] = std::min({min[1], triangle->p1[1], triangle->p2[1], triangle->p3[1]});
    min[2] = std::min({min[2], triangle->p1[2], triangle->p2[2], triangle->p3[2]});
  }

  root->box = BoundingBox(min, max);
  root->buildTree(root, triangles);
  return root;
}

bool KDTree::hasNearestTriangle(Vector<FLOAT, 3> eye, Vector<FLOAT, 3> direction, Triangle<FLOAT> *&nearest_triangle, FLOAT &t, FLOAT &u, FLOAT &v, FLOAT minimum_t)
{
  if (!box.intersects(eye, direction))
  {
    return false;
  }

  if (this->left != nullptr && this->left->hasNearestTriangle(eye, direction, nearest_triangle, t, u, v, minimum_t))
  {
    minimum_t = t;
  }

  if (this->right && this->right->hasNearestTriangle(eye, direction, nearest_triangle, t, u, v, minimum_t))
  {
    minimum_t = t;
  }

  for (auto const triangle : this->triangles)
  {
    stats.no_ray_triangle_intersection_tests++;
    if (triangle->intersects(eye, direction, t, u, v, minimum_t))
    {
      stats.no_ray_triangle_intersections_found++;
      nearest_triangle = triangle;
      minimum_t = t;
    }
  }

  t = minimum_t;
  return nearest_triangle != nullptr;
}
