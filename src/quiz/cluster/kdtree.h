/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once
#include "../../render/render.h"
#include <vector>
#include <cmath>
#include <cstddef>

// -----------------------------
// KD-Tree Node
// -----------------------------
struct Node
{
    std::vector<float> point;  // x,y or x,y,z
    int id;
    Node* left;
    Node* right;

    Node(const std::vector<float>& arr, int setId)
    : point(arr), id(setId), left(nullptr), right(nullptr)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

// -----------------------------
// KD-Tree (2D or 3D)
// -----------------------------
// Auto-detects k (2 or 3) on first insert.
struct KdTree
{
    Node* root;
    unsigned k_;   // dimensionality (2 or 3). Set at first insert.

    KdTree() : root(nullptr), k_(0) {}
    ~KdTree() { delete root; }

    // ---------- Insert ----------
    static void insertHelper(Node*& node,
                             const std::vector<float>& point,
                             int id,
                             unsigned depth,
                             unsigned k)
    {
        if (node == nullptr)
        {
            node = new Node(point, id);
            return;
        }

        unsigned cd = depth % k;  // current split dimension
        if (point[cd] < node->point[cd])
            insertHelper(node->left, point, id, depth + 1, k);
        else
            insertHelper(node->right, point, id, depth + 1, k);
    }

    void insert(std::vector<float> point, int id)
    {
        // Accept 2D or 3D points only
        if (point.size() < 2) return;
        unsigned dims = (point.size() >= 3) ? 3u : 2u;

        // Initialize k_ on first insert; otherwise ensure consistent dims
        if (k_ == 0) k_ = dims;

        // If a different-sized point arrives later, truncate or pad (defensive)
        if (point.size() != k_)
        {
            std::vector<float> fixed(k_, 0.0f);
            for (unsigned i = 0; i < k_ && i < point.size(); ++i)
                fixed[i] = point[i];
            point.swap(fixed);
        }

        insertHelper(root, point, id, 0, k_);
    }

    // ---------- Search ----------
    static void searchHelper(Node* node,
                             const std::vector<float>& target,
                             float distanceTol,
                             unsigned depth,
                             unsigned k,
                             std::vector<int>& ids)
    {
        if (node == nullptr) return;

        // AABB pre-check (k-D)
        bool inside = true;
        for (unsigned d = 0; d < k; ++d)
        {
            if (node->point[d] < target[d] - distanceTol || node->point[d] > target[d] + distanceTol)
            {
                inside = false; break;
            }
        }
        if (inside)
        {
            // Exact Euclidean distance in k-D
            float dist2 = 0.0f;
            for (unsigned d = 0; d < k; ++d)
            {
                float diff = node->point[d] - target[d];
                dist2 += diff * diff;
            }
            if (dist2 <= distanceTol * distanceTol)
                ids.push_back(node->id);
        }

        // Recurse based on split dimension
        unsigned cd = depth % k;
        if (target[cd] - distanceTol < node->point[cd])
            searchHelper(node->left, target, distanceTol, depth + 1, k, ids);
        if (target[cd] + distanceTol > node->point[cd])
            searchHelper(node->right, target, distanceTol, depth + 1, k, ids);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        if (k_ == 0) return ids;                 // empty tree
        if (target.size() != k_)
        {
            // normalize target size to k_
            std::vector<float> fixed(k_, 0.0f);
            for (unsigned i = 0; i < k_ && i < target.size(); ++i)
                fixed[i] = target[i];
            target.swap(fixed);
        }
        searchHelper(root, target, distanceTol, 0, k_, ids);
        return ids;
    }
};
