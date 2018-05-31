using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace ToyPhysics.Collision
{
    public class DynamicTreeNode
    {
        internal AABB AABB;
        public object Context;
        internal int Depth;

        internal DynamicTreeNode Parent;
        internal DynamicTreeNode LeftChild;
        internal DynamicTreeNode RightChild;

        internal void Initialize(AABB aabb, object context, int depth)
        {
            AABB = aabb;
            Context = context;
            Depth = depth;
        }

        internal void Free()
        {
            AABB = default(AABB);
            Context = null;
            Depth = -1;
            Parent = LeftChild = RightChild = null;
        }

        internal bool IsLeaf => LeftChild == null;

        internal bool IsRoot => Parent == null;

        internal bool IsFree => Depth < 0;
    }

    public class DynamicTree
    {
        private List<DynamicTreeNode> nodes = new List<DynamicTreeNode>();
        private int count;
        private DynamicTreeNode root;
        private Queue<DynamicTreeNode> freeNodes = new Queue<DynamicTreeNode>();

        private Stack<DynamicTreeNode> raycastStack = new Stack<DynamicTreeNode>();
        private Stack<DynamicTreeNode> queryStack = new Stack<DynamicTreeNode>();

        public DynamicTreeNode AddProxy(AABB aabb, object context)
        {
            var node = AllocateNode();

            node.Initialize(aabb.Enlarge(new Vector2D(Configuration.FatAABBExtension, Configuration.FatAABBExtension)), context, 0);

            InsertLeaf(node);

            return node;
        }

        public void RemoveProxy(DynamicTreeNode node)
        {
            Debug.Assert(node.IsLeaf);

            RemoveLeaf(node);
            FreeNode(node);
        }

        public bool MoveProxy(DynamicTreeNode node, AABB aabb, Vector2D displacement)
        {
            Debug.Assert(node.IsLeaf);

            if (node.AABB.Contains(aabb))
                return false;

            RemoveLeaf(node);

            // Extend AABB.
            aabb = aabb.Enlarge(new Vector2D(Configuration.FatAABBExtension, Configuration.FatAABBExtension));

            // Predict AABB displacement.
            displacement = Configuration.AABBDisplacementMultiplier * displacement;

            if (displacement.X < 0.0f)
                aabb.LowerBound.X += displacement.X;
            else
                aabb.UpperBound.X += displacement.X;

            if (displacement.Y < 0.0f)
                aabb.LowerBound.Y += displacement.Y;
            else
                aabb.UpperBound.Y += displacement.Y;

            node.AABB = aabb;

            InsertLeaf(node);
            return true;
        }

        public void Overlap(AABB aabb, Func<DynamicTreeNode, bool> callback)
        {
            queryStack.Clear();
            queryStack.Push(root);

            while (queryStack.Count > 0)
            {
                var node = queryStack.Pop();
                if (node == null)
                    continue;

                if (aabb.Overlap(node.AABB))
                {
                    if (node.IsLeaf)
                    {
                        if (!callback(node))
                            return;
                    }
                    else
                    {
                        queryStack.Push(node.LeftChild);
                        queryStack.Push(node.RightChild);
                    }
                }
            }
        }

        public struct RayCastInput
        {
            public Vector2D Point1;
            public Vector2D Point2;
            public float MaxFraction;
        }

        public void RayCast(RayCastInput input, Func<RayCastInput, DynamicTreeNode, float> callback)
        {
            var p1 = input.Point1;
            var p2 = input.Point2;
            var r = p2 - p1;
            Debug.Assert(r.LengthSquared() > 0.0f);
            r.Normalize();

            // v is perpendicular to the segment.
            var absV = new Vector2D(Math.Abs(r.Y), Math.Abs(r.X)); //FPE: Inlined the 'v' variable

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            float maxFraction = input.MaxFraction;

            // Build a bounding box for the segment.
            AABB segmentAABB = new AABB();
            {
                var t = p1 + maxFraction * (p2 - p1);
                segmentAABB.LowerBound = Vector2D.Min(p1, t);
                segmentAABB.UpperBound = Vector2D.Max(p1, t);
            }

            raycastStack.Clear();
            raycastStack.Push(root);

            while (raycastStack.Count > 0)
            {
                var node = raycastStack.Pop();
                if (node == null)
                    continue;

                if (!segmentAABB.Overlap(node.AABB))
                    continue;

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                Vector2D c = node.AABB.Center;
                Vector2D h = node.AABB.Extents;
                float separation = Math.Abs(Vector2D.Dot(new Vector2D(-r.Y, r.X), p1 - c)) - Vector2D.Dot(absV, h);
                if (separation > 0.0f)
                    continue;

                if (node.IsLeaf)
                {
                    RayCastInput subInput;
                    subInput.Point1 = input.Point1;
                    subInput.Point2 = input.Point2;
                    subInput.MaxFraction = maxFraction;

                    float value = callback(subInput, node);

                    if (value == 0.0f)
                    {
                        // the client has terminated the raycast.
                        return;
                    }

                    if (value > 0.0f)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        Vector2D t = p1 + maxFraction * (p2 - p1);
                        segmentAABB.LowerBound = Vector2D.Min(p1, t);
                        segmentAABB.UpperBound = Vector2D.Max(p1, t);
                    }
                }
                else
                {
                    raycastStack.Push(node.LeftChild);
                    raycastStack.Push(node.RightChild);
                }
            }
        }

        private DynamicTreeNode AllocateNode()
        {
            DynamicTreeNode node;
            if (freeNodes.Count > 0)
            {
                node = freeNodes.Dequeue();
            }
            else
            {
                node = new DynamicTreeNode();
                nodes.Add(node);
            }

            ++count;
            return node;
        }

        private void FreeNode(DynamicTreeNode node)
        {
            Debug.Assert(0 < count);
            node.Free();
            freeNodes.Enqueue(node);
            --count;
        }

        private void InsertLeaf(DynamicTreeNode leaf)
        {
            if (root == null)
            {
                root = leaf;
                root.Parent = null;
                return;
            }

            // Find the best sibling for this node
            var sibling = FindSibling(leaf);

            // Create a new parent.
            var oldParent = sibling.Parent;
            var newParent = AllocateNode();
            newParent.Initialize(leaf.AABB.Combine(sibling.AABB), null, sibling.Depth + 1);
            newParent.Parent = oldParent;

            if (oldParent != null)
            {
                // The sibling was not the root.
                if (oldParent.LeftChild == sibling)
                    oldParent.LeftChild = newParent;
                else
                    oldParent.RightChild = newParent;

                newParent.LeftChild = sibling;
                newParent.RightChild = leaf;
                sibling.Parent = newParent;
                leaf.Parent = newParent;
            }
            else
            {
                // The sibling was the root.
                newParent.LeftChild = sibling;
                newParent.RightChild = leaf;
                sibling.Parent = newParent;
                leaf.Parent = newParent;
                root = newParent;
            }

            // Walk back up the tree fixing depths and AABBs
            BalanceBranch(leaf.Parent);

#if DEBUG
            Validate();
#endif
        }

        private DynamicTreeNode FindSibling(DynamicTreeNode leaf)
        {
            var sibling = root;
            while (!sibling.IsLeaf)
            {
                var area = sibling.AABB.Perimeter;

                var combinedAABB = leaf.AABB.Combine(sibling.AABB);
                var combinedArea = combinedAABB.Perimeter;

                // Cost of creating a new parent for this node and the new leaf
                var cost = 2.0f * combinedArea;

                // Minimum cost of pushing the leaf further down the tree
                var inheritanceCost = 2.0f * (combinedArea - area);

                // Cost of descending into child1
                var cost1 = CalculateSiblingChildCost(leaf, sibling.LeftChild, inheritanceCost);
                var cost2 = CalculateSiblingChildCost(leaf, sibling.RightChild, inheritanceCost);

                // Descend according to the minimum cost.
                if (cost < cost1 && cost1 < cost2)
                    return sibling;

                // Descend
                sibling = cost1 < cost2 ? sibling.LeftChild : sibling.RightChild;
            }

            return sibling;
        }

        private float CalculateSiblingChildCost(DynamicTreeNode leaf, DynamicTreeNode child, float inheritanceCost)
        {
            var aabb = leaf.AABB.Combine(child.AABB);
            if (child.IsLeaf)
                return aabb.Perimeter + inheritanceCost;
            else
                return (aabb.Perimeter - child.AABB.Perimeter) + inheritanceCost;
        }

        private void RemoveLeaf(DynamicTreeNode leaf)
        {
            Debug.Assert(leaf.IsLeaf);

            if (leaf == root)
            {
                root = null;
                return;
            }

            var parent = leaf.Parent;
            var grandParent = parent.Parent;
            var sibling = parent.LeftChild == leaf ? parent.RightChild : parent.LeftChild;

            if (grandParent != null)
            {
                // Destroy parent and connect sibling to grandParent.
                if (grandParent.LeftChild == parent)
                    grandParent.LeftChild = sibling;
                else
                    grandParent.RightChild = sibling;
                sibling.Parent = grandParent;
                FreeNode(parent);

                // Adjust ancestor bounds.
                BalanceBranch(grandParent);
            }
            else
            {
                root = sibling;
                sibling.Parent = null;
                FreeNode(parent);
            }

#if DEBUG
            Validate();
#endif
        }

        private void BalanceBranch(DynamicTreeNode node)
        {
            while (node != null)
            {
                node = Balance(node);

                Debug.Assert(node.LeftChild != null);
                Debug.Assert(node.RightChild != null);

                node.Depth = 1 + Math.Max(node.LeftChild.Depth, node.RightChild.Depth);
                node.AABB = node.LeftChild.AABB.Combine(node.RightChild.AABB);

                node = node.Parent;
            }
        }

        private DynamicTreeNode Balance(DynamicTreeNode node)
        {
            Debug.Assert(node != null);

            if (node.IsLeaf || node.Depth < 2)
                return node;

            var nodeB = node.LeftChild;
            var nodeC = node.RightChild;

            int balance = nodeC.Depth - nodeB.Depth;

            // Rotate C up
            if (balance > 1)
                return Rotate(node, false);

            // Rotate B up
            if (balance < -1)
                return Rotate(node, true);

            return node;
        }

        private DynamicTreeNode Rotate(DynamicTreeNode parent, bool leftChild)
        {
            var child1 = leftChild ? parent.LeftChild : parent.RightChild;
            var child2 = leftChild ? parent.RightChild : parent.LeftChild;

            var grandChild1 = child1.LeftChild;
            var grandChild2 = child1.RightChild;

            // Swap A and C
            child1.LeftChild = parent;
            child1.Parent = parent.Parent;
            parent.Parent = child1;

            // A's old parent should point to C
            if (child1.Parent != null)
            {
                if (child1.Parent.LeftChild == parent)
                {
                    child1.Parent.LeftChild = child1;
                }
                else
                {
                    Debug.Assert(child1.Parent.RightChild == parent);
                    child1.Parent.RightChild = child1;
                }
            }
            else
            {
                root = child1;
            }

            // Rotate
            if (grandChild1.Depth > grandChild2.Depth)
            {
                child1.RightChild = grandChild1;
                if (leftChild)
                    parent.LeftChild = grandChild2;
                else
                    parent.RightChild = grandChild2;
                grandChild2.Parent = parent;
                parent.AABB = child2.AABB.Combine(grandChild2.AABB);
                child1.AABB = parent.AABB.Combine(grandChild1.AABB);

                parent.Depth = 1 + Math.Max(child2.Depth, grandChild2.Depth);
                child1.Depth = 1 + Math.Max(parent.Depth, grandChild1.Depth);
            }
            else
            {
                child1.RightChild = grandChild2;
                if (leftChild)
                    parent.LeftChild = grandChild1;
                else
                    parent.RightChild = grandChild1;
                grandChild1.Parent = parent;
                parent.AABB = child2.AABB.Combine(grandChild1.AABB);
                child1.AABB = parent.AABB.Combine(grandChild2.AABB);

                parent.Depth = 1 + Math.Max(child2.Depth, grandChild1.Depth);
                child1.Depth = 1 + Math.Max(parent.Depth, grandChild2.Depth);
            }

            return child1;
        }

        public int ComputeDepth(DynamicTreeNode node)
        {
            if (node.IsLeaf)
                return 0;

            int height1 = ComputeDepth(node.LeftChild);
            int height2 = ComputeDepth(node.RightChild);
            return 1 + Math.Max(height1, height2);
        }

        public int ComputeDepth()
        {
            return ComputeDepth(root);
        }

        public void Validate()
        {
            ValidateStructure(root);
            ValidateMetrics(root);

            Debug.Assert(Depth == ComputeDepth());

            Debug.Assert(count + freeNodes.Count == nodes.Count);
        }

        void ValidateStructure(DynamicTreeNode node)
        {
            if (node == null)
                return;

            if (node == root)
                Debug.Assert(node.Parent == null);

            var leftChild = node.LeftChild;
            var rightChild = node.RightChild;

            if (node.IsLeaf)
            {
                Debug.Assert(leftChild == null);
                Debug.Assert(rightChild == null);
                Debug.Assert(node.Depth == 0);
                return;
            }

            Debug.Assert(leftChild.Parent == node);
            Debug.Assert(rightChild.Parent == node);

            ValidateStructure(leftChild);
            ValidateStructure(rightChild);
        }

        void ValidateMetrics(DynamicTreeNode node)
        {
            if (node == null)
                return;

            var leftChild = node.LeftChild;
            var rightChild = node.RightChild;

            if (node.IsLeaf)
            {
                Debug.Assert(leftChild == null);
                Debug.Assert(rightChild == null);
                Debug.Assert(node.Depth == 0);
                return;
            }

            int depth1 = leftChild.Depth;
            int depth2 = rightChild.Depth;
            int depth = 1 + Math.Max(depth1, depth2);
            Debug.Assert(node.Depth == depth);

            var aabb = leftChild.AABB.Combine(rightChild.AABB);

            Debug.Assert(aabb.LowerBound == node.AABB.LowerBound);
            Debug.Assert(aabb.UpperBound == node.AABB.UpperBound);

            ValidateMetrics(leftChild);
            ValidateMetrics(rightChild);
        }

        /*
        public void RebuildBottomUp()
        {
            int[] nodes = new int[this.count];
            int count = 0;

            // Build array of leaves. Free the rest.
            for (int i = 0; i < _nodeCapacity; ++i)
            {
                if (this.nodes[i].Depth < 0)
                {
                    // free node in pool
                    continue;
                }

                if (this.nodes[i].IsLeaf())
                {
                    this.nodes[i].Parent = NullNode;
                    nodes[count] = i;
                    ++count;
                }
                else
                {
                    FreeNode(i);
                }
            }

            while (count > 1)
            {
                float minCost = Settings.MaxFloat;
                int iMin = -1, jMin = -1;
                for (int i = 0; i < count; ++i)
                {
                    AABB AABBi = this.nodes[nodes[i]].AABB;

                    for (int j = i + 1; j < count; ++j)
                    {
                        AABB AABBj = this.nodes[nodes[j]].AABB;
                        AABB b = new AABB();
                        b.Combine(ref AABBi, ref AABBj);
                        float cost = b.Perimeter;
                        if (cost < minCost)
                        {
                            iMin = i;
                            jMin = j;
                            minCost = cost;
                        }
                    }
                }

                int index1 = nodes[iMin];
                int index2 = nodes[jMin];
                DynamicTreeNode<T> child1 = this.nodes[index1];
                DynamicTreeNode<T> child2 = this.nodes[index2];

                int parentIndex = AllocateNode();
                DynamicTreeNode<T> parent = this.nodes[parentIndex];
                parent.LeftChild = index1;
                parent.RightChild = index2;
                parent.Depth = 1 + Math.Max(child1.Depth, child2.Depth);
                parent.AABB.Combine(ref child1.AABB, ref child2.AABB);
                parent.Parent = NullNode;

                child1.Parent = parentIndex;
                child2.Parent = parentIndex;

                nodes[jMin] = nodes[count - 1];
                nodes[iMin] = parentIndex;
                --count;
            }

            root = nodes[0];

            Validate();
        }
        */

        public void ShiftOrigin(Vector2D newOrigin)
        {
            // Build array of leaves. Free the rest.
            foreach (var node in nodes)
            {
                if (!node.IsFree)
                {
                    node.AABB.LowerBound -= newOrigin;
                    node.AABB.UpperBound -= newOrigin;
                }
            }
        }

        public int Count => count;

        public int FreeCount => freeNodes.Count;

        public int Depth => root != null ? root.Depth : 0;

        public float AreaRatio
        {
            get
            {
                if (root == null)
                    return 0;

                var totalArea = nodes.Sum(node => node.IsFree ? 0f : node.AABB.Perimeter);

                return totalArea / root.AABB.Perimeter;
            }
        }

        public int MaxBalance => nodes.Max(node => node.IsFree ? 0 : Math.Abs(node.LeftChild.Depth - node.RightChild.Depth));
    }
}
