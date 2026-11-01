using System;
using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding2D
{
    /// <summary>
    /// A connection from one node to another. Links are directional; use Node.Connect with
    /// bidirectional = true to create the reverse link as well.
    /// </summary>
    [Serializable]
    public class NodeLink
    {
        public Node target;

        [Min(0f), Tooltip("Cost multiplier. Final traversal cost = Distance * weight.")]
        public float weight = 1f;

        [Tooltip("Follow a curve between the nodes instead of a straight line.")]
        public bool useSpline;

        [Tooltip("Spline tangent offset from the owning node (world space offset).")]
        public Vector2 startTangent;

        [Tooltip("Spline tangent offset from the target node (world space offset).")]
        public Vector2 endTangent;

        const int LengthSamples = 20;

        /// <summary>Length of the path between the nodes (curve length if useSpline is on).</summary>
        public float GetDistance(Node owner)
        {
            if (owner == null || target == null) return 0f;
            if (!useSpline) return Vector2.Distance(owner.Position, target.Position);

            float length = 0f;
            Vector2 prev = owner.Position;
            for (int i = 1; i <= LengthSamples; i++)
            {
                Vector2 p = Evaluate(owner, i / (float)LengthSamples);
                length += Vector2.Distance(prev, p);
                prev = p;
            }
            return length;
        }

        /// <summary>Traversal cost used by the pathfinder: distance scaled by weight.</summary>
        public float GetCost(Node owner) => GetDistance(owner) * weight;

        /// <summary>Point along the link, t in [0,1].</summary>
        public Vector2 Evaluate(Node owner, float t)
        {
            Vector2 a = owner.Position;
            Vector2 b = target.Position;
            if (!useSpline) return Vector2.Lerp(a, b, t);

            // Cubic Bezier: control points are the tangent offsets from each end.
            Vector2 c1 = a + startTangent;
            Vector2 c2 = b + endTangent;
            float u = 1f - t;
            return u * u * u * a + 3f * u * u * t * c1 + 3f * u * t * t * c2 + t * t * t * b;
        }
    }

    /// <summary>
    /// A single pathfinding node. Place as a child of a NodeParent.
    /// </summary>
    [DisallowMultipleComponent]
    public class Node : MonoBehaviour
    {
        [SerializeField] List<NodeLink> links = new List<NodeLink>();

        public IReadOnlyList<NodeLink> Links => links;
        public Vector2 Position => transform.position;

        /// <summary>Returns the link from this node to target, or null.</summary>
        public NodeLink GetLink(Node target)
        {
            for (int i = 0; i < links.Count; i++)
                if (links[i].target == target) return links[i];
            return null;
        }

        public bool IsConnectedTo(Node target) => GetLink(target) != null;

        /// <summary>
        /// Connects this node to target. If a link already exists its weight is updated.
        /// </summary>
        public NodeLink Connect(Node target, float weight = 1f, bool bidirectional = true)
        {
            if (target == null || target == this) return null;

            NodeLink link = GetLink(target);
            if (link == null)
            {
                link = new NodeLink { target = target };
                links.Add(link);
            }
            link.weight = weight;

            if (bidirectional) target.Connect(this, weight, false);
            return link;
        }

        public void Disconnect(Node target, bool bidirectional = true)
        {
            links.RemoveAll(l => l.target == target);
            if (bidirectional && target != null) target.Disconnect(this, false);
        }

        public void ClearLinks() => links.Clear();

        /// <summary>Removes links whose target has been destroyed or points at self.</summary>
        public void CleanLinks() => links.RemoveAll(l => l.target == null || l.target == this);

        void OnValidate() => CleanLinks();
    }
}
