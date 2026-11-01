using System.Collections.Generic;
using UnityEngine;

namespace Niket.Pathfinding2D
{
    /// <summary>
    /// Parent of all pathfinding nodes. Collects child Nodes and offers lookup and bulk-linking helpers.
    /// </summary>
    public class NodeParent : MonoBehaviour
    {
        [Header("Gizmos")]
        public bool showLinks = true;
        public bool showLabels = true;
        public float nodeRadius = 0.15f;
        public Color nodeColor = new Color(1f, 0.9f, 0.2f);
        public Color linkColor = new Color(0.2f, 0.9f, 0.4f);
        public Color splineLinkColor = new Color(0.3f, 0.7f, 1f);

        [Header("Auto Link")]
        [Min(0f)] public float autoLinkRadius = 3f;
        [Min(0f)] public float autoLinkWeight = 1f;

        [SerializeField, HideInInspector] List<Node> nodes = new List<Node>();

        public IReadOnlyList<Node> Nodes => nodes;

        void Awake() => RefreshNodes();

        /// <summary>Re-collects all Node components under this parent.</summary>
        public void RefreshNodes()
        {
            nodes.Clear();
            GetComponentsInChildren(true, nodes);
        }

        /// <summary>Closest node to a world position, or null if there are none.</summary>
        public Node GetClosestNode(Vector2 position)
        {
            Node best = null;
            float bestSqr = float.MaxValue;
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i] == null) continue;
                float sqr = ((Vector2)nodes[i].transform.position - position).sqrMagnitude;
                if (sqr < bestSqr) { bestSqr = sqr; best = nodes[i]; }
            }
            return best;
        }

        /// <summary>Links every pair of nodes within autoLinkRadius (bidirectional).</summary>
        public void AutoLink()
        {
            RefreshNodes();
            float sqrRadius = autoLinkRadius * autoLinkRadius;
            for (int i = 0; i < nodes.Count; i++)
            for (int j = i + 1; j < nodes.Count; j++)
            {
                if (((Vector2)nodes[i].transform.position - (Vector2)nodes[j].transform.position).sqrMagnitude <= sqrRadius)
                    nodes[i].Connect(nodes[j], autoLinkWeight, true);
            }
        }

        public void ClearAllLinks()
        {
            RefreshNodes();
            foreach (Node n in nodes) n.ClearLinks();
        }

        void Reset() => RefreshNodes();
    }
}
