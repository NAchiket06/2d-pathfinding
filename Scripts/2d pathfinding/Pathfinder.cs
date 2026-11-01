using System.Collections.Generic;
using UnityEngine;

namespace Niket.Pathfinding2D
{
    /// <summary>
    /// A* search over a NodeParent's node graph. Link cost is Distance * weight.
    /// Reuses its internal buffers between searches; not thread safe.
    /// </summary>
    public class Pathfinder
    {
        struct Entry
        {
            public Node node;
            public float f;
        }

        readonly NodeParent parent;
        readonly Dictionary<Node, float> gScore = new Dictionary<Node, float>();
        readonly Dictionary<Node, Node> cameFrom = new Dictionary<Node, Node>();
        readonly HashSet<Node> closed = new HashSet<Node>();
        readonly List<Entry> open = new List<Entry>(); // binary min-heap on f

        public Pathfinder(NodeParent parent)
        {
            this.parent = parent;
        }

        /// <summary>Finds the path between the nodes closest to the two world positions.</summary>
        public bool TryFindPath(Vector2 startPos, Vector2 endPos, List<Node> path, out float cost)
        {
            return TryFindPath(parent.GetClosestNode(startPos), parent.GetClosestNode(endPos), path, out cost);
        }

        /// <summary>
        /// Finds the cheapest path from start to end. On success, path is filled with the nodes in
        /// order (including start and end) and cost is the total traversal cost.
        /// </summary>
        public bool TryFindPath(Node start, Node end, List<Node> path, out float cost)
        {
            path.Clear();
            cost = 0f;
            if (start == null || end == null) return false;
            if (start == end)
            {
                path.Add(start);
                return true;
            }

            // The straight-line heuristic must never overestimate, so scale it by the cheapest weight.
            float hScale = Mathf.Min(1f, GetMinWeight());

            gScore.Clear();
            cameFrom.Clear();
            closed.Clear();
            open.Clear();

            gScore[start] = 0f;
            Push(start, Heuristic(start, end, hScale));

            while (open.Count > 0)
            {
                Node current = Pop();
                if (!closed.Add(current)) continue; // stale heap entry

                if (current == end)
                {
                    cost = gScore[end];
                    BuildPath(start, end, path);
                    return true;
                }

                float currentG = gScore[current];
                IReadOnlyList<NodeLink> links = current.Links;
                for (int i = 0; i < links.Count; i++)
                {
                    NodeLink link = links[i];
                    Node next = link.target;
                    if (next == null || closed.Contains(next)) continue;

                    float tentative = currentG + link.GetCost(current);
                    if (gScore.TryGetValue(next, out float existing) && tentative >= existing) continue;

                    gScore[next] = tentative;
                    cameFrom[next] = current;
                    Push(next, tentative + Heuristic(next, end, hScale));
                }
            }

            return false;
        }

        float GetMinWeight()
        {
            float min = float.MaxValue;
            foreach (Node n in parent.Nodes)
            {
                if (n == null) continue;
                foreach (NodeLink l in n.Links)
                    if (l.weight < min) min = l.weight;
            }
            return min == float.MaxValue ? 1f : min;
        }

        static float Heuristic(Node a, Node b, float scale) => Vector2.Distance(a.Position, b.Position) * scale;

        void BuildPath(Node start, Node end, List<Node> path)
        {
            for (Node n = end; n != start; n = cameFrom[n]) path.Add(n);
            path.Add(start);
            path.Reverse();
        }

        void Push(Node node, float f)
        {
            open.Add(new Entry { node = node, f = f });
            int i = open.Count - 1;
            while (i > 0)
            {
                int p = (i - 1) / 2;
                if (open[p].f <= open[i].f) break;
                (open[p], open[i]) = (open[i], open[p]);
                i = p;
            }
        }

        Node Pop()
        {
            Node top = open[0].node;
            int last = open.Count - 1;
            open[0] = open[last];
            open.RemoveAt(last);

            int i = 0;
            while (true)
            {
                int l = 2 * i + 1, r = l + 1, smallest = i;
                if (l < open.Count && open[l].f < open[smallest].f) smallest = l;
                if (r < open.Count && open[r].f < open[smallest].f) smallest = r;
                if (smallest == i) break;
                (open[smallest], open[i]) = (open[i], open[smallest]);
                i = smallest;
            }
            return top;
        }
    }
}
