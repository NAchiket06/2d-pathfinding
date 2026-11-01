using UnityEditor;
using UnityEngine;

namespace Niket.Pathfinding2D.Editor
{
    /// <summary>Scene visualisation for nodes and their links, plus tangent handles for spline links.</summary>
    [CustomEditor(typeof(Node))]
    public class NodeEditor : UnityEditor.Editor
    {
        const int CurveSegments = 20;

        // Drawn for every node in the scene, selected or not.
        [DrawGizmo(GizmoType.NonSelected | GizmoType.Selected | GizmoType.Pickable)]
        static void DrawNodeGizmo(Node node, GizmoType type)
        {
            NodeParent parent = node.GetComponentInParent<NodeParent>();
            bool selected = (type & GizmoType.Selected) != 0;
            float radius = parent != null ? parent.nodeRadius : 0.15f;

            Gizmos.color = selected ? Color.white : (parent != null ? parent.nodeColor : Color.yellow);
            Gizmos.DrawSphere(node.transform.position, radius);

            if (parent != null && !parent.showLinks) return;

            foreach (NodeLink link in node.Links)
            {
                if (link.target == null) continue;
                DrawLink(node, link, parent, selected || IsSelected(link.target));
            }
        }

        static bool IsSelected(Node n) => Selection.Contains(n.gameObject);

        static void DrawLink(Node node, NodeLink link, NodeParent parent, bool highlight)
        {
            Color baseColor = link.useSpline
                ? (parent != null ? parent.splineLinkColor : Color.cyan)
                : (parent != null ? parent.linkColor : Color.green);

            baseColor.a = .05f;
            Gizmos.color = highlight ? Color.Lerp(baseColor, Color.white, 0.5f) : baseColor;

            Vector3 prev = node.Position;
            int segments = link.useSpline ? CurveSegments : 1;
            for (int i = 1; i <= segments; i++)
            {
                Vector3 p = link.Evaluate(node, i / (float)segments);
                Gizmos.DrawLine(prev, p);
                prev = p;
            }

            // Arrow head near the target so one-way links are readable.
            Vector3 end = link.target.Position;
            Vector3 before = link.Evaluate(node, 0.9f);
            Vector3 dir = (end - before).normalized;
            if (dir.sqrMagnitude > 0f)
            {
                Vector3 side = new Vector3(-dir.y, dir.x, 0f);
                Vector3 tip = link.Evaluate(node, 0.85f);
                float size = 0.15f;
                Gizmos.DrawLine(tip, tip - dir * size + side * size * 0.5f);
                Gizmos.DrawLine(tip, tip - dir * size - side * size * 0.5f);
            }

            if (parent != null && parent.showLabels)
            {
                Vector3 mid = link.Evaluate(node, 0.5f);
                Handles.Label(mid, $"d {link.GetDistance(node):0.0} x {link.weight:0.0}");
            }
        }

        // Tangent handles for spline links on the selected node.
        void OnSceneGUI()
        {
            Node node = (Node)target;
            foreach (NodeLink link in node.Links)
            {
                if (link.target == null || !link.useSpline) continue;

                Vector3 a = node.Position;
                Vector3 b = link.target.Position;

                EditorGUI.BeginChangeCheck();
                Vector3 h1 = Handles.FreeMoveHandle(a + (Vector3)link.startTangent, 0.08f, Vector3.zero, Handles.DotHandleCap);
                Vector3 h2 = Handles.FreeMoveHandle(b + (Vector3)link.endTangent, 0.08f, Vector3.zero, Handles.DotHandleCap);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(node, "Move Link Tangent");
                    link.startTangent = h1 - a;
                    link.endTangent = h2 - b;
                    EditorUtility.SetDirty(node);
                }

                Handles.color = new Color(1f, 1f, 1f, 0.35f);
                Handles.DrawLine(a, h1);
                Handles.DrawLine(b, h2);
            }
        }

        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            Node node = (Node)target;
            EditorGUILayout.Space();
            using (new EditorGUILayout.HorizontalScope())
            {
                if (GUILayout.Button("Link Selected Nodes"))
                    LinkSelected(node);
                if (GUILayout.Button("Clear Links"))
                {
                    Undo.RecordObject(node, "Clear Links");
                    node.ClearLinks();
                }
            }
        }

        // Connect this node to all other selected nodes (bidirectional).
        static void LinkSelected(Node node)
        {
            foreach (GameObject go in Selection.gameObjects)
            {
                Node other = go.GetComponent<Node>();
                if (other == null || other == node) continue;
                Undo.RecordObjects(new Object[] { node, other }, "Link Nodes");
                node.Connect(other);
                EditorUtility.SetDirty(node);
                EditorUtility.SetDirty(other);
            }
        }
    }
}
