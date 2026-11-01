using UnityEditor;
using UnityEngine;

namespace Pathfinding2D.Editor
{
    [CustomEditor(typeof(NodeParent))]
    public class NodeParentEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            NodeParent parent = (NodeParent)target;
            EditorGUILayout.Space();
            EditorGUILayout.LabelField($"Nodes: {parent.Nodes.Count}");

            if (GUILayout.Button("Refresh Nodes"))
            {
                Undo.RecordObject(parent, "Refresh Nodes");
                parent.RefreshNodes();
                EditorUtility.SetDirty(parent);
            }

            if (GUILayout.Button("Auto Link By Radius"))
            {
                parent.RefreshNodes();
                RecordNodes(parent, "Auto Link Nodes");
                parent.AutoLink();
                SetDirtyNodes(parent);
            }

            if (GUILayout.Button("Clear All Links"))
            {
                parent.RefreshNodes();
                RecordNodes(parent, "Clear All Links");
                parent.ClearAllLinks();
                SetDirtyNodes(parent);
            }

            if (GUILayout.Button("Add Node"))
            {
                var go = new GameObject("Node");
                Undo.RegisterCreatedObjectUndo(go, "Add Node");
                go.transform.SetParent(parent.transform, false);
                go.AddComponent<Node>();
                Selection.activeGameObject = go;
                parent.RefreshNodes();
            }
        }

        static void RecordNodes(NodeParent parent, string label)
        {
            var objs = new Object[parent.Nodes.Count];
            for (int i = 0; i < objs.Length; i++) objs[i] = parent.Nodes[i];
            Undo.RecordObjects(objs, label);
        }

        static void SetDirtyNodes(NodeParent parent)
        {
            foreach (Node n in parent.Nodes) EditorUtility.SetDirty(n);
        }
    }
}
