# 2D Pathfinding

A small node-graph A* pathfinder for 2D Unity projects, with editor tooling for building the graph in the Scene view.

You place `Node` objects in the scene, link them (optionally with curved spline links and per-link cost weights), and ask `Pathfinder` for the cheapest route between two points.

## Features

- A* search over a hand-built node graph (binary-heap open set, reusable buffers)
- Directional links with a cost multiplier (`cost = distance * weight`)
- Optional Bezier spline links with draggable tangent handles in the Scene view
- Auto-link nodes within a radius, or link selected nodes manually
- Gizmo visualisation of nodes, links, direction arrows and cost labels
- Undo support for all editor actions

## Installation

Copy the `Scripts` folder into your Unity project's `Assets` folder. Scripts under `Editor/` are editor-only and are excluded from builds automatically. Everything lives in the `Niket.Pathfinding2D` namespace.

## Setup

1. Create an empty GameObject and add the **NodeParent** component.
2. Use **Add Node** in its inspector to create child `Node` objects, then move them into position.
3. Link the nodes, either:
   - **Auto Link By Radius** on the NodeParent, which links every pair within `autoLinkRadius` using `autoLinkWeight`, or
   - select several nodes, with the one you want to link from as the active selection, and press **Link Selected Nodes** in its inspector.
4. Fine-tune individual links in each Node's `links` list: `weight`, `useSpline`, `startTangent`, `endTangent`.

### NodeParent inspector

| Button | Action |
| --- | --- |
| Refresh Nodes | Re-collects all child `Node` components |
| Auto Link By Radius | Bidirectionally links all node pairs within `autoLinkRadius` |
| Clear All Links | Removes every link from every node |
| Add Node | Creates a new child node |

Gizmo settings (`showLinks`, `showLabels`, `nodeRadius`, colours) are also on the NodeParent.

### Reading the gizmos

- Arrow heads show link direction. A two-way connection shows an arrow each way.
- Link labels read `d <distance> x <weight>`.
- Spline links use `splineLinkColor`. Select a node to drag the tangent handles for its spline links.
- Links touching a selected node are highlighted.

## Usage

```csharp
using System.Collections.Generic;
using Niket.Pathfinding2D;
using UnityEngine;

public class Walker : MonoBehaviour
{
    [SerializeField] NodeParent graph;

    Pathfinder pathfinder;
    readonly List<Node> path = new List<Node>();

    void Start()
    {
        pathfinder = new Pathfinder(graph);
    }

    public void GoTo(Vector2 target)
    {
        if (pathfinder.TryFindPath(transform.position, target, path, out float cost))
        {
            // path holds the nodes in order, start to end
            foreach (Node n in path) Debug.Log(n.name);
        }
    }
}
```

`TryFindPath` has two overloads:

- `TryFindPath(Vector2 startPos, Vector2 endPos, List<Node> path, out float cost)` snaps each position to the closest node.
- `TryFindPath(Node start, Node end, List<Node> path, out float cost)` uses the nodes you give it.

Both return `false` if no route exists. The `path` list is cleared and reused, so keep one around instead of allocating per search.

### Building graphs from code

```csharp
a.Connect(b);                          // bidirectional, weight 1
a.Connect(b, weight: 3f, bidirectional: false); // one-way, 3x cost
a.Disconnect(b);
```

## Notes

- Nodes are collected in `Awake` and via the **Refresh Nodes** button. If you add nodes at runtime, call `NodeParent.RefreshNodes()`.
- `Pathfinder` reuses internal buffers and is not thread safe. Use one instance per thread.
- The heuristic is straight-line distance scaled by the cheapest link weight (capped at 1), so it never overestimates and paths are optimal. Spline links shorter-than-straight are not possible, so this holds for them too.
- Link weights of 0 are allowed but make the heuristic scale 0, which degrades A* to Dijkstra.

## Files

| File | Purpose |
| --- | --- |
| `Node.cs` | `Node` component and `NodeLink` data |
| `NodeParent.cs` | Node container, closest-node lookup, auto-linking |
| `Pathfinder.cs` | A* search |
| `Editor/NodeEditor.cs` | Node gizmos, tangent handles, inspector buttons |
| `Editor/NodeParentEditor.cs` | NodeParent inspector buttons |
